#!/usr/bin/env python3
"""官方 R1_A5/A7 的 IK 目标函数  vs  我方 r1_arm_ik.h 的 DLS —— 同输入对照。

背景
----
官方的 IK 是 CasADi + IPOPT 求解
    min  50*||p_ee(q) − p_t||² + c_rot*||log3(R_ee(q)·R_tᵀ)||² + 0.02*||q||² + 0.1*||q − q_last||²
    s.t. 关节限位
我方是阻尼最小二乘（DLS）迭代逼近同一目标。要回答的问题是：
**"我方求解器的行为是否与官方一致"，具体到"手往下放时位置跟不住"这一现象，
   是官方目标函数本身的性质，还是我方移植引入的偏差。**

做法
----
* 模型：直接用官方 URDF（submodules/xr_teleoperate/assets/r1）经 pinocchio 构建，
  与官方 R1_A5_ArmIK.__init__ 里一模一样的 reduced model + EE frame。
  → 先做 FK 一致性校验（我方 C++ FK vs 官方 pinocchio FK），不一致则后面无意义。
* 权重：从官方 robot_arm_ik.py 里正则抓取（50 / c_rot / 0.02 / 0.1），不写死。
* 求解：官方 IPOPT 的 casadi 绑定在 PyPI wheel 里没有（`pinocchio.casadi` 不可用），
  故目标函数与模型保持官方原样，只把 NLP 求解器换成 scipy SLSQP，
  并把迭代预算也对齐官方（max_iter=30 / tol=1e-4），另跑一组收敛版作参考。
* 我方：通过 sim/build/bridge_ik（行协议）调用 r1_arm_ik.h 的 DLS 单臂求解。

只解左臂。官方是双臂联合优化，但目标函数与约束对两臂完全可分，
所以"固定右臂 + 只解左臂"与官方联合求解的左臂分量在数学上等价。

用法：python3 sim/check_ik_vs_official.py [a5|a7]
"""
from __future__ import annotations

import pathlib
import re
import subprocess
import sys

import numpy as np
import pinocchio as pin
from scipy.optimize import minimize

ROOT = pathlib.Path(__file__).resolve().parent.parent
OFFICIAL_IK = ROOT / "submodules/xr_teleoperate/teleop/robot_control/robot_arm_ik.py"
ASSETS = ROOT / "submodules/xr_teleoperate/assets/r1"
BRIDGE = ROOT / "sim/build/bridge_ik"

# 目标位置（左腕，腰部/骨盆系）—— 与 sim/probe_ik_solver.cpp 同一组，便于横向比较
X, Y = 0.25, 0.15
Z_SWEEP = [0.60, 0.50, 0.40, 0.30, 0.20, 0.10, 0.00, -0.10, -0.20, -0.28]


# --------------------------------------------------------------------------
# 官方权重：从源码抓，避免写死
# --------------------------------------------------------------------------
def official_weights(variant: str) -> dict:
    src = OFFICIAL_IK.read_text()
    # 找到 "class R1_A5_ArmIK:" 到下一个 class 之间的 opti.minimize
    m = re.search(rf"class R1_{variant.upper()}_ArmIK:.*?(?=\nclass |\Z)", src, re.S)
    if not m:
        raise SystemExit(f"未在官方源码里找到 R1_{variant.upper()}_ArmIK")
    body = m.group(0)
    mm = re.search(r"opti\.minimize\(\s*([0-9.]+)\s*\*\s*self\.translational_cost\s*\+\s*"
                   r"(?:([0-9.]+)\s*\*\s*)?self\.rotation_cost\s*\+\s*"
                   r"(?:([0-9.]+)\s*\*\s*)?self\.regularization_cost\s*\+\s*"
                   r"(?:([0-9.]+)\s*\*\s*)?self\.smooth_cost", body)
    if not mm:
        raise SystemExit("未能解析官方 optimize 权重")
    # 省略系数时官方写的是裸项，系数即 1.0
    w_pos, w_rot, w_reg, w_smooth = (float(g) if g else 1.0 for g in mm.groups())
    # EE 偏移 + 挂载关节
    me = re.search(r"addFrame\(\s*[^)]*?'L_ee',\s*\n?\s*[^,]*getJointId\('(\w+)'\)[^,]*,\s*\n?\s*"
                   r"pin\.SE3\(np\.eye\(3\),\s*\n?\s*np\.array\(\[([0-9.]+),0,0\]\)", body)
    ee_joint, ee_off = (me.group(1), float(me.group(2))) if me else (None, None)
    return {"w_pos": w_pos, "w_rot": w_rot, "w_reg": w_reg, "w_smooth": w_smooth,
            "ee_joint": ee_joint, "ee_off": ee_off}


# --------------------------------------------------------------------------
# 官方模型（与 R1_A5_ArmIK.__init__ 同构）
# --------------------------------------------------------------------------
class OfficialModel:
    def __init__(self, variant: str, w: dict):
        self.variant = variant
        urdf = ASSETS / f"r1_{variant}.urdf"
        robot = pin.RobotWrapper.BuildFromURDF(str(urdf), str(ASSETS) + "/")
        self.red = robot.buildReducedRobot(
            list_of_joints_to_lock=["waist_yaw_joint", "head_pitch_joint", "head_yaw_joint"],
            reference_configuration=np.array([0.0] * robot.model.nq))
        n = self.red.model.nq // 2
        off = w["ee_off"]
        jn_l = w["ee_joint"]                      # 官方挂载关节，如 left_wrist_roll_joint
        jn_r = jn_l.replace("left_", "right_", 1)
        assert self.red.model.existJointName(jn_l) and self.red.model.existJointName(jn_r), \
            f"reduced model 里没有 {jn_l} / {jn_r}"
        for nm, jn in (("L_ee", jn_l), ("R_ee", jn_r)):
            self.red.model.addFrame(pin.Frame(
                nm, self.red.model.getJointId(jn),
                pin.SE3(np.eye(3), np.array([off, 0, 0]).T), pin.FrameType.OP_FRAME))
        self.data = self.red.model.createData()
        self.L = self.red.model.getFrameId("L_ee")
        self.lo = np.array(self.red.model.lowerPositionLimit[:n])
        self.hi = np.array(self.red.model.upperPositionLimit[:n])
        self.n = n

    def fk_left(self, q):
        qq = np.zeros(2 * self.n)
        qq[:self.n] = q
        pin.framesForwardKinematics(self.red.model, self.data, qq)
        M = self.data.oMf[self.L]
        t = M.translation
        R = M.rotation
        # ⚠️ 必须 copy：pinocchio 4.x 的 SE3.translation / .rotation 返回的是
        # 直接指向内部缓冲区的 numpy 视图（或返回视图的方法）。不 copy 而把结果
        # 攒进列表时，所有元素都会别名到**最后一次** FK 的结果 —— 表现为一批
        # 不同目标解出完全相同的误差（踩过一次）。
        t = t() if callable(t) else t
        R = R() if callable(R) else R
        return (np.array(t, float).ravel().copy(), np.array(R, float).copy())


def geodesic(Ra, Rb):
    c = (np.trace(Ra @ Rb.T) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


# --------------------------------------------------------------------------
# 我方求解器（行协议批量调用）
# --------------------------------------------------------------------------
class Bridge:
    def __init__(self):
        if not BRIDGE.exists():
            subprocess.run(["clang++", "-std=c++17", "-O2", "-I", "sim/build/inc",
                            "-I", "example/r1/high_level", "sim/bridge_ik.cpp",
                            "-o", "sim/build/bridge_ik"], cwd=ROOT, check=True)

    def run(self, lines):
        p = subprocess.run([str(BRIDGE)], input="\n".join(lines) + "\n",
                           capture_output=True, text=True, check=True)
        return p.stdout.strip().splitlines()

    def fk(self, variant, q):
        v = [float(x) for x in self.run([f"FK {variant} " + " ".join(map(str, q))])[0].split()[1:]]
        return (np.array(v[0:3]), q2R(v[3:7]), np.array(v[7:10]), q2R(v[10:14]))

    def ik_left(self, variant, p_t, q0, R_t=None):
        R_t = np.eye(3) if R_t is None else R_t
        if np.allclose(R_t, np.eye(3)):
            line = f"IKL {variant} {p_t[0]} {p_t[1]} {p_t[2]} " + " ".join(map(str, q0))
        else:
            flat = " ".join(str(R_t[r, c]) for r in range(3) for c in range(3))
            line = f"IKL2 {variant} {p_t[0]} {p_t[1]} {p_t[2]} {flat} " + " ".join(map(str, q0))
        parts = self.run([line])[0].split()
        v = [float(x) for x in parts[1:]]
        return np.array(v[:len(q0)]), v[len(q0)], v[len(q0) + 1]

    def ik_left_batch(self, variant, tasks):
        """tasks: [(p_t, q0, R_t|None)] -> [(q, pos_err_mm, rot_err_deg)]"""
        lines, n = [], len(tasks[0][1])
        for p_t, q0, R_t in tasks:
            R_t = np.eye(3) if R_t is None else R_t
            flat = " ".join(str(R_t[r, c]) for r in range(3) for c in range(3))
            lines.append(f"IKL2 {variant} {p_t[0]} {p_t[1]} {p_t[2]} {flat} "
                         + " ".join(map(str, q0)))
        out = []
        for s in self.run(lines):
            v = [float(x) for x in s.split()[1:]]
            out.append((np.array(v[:n]), v[n], v[n + 1]))
        return out


def q2R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------------------
# 官方目标函数（左臂，5 变量）
# --------------------------------------------------------------------------
def make_cost(model, w, p_t, R_t, q_prev):
    def cost(q):
        p, R = model.fk_left(q)
        e_p = p - p_t
        e_r = geodesic(R, R_t)
        return (w["w_pos"] * float(e_p @ e_p) + w["w_rot"] * e_r ** 2
                + w["w_reg"] * float(q @ q) + w["w_smooth"] * float((q - q_prev) @ (q - q_prev)))
    return cost


def solve_official(model, w, p_t, q0, R_t=None, maxiter=30):
    R_t = np.eye(3) if R_t is None else R_t
    res = minimize(make_cost(model, w, p_t, R_t, q0), q0, method="SLSQP",
                   bounds=list(zip(model.lo, model.hi)),
                   options={"maxiter": maxiter, "ftol": 1e-4, "disp": False})
    q = np.clip(res.x, model.lo, model.hi)
    p, R = model.fk_left(q)
    return q, float(np.linalg.norm(p - p_t) * 1000), float(np.degrees(geodesic(R, R_t)))


# --------------------------------------------------------------------------
def main():
    variant = (sys.argv[1] if len(sys.argv) > 1 else "a5").lower()
    w = official_weights(variant)
    print("=== 官方权重（从 robot_arm_ik.py 抓取，非写死）===")
    print(f"  min  {w['w_pos']}*pos² + {w['w_rot']}*rot² + {w['w_reg']}*||q||² "
          f"+ {w['w_smooth']}*||q−q_last||²")
    print(f"  权重比 w_rot/w_pos = {w['w_rot'] / w['w_pos']:.4f}  "
          f"（1 rad 姿态误差 ≈ {1000 * np.sqrt(w['w_rot'] / w['w_pos']):.0f} mm 位置误差）")
    print(f"  EE = {w['ee_joint']} + x {w['ee_off']} m\n")

    model = OfficialModel(variant, w)
    br = Bridge()
    n = model.n

    # ---------- ① FK 一致性 ----------
    print("=== ① FK 一致性：我方 C++ FK  vs  官方 pinocchio FK ===")
    rng = np.random.default_rng(7)
    wp = wr = 0.0
    for k in range(5):
        q = rng.uniform(-1.0, 1.0, 2 * n)
        p_l, R_l, p_r, R_r = br.fk(variant, q)
        pin.framesForwardKinematics(model.red.model, model.data, q)
        Mo, Mr = model.data.oMf[model.L], model.data.oMf[model.red.model.getFrameId("R_ee")]
        to = lambda M: np.asarray(M.translation, float).ravel()
        Ro = lambda M: np.asarray(M.rotation, float)
        dp = max(np.linalg.norm(to(Mo) - p_l), np.linalg.norm(to(Mr) - p_r))
        da = max(geodesic(Ro(Mo), R_l), geodesic(Ro(Mr), R_r))
        wp, wr = max(wp, dp), max(wr, da)
    print(f"  5 组随机关节角：最大位置偏差 {wp:.2e} m，最大姿态偏差 {np.degrees(wr):.2e}°"
          f"  → {'一致' if wp < 1e-9 and np.degrees(wr) < 1e-3 else '不一致 ✗'}\n")

    # ---------- ② 目标 z 扫描：官方目标函数 vs 我方 DLS ----------
    print(f"=== ② 目标 z 由高到低（x={X} y={Y}，目标姿态=单位阵，warm start 接力）===")
    print(f"{'目标z':>7} | {'官方目标函数(SLSQP,30iter)':^31} | {'官方目标函数(收敛版)':^31} | "
          f"{'我方 DLS(30iter, 四项目标)':^31}")
    print(f"{'':>7} | {'位置mm':>10}{'姿态°':>9}{'肘°':>11} | "
          f"{'位置mm':>10}{'姿态°':>9}{'肘°':>11} | {'位置mm':>10}{'姿态°':>9}{'肘°':>11}")
    print("-" * 116)

    qo_budget = np.zeros(n)
    qo_conv = np.zeros(n)
    qm = np.zeros(n)
    rows = []
    for z in Z_SWEEP:
        p_t = np.array([X, Y, z])
        qo_budget, eo_b, ro_b = solve_official(model, w, p_t, qo_budget, maxiter=30)
        qo_conv, eo_c, ro_c = solve_official(model, w, p_t, qo_conv, maxiter=200)
        qm, em, rm = br.ik_left(variant, p_t, qm)
        rows.append((z, eo_b, ro_b, qo_budget[3], eo_c, ro_c, qo_conv[3], em, rm, qm[3]))
        print(f"{z:+7.2f} | {eo_b:10.2f}{ro_b:9.2f}{np.degrees(qo_budget[3]):11.1f} | "
              f"{eo_c:10.2f}{ro_c:9.2f}{np.degrees(qo_conv[3]):11.1f} | "
              f"{em:10.2f}{rm:9.2f}{np.degrees(qm[3]):11.1f}")

    # ---------- ③ 多点随机初值：区分「局部极小」与「该目标真做不到」 ----------
    print(f"\n=== ③ 每点 {40} 个随机初值，看位置误差能做到的最小值 ===")
    print("  （若官方目标函数的最小值也很小 → 位置是可保的，是我方求解器不到位；")
    print("    若两者都大 → 是官方目标函数在 5 自由度下的结构性质）")
    print(f"{'目标z':>7} | {'官方目标函数 位置err min/中位 mm':>34} | {'我方 DLS 位置err min/中位 mm':>30}")
    print("-" * 80)
    rng = np.random.default_rng(11)
    starts = [rng.uniform(model.lo, model.hi) for _ in range(40)]
    for z in Z_SWEEP:
        p_t = np.array([X, Y, z])
        eo = []
        for s in starts:
            _, e, _ = solve_official(model, w, p_t, s, maxiter=200)
            eo.append(e)
        tasks = [(p_t, s, None) for s in starts]
        em = [t[1] for t in br.ik_left_batch(variant, tasks)]
        eo, em = np.sort(eo), np.sort(em)
        print(f"{z:+7.2f} | {eo[0]:15.2f} / {np.median(eo):14.2f} "
              f"| {em[0]:12.2f} / {np.median(em):14.2f}")

    # ---------- ④ 结论 ----------
    print("\n=== ④ 判据 ===")
    eo_b = np.array([r[1] for r in rows])
    eo_c = np.array([r[4] for r in rows])
    em = np.array([r[7] for r in rows])
    print(f"  官方目标函数（30 iter，对齐 IPOPT 预算）位置误差最大 {eo_b.max():.1f} mm")
    print(f"  官方目标函数（收敛版）                   位置误差最大 {eo_c.max():.1f} mm")
    print(f"  我方 DLS（30 iter，四项目标）             位置误差最大 {em.max():.1f} mm")
    d = np.abs(em - eo_c)
    verdict = ("我方 DLS 与官方目标函数的解一致 → 移植无偏差"
               if d.max() < 5.0 else
               f"两者偏差最大 {d.max():.1f} mm → 我方 DLS 有额外损失，需对齐求解器")
    print(f"  逐点 |我方 − 官方收敛版| 最大 {d.max():.2f} mm  → {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
