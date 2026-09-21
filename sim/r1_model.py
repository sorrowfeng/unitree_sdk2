"""R1 上半身 MuJoCo 模型封装：加载官方 URDF、关节映射、独立正运动学。

坐标约定（与 example/r1/high_level/r1_arm_ik.h 对齐）：
  URDF 根链 pelvis_link -(waist_yaw_joint)-> waist_yaw_link，
  r1_arm_ik.h 的 R1ArmKinematics 以 waist_yaw_link 系为根（waist_yaw 锁 0，
  与官方 xr_teleoperate 一致）。把 waist_yaw 置 0 后 MuJoCo 世界系与该根系重合，
  故两者 FK 结果可直接比较。

关节链顺序（与 LowCmd 槽位 / 官方 JointArmIndex 一致）：
  a5: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
  a7: 上述 + wrist_pitch, wrist_yaw
EE 定义：a5 = wrist_roll_link 沿本体 x 前移 0.20 m；a7 = wrist_yaw_link 前移 0.05 m。
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
ASSETS = _HERE.parent / "submodules" / "xr_teleoperate" / "assets" / "r1"

SUFFIX = {
    "a5": ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll"],
    "a7": [
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    ],
}
EE_BODY = {"a5": "wrist_roll_link", "a7": "wrist_yaw_link"}
EE_OFFSET = {"a5": 0.20, "a7": 0.05}
SIDES = ("left", "right")


def rot_err_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（度）。"""
    c = (np.trace(Ra @ Rb.T) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


class R1Sim:
    """MuJoCo 中的 R1 上半身：只做运动学（不跑动力学，回放用 qpos 直驱）。"""

    def __init__(self, variant: str = "a5"):
        if variant not in SUFFIX:
            raise ValueError(f"variant 只支持 a5/a7，收到 {variant}")
        self.variant = variant
        self.n = len(SUFFIX[variant])
        self.urdf = ASSETS / f"r1_{variant}.urdf"
        self.model = mujoco.MjModel.from_xml_path(str(self.urdf))
        self.data = mujoco.MjData(self.model)

        self.qadr = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.qadr[name] = int(self.model.jnt_qposadr[i])

        self.joint_names = {
            s: [f"{s}_{k}_joint" for k in SUFFIX[variant]] for s in SIDES
        }
        self.ee_body = {
            s: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_{EE_BODY[variant]}"
            )
            for s in SIDES
        }
        self.ee_offset = EE_OFFSET[variant]

        # 关节限位（链顺序：[左 n, 右 n]），取自 URDF
        lim = []
        for s in SIDES:
            for jn in self.joint_names[s]:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                lim.append(self.model.jnt_range[jid])
        self.limits = np.array(lim, dtype=float)

        self.reset()

    # ---------- 状态设置 ----------

    def reset(self, q: np.ndarray | None = None) -> None:
        """把腰/头归零（与官方一致），手臂设为 q（链顺序，长度 2n）。"""
        self.data.qpos[:] = 0.0
        if q is not None:
            self.set_q(q)
        else:
            mujoco.mj_kinematics(self.model, self.data)

    def set_q(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float).ravel()
        if q.size != 2 * self.n:
            raise ValueError(f"q 长度应为 {2 * self.n}，收到 {q.size}")
        for idx, s in enumerate(SIDES):
            base = 0 if s == "left" else self.n
            for k, jn in enumerate(self.joint_names[s]):
                self.data.qpos[self.qadr[jn]] = q[base + k]
        mujoco.mj_kinematics(self.model, self.data)

    # ---------- 运动学 ----------

    def ee(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        """返回该侧 EE 的 (位置 3, 旋转 3x3)，含官方手安装偏移。"""
        bid = self.ee_body[side]
        R = self.data.xmat[bid].reshape(3, 3).copy()
        p = self.data.xpos[bid].copy() + R @ np.array([self.ee_offset, 0.0, 0.0])
        return p, R

    def ee_dual(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pl, Rl = self.ee("left")
        pr, Rr = self.ee("right")
        return pl, Rl, pr, Rr

    # ---------- 采样 ----------

    def sample_q(
        self,
        rng: np.random.Generator,
        margin_frac: float = 0.12,
        center: np.ndarray | None = None,
        span: float | None = None,
    ) -> np.ndarray:
        """采样关节角。

        - span=None：在限位内均匀采样（去掉 margin_frac 的边缘余量，避开卡限位）。
        - center/span 给定：在 center 附近按 +-span 截断采样（模拟真实遥操工作空间）。
        """
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        if center is not None:
            c = np.asarray(center, dtype=float).ravel()
            q = c + rng.uniform(-span, span, size=c.size)
            return np.clip(q, lo, hi)
        m = margin_frac * (hi - lo)
        return rng.uniform(lo + m, hi - m)
