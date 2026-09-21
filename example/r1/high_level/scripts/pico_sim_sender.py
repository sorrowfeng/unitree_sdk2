#!/usr/bin/env python3
# ============================================================================
# pico_sim_sender.py
# PICO 模拟发送器：在没有头显的情况下，向机器人端发 UDP-JSON v3 报文。
#
# 用途（配合 example/r1/high_level/r1_dual_arm_loco.cpp --pico）：
#   1. 验证链路：报文 -> 解析 -> 安全闸门 -> 对齐 -> IK -> 关节目标
#   2. 验证安全分支：急停闩锁、safe_to_execute、掉包、stop_signal、return_zero
#   3. 端到端精度：本脚本用官方 URDF 做正运动学生成"机器人系腕部位姿"，
#      再按 r1_xr_pose_alignment.h 的逆变换打包成 OpenXR 位姿发出去。
#      机器人端应把它解回同一组关节角 —— 用 --truth-out 落盘真值即可比对。
#
# 位姿生成（--pose-mode）：
#   wave   关节空间正弦摆动（默认，保证可达）
#   circle 任务空间圆周：零位末端位姿上叠加正弦平移
#   两者都先由 MuJoCo 官方 URDF 做 FK，保证发出的位姿是 R1 可达的。
#
# 场景（--scenario，对应主程序的各条安全分支）：
#   normal      全程 active_stream + safe_to_execute=true
#   estop       T 秒后 emergency_stop_latched=true，且 safe_to_execute 仍为 true
#               ——刻意用最严苛的"两字段不一致"情形：本端必须只看闩锁位就停，
#               不允许依赖 PICO 端同时把 safe_to_execute 也置 false。
#   unsafe      全程 safe_to_execute=false（应不更新臂目标）
#   lost        T 秒后手部 quality=lost（应判 src_valid=false）
#   dropout     T 秒后完全停发（应触发 >600ms 阻尼）
#   stop_signal T 秒后 operator_mode=stop_signal（应触发阻尼）
#   return_zero T 秒后 operator_mode=return_zero（应回零）
#
# 典型用法：
#   # 本机只验证解析+IK（不发包）
#   python3 pico_sim_sender.py --dry-run --duration 3 --print-every 10
#   # 发到机器人（R1 背包）
#   python3 pico_sim_sender.py --host 192.168.123.164 --port 9999 --duration 60
#   # 生成报文后拷到机器人侧回放（无 MuJoCo 的机器也能用）
#   python3 pico_sim_sender.py --dump /tmp/pico.jsonl --truth-out /tmp/truth.jsonl
#   python3 pico_sim_sender.py --replay /tmp/pico.jsonl --host 192.168.123.164
# ============================================================================

from __future__ import annotations

import argparse
import json
import math
import pathlib
import socket
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# 与 r1_xr_pose_alignment.h 严格互逆的坐标变换
# ---------------------------------------------------------------------------

# robotOpenXr()：OpenXR(右手, Y上, Z后, X右) -> Robot(Z上, Y左, X前)
T_RO = np.array([[0.0, 0.0, -1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0]])
# openXrRobot()：T_RO 的逆（纯旋转，逆=转置）
T_OR = np.array([[0.0, -1.0, 0.0],
                 [0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0]])

WAIST_OFFSET = np.array([0.15, 0.0, 0.45])   # worldToWaist 里的头部->腰部偏移
DEFAULT_HEAD_XR = np.array([0.0, 1.5, -0.2])  # 对应 defaultHeadPose()


def _hom(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def head_yaw_rotation(r: np.ndarray) -> np.ndarray:
    """与 headYawRotation() 一致：只取头部 x 轴的水平投影构造偏航系。"""
    x = r[:, 0].copy()
    x[2] = 0.0
    n = np.linalg.norm(x)
    if n < 1e-9:
        return np.eye(3)
    x /= n
    y = np.cross(np.array([0.0, 0.0, 1.0]), x)
    ny = np.linalg.norm(y)
    if ny < 1e-9:
        return np.eye(3)
    y /= ny
    return np.column_stack([x, y, np.array([0.0, 0.0, 1.0])])


def basis_to_robot(pose: np.ndarray) -> np.ndarray:
    """alignWristToRobot 第一步：OpenXR 基系 -> Robot 基系。"""
    return _hom(T_RO, np.zeros(3)) @ pose @ _hom(T_OR, np.zeros(3))


def robot_to_openxr(p_waist: np.ndarray, head_xr: np.ndarray | None = None,
                    mode: str = "head_yaw") -> np.ndarray:
    """alignWristToRobot 的逆：机器人腰部系位姿 -> OpenXR 世界系位姿。

    正变换（kHeadYaw）：
        A = T_RO*P_xr*T_OR ; H = T_RO*head_xr*T_OR
        out.R = Ryaw^T * A.R
        out.t = Ryaw^T * (A.t - H.t) + (0.15, 0, 0.45)
    逆变换即按上式反解。
    """
    head_xr = DEFAULT_HEAD_XR if head_xr is None else np.asarray(head_xr, float)
    h = basis_to_robot(_hom(np.eye(3), head_xr))

    if mode == "head_trans":
        # 只做平移：out.t = A.t - H.t + offset，旋转保持 A.R
        a_t = p_waist[:3, 3] - WAIST_OFFSET + h[:3, 3]
        a_r = p_waist[:3, :3].copy()
    else:
        ryaw = head_yaw_rotation(h[:3, :3])
        a_t = ryaw @ (p_waist[:3, 3] - WAIST_OFFSET) + h[:3, 3]
        a_r = ryaw @ p_waist[:3, :3]

    a = _hom(a_r, a_t)
    return _hom(T_OR, np.zeros(3)) @ a @ _hom(T_RO, np.zeros(3))


def euler_zyx_deg(r: np.ndarray) -> tuple[float, float, float]:
    """R = Rz(yaw)Ry(pitch)Rx(roll) 分解，与 PICO 端约定一致。"""
    pitch = math.asin(float(-np.clip(r[2, 0], -1.0, 1.0)))
    yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
    return math.degrees(pitch), math.degrees(yaw), math.degrees(roll)


def quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵 -> 单位四元数 (x, y, z, w)，与 XrQuaternionf 字段顺序一致。"""
    tr = float(r[0, 0] + r[1, 1] + r[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def pose_fields(p: np.ndarray) -> dict:
    """4x4 位姿 -> 协议里的 pose 字段（位置 + 欧拉角 + 原始四元数）。"""
    pitch, yaw, roll = euler_zyx_deg(p[:3, :3])
    qx, qy, qz, qw = quat_xyzw(p[:3, :3])
    return {
        "position": {"x": float(p[0, 3]), "y": float(p[1, 3]), "z": float(p[2, 3])},
        "orientation": {"pitch": pitch, "yaw": yaw, "roll": roll},
        "orientation_quat": {"x": qx, "y": qy, "z": qz, "w": qw},
    }


# ---------------------------------------------------------------------------
# 位姿生成：用官方 URDF 做 FK，保证发出的位姿是 R1 可达的
# ---------------------------------------------------------------------------

class PoseSource:
    """产生机器人腰部系下的双腕位姿，并同时给出对应的关节角真值。"""

    def __init__(self, variant: str, pose_mode: str):
        root = pathlib.Path(__file__).resolve().parents[4]
        sys.path.insert(0, str(root / "sim"))
        from r1_model import R1Sim  # 延迟导入：--replay 模式不需要 MuJoCo

        self.sim = R1Sim(variant)
        self.variant = variant
        self.n = self.sim.n
        self.mode = pose_mode
        rng = np.random.default_rng(20260921)
        self.freq = rng.uniform(0.20, 0.45, size=2 * self.n)
        self.phase = rng.uniform(0.0, 2 * np.pi, size=2 * self.n)
        # 参考位姿 = 零位（与主程序 q_home 一致），摆动幅度 0.20 rad。
        # 幅度需保证落在关节限位内：越限目标会让 IK 饱和，测出的误差
        # 反映的是"目标不可达"而非求解器精度。
        self.span = 0.20

    def q_at(self, t: float) -> np.ndarray:
        q = self.span * np.sin(2 * np.pi * self.freq * t + self.phase)
        lo, hi = self.sim.limits[:, 0], self.sim.limits[:, 1]
        return np.clip(q, lo + 1e-3, hi - 1e-3)

    def wrist_poses(self, t: float):
        """返回 (左腕 4x4, 右腕 4x4, q 真值)，均为机器人腰部系。"""
        q = self.q_at(t)
        self.sim.set_q(q)
        pl, rl, pr, rr = self.sim.ee_dual()

        if self.mode == "circle":
            # 零位末端位姿上叠加圆周，半径 3cm（远小于臂展，保证可达）
            r = 0.03
            w = 2 * np.pi * 0.3
            dl = np.array([0.0, r * math.sin(w * t), r * (1.0 - math.cos(w * t))])
            dr = np.array([0.0, -r * math.sin(w * t), r * (1.0 - math.cos(w * t))])
            Tl = _hom(rl, pl + dl)
            Tr = _hom(rr, pr + dr)
        else:
            Tl = _hom(rl, pl)
            Tr = _hom(rr, pr)
        return Tl, Tr, q


# ---------------------------------------------------------------------------
# 报文组装
# ---------------------------------------------------------------------------

def build_packet(seq: int, t: float, tl: np.ndarray, tr: np.ndarray,
                 q_true: np.ndarray, cfg) -> dict:
    """组装一份符合 UDP-JSON v3 的完整报文。"""
    scenario = cfg.scenario
    switch_at = cfg.switch_at

    mode = "active_stream"
    safe = True
    estop = False
    quality = "live"
    trigger = 0.35 + 0.35 * math.sin(2 * math.pi * 0.2 * t)
    grip = 0.5 + 0.5 * math.sin(2 * math.pi * 0.15 * t)

    if scenario == "estop" and t >= switch_at:
        # 只置闩锁位、safe_to_execute 保持 true：这是最难的一道。
        # 若接收端只看 safe_to_execute，就会继续执行运动（历史 bug）。
        estop = True
    elif scenario == "unsafe":
        safe = False
    elif scenario == "lost" and t >= switch_at:
        quality = "lost"
    elif scenario == "stop_signal" and t >= switch_at:
        mode = "stop_signal"
        safe = False
    elif scenario == "return_zero" and t >= switch_at:
        mode = "return_zero"
        safe = False

    def side(pose_waist: np.ndarray) -> dict:
        p_xr = robot_to_openxr(pose_waist, cfg.head_xr, cfg.frame)
        return {
            "quality": quality,
            "valid": quality == "live",
            "calibration_applied": True,
            "pose": pose_fields(p_xr),
            "aim_pose": pose_fields(p_xr),
            "input": {
                "trigger": round(trigger, 4),
                "grip": round(grip, 4),
                "thumbstick": {"x": 0.0, "y": 0.0},
                "primary_pressed": False,
                "secondary_pressed": False,
            },
        }

    # 6 路 0..10000 手部关节目标（PICO 端原样透传口径）
    hand_l = [int(x) for x in np.linspace(0, grip * 10000, 6)]
    hand_r = [int(x) for x in np.linspace(0, trigger * 10000, 6)]

    pkt = {
        "sequence": seq,
        "timestamp": round(t * 1000.0, 3),
        "operator_mode": mode,
        "sdk": "PICOHandLink",
        "calibrated": True,
        "safety": {
            "operator_active": True,
            "teleop_armed": True,
            "emergency_stop_latched": estop,
            "session_running": True,
            "session_focused": True,
            "calibration_ready": True,
            "tracking_valid": quality == "live",
            "sample_fresh": True,
            "safe_to_execute": safe,
            "sample_age_ms": 5.0,
            "max_sample_age_ms": 250.0,
            "reason": "ok" if (safe and not estop) else f"sim:{scenario}",
        },
        "teleop": {
            "output_valid": quality == "live",
            "left": side(tl),
            "right": side(tr),
        },
        "controllers": {
            "left": side(tl),
            "right": side(tr),
        },
        "robot_control": {
            "model": "f1",
            "arms": {
                "left": [round(float(v), 6) for v in q_true[: len(q_true) // 2]],
                "right": [round(float(v), 6) for v in q_true[len(q_true) // 2:]],
            },
            "hands": {"left": hand_l, "right": hand_r},
            "head": {"yaw": 0.0, "pitch": 0.0},
            "base": {
                "linear_x": cfg.base_vx,
                "linear_y": cfg.base_vy,
                "angular_z": cfg.base_wz,
            },
        },
    }
    if cfg.with_hmd:
        pkt["hmd"] = {
            "quality": "live",
            "pose": pose_fields(_hom(np.eye(3), cfg.head_xr)),
        }
    return pkt


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def parse_args():
    here = pathlib.Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(
        description="PICO 模拟发送器（UDP-JSON v3）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", default="127.0.0.1", help="目标地址（机器人 IP）")
    ap.add_argument("--port", type=int, default=9999, help="目标端口（默认 9999）")
    ap.add_argument("--variant", default="a5", choices=["a5", "a7"])
    ap.add_argument("--source", default="controllers", choices=["controllers", "teleop"],
                    help="与主程序 --pico-source 保持一致（仅用于提示）")
    ap.add_argument("--frame", default="head_yaw",
                    choices=["head_yaw", "head_trans", "basis"],
                    help="与主程序 --pico-frame 保持一致")
    ap.add_argument("--hz", type=float, default=90.0, help="发送频率（PICO 实机约 90Hz）")
    ap.add_argument("--duration", type=float, default=20.0, help="持续秒数（0=不限时）")
    ap.add_argument("--pose-mode", default="wave", choices=["wave", "circle"])
    ap.add_argument("--scenario", default="normal",
                    choices=["normal", "estop", "unsafe", "lost", "dropout",
                             "stop_signal", "return_zero"])
    ap.add_argument("--switch-at", type=float, default=3.0,
                    help="场景切换时刻（秒），estop/lost/dropout/stop_signal/return_zero 用")
    ap.add_argument("--with-hmd", action="store_true",
                    help="同时发 hmd 位姿（默认不发，走 defaultHeadPose）")
    ap.add_argument("--base-vx", type=float, default=0.0)
    ap.add_argument("--base-vy", type=float, default=0.0)
    ap.add_argument("--base-wz", type=float, default=0.0)
    ap.add_argument("--dump", type=pathlib.Path, help="把报文写入 JSONL（不发包）")
    ap.add_argument("--replay", type=pathlib.Path, help="回放已生成的 JSONL（不需要 MuJoCo）")
    ap.add_argument("--truth-out", type=pathlib.Path, help="记录关节角真值 JSONL")
    ap.add_argument("--dry-run", action="store_true", help="不发送，仅在本机生成")
    ap.add_argument("--print-every", type=int, default=0,
                    help="每 N 个包打印一次（0=不打印）")
    cfg = ap.parse_args()
    cfg.head_xr = DEFAULT_HEAD_XR
    cfg._here = here
    return cfg


def main() -> int:
    cfg = parse_args()

    # ---- 回放模式：直接按文件里的报文原速发送，不需要 MuJoCo ----
    if cfg.replay:
        lines = [ln for ln in cfg.replay.read_text().splitlines() if ln.strip()]
        if not lines:
            print(f"[sim] {cfg.replay} 为空", file=sys.stderr)
            return 1
        sock = None
        if not cfg.dry_run:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        period = 1.0 / cfg.hz
        print(f"[sim] 回放 {len(lines)} 包 -> "
              f"{'dry-run' if cfg.dry_run else f'{cfg.host}:{cfg.port}'} @{cfg.hz:g}Hz")
        try:
            for i, ln in enumerate(lines):
                if sock is not None:
                    sock.sendto(ln.encode("utf-8"), (cfg.host, cfg.port))
                if cfg.print_every and i % cfg.print_every == 0:
                    print(f"[sim] #{i} {ln[:110]}...")
                time.sleep(period)
        except KeyboardInterrupt:
            print("\n[sim] 已中断")
        finally:
            if sock is not None:
                sock.close()
        print(f"[sim] 回放完成，共 {len(lines)} 包")
        return 0

    # ---- 生成模式：FK 造位姿 -> 反推 OpenXR -> 组包 ----
    try:
        source = PoseSource(cfg.variant, cfg.pose_mode)
    except ImportError as e:
        print(f"[sim] 无法导入 sim/r1_model.py（需要 MuJoCo）：{e}\n"
              f"      请用 sim 的 venv 运行，或改用 --replay 模式", file=sys.stderr)
        return 1

    print(f"[sim] variant={cfg.variant} pose={cfg.pose_mode} frame={cfg.frame} "
          f"scenario={cfg.scenario} hz={cfg.hz:g} "
          f"target={'dry-run' if cfg.dry_run else f'{cfg.host}:{cfg.port}'}")

    sock = None
    if not cfg.dry_run and not cfg.dump:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    dump_fh = cfg.dump.open("w", encoding="utf-8") if cfg.dump else None
    truth_fh = cfg.truth_out.open("w", encoding="utf-8") if cfg.truth_out else None

    period = 1.0 / cfg.hz
    seq = 0
    t0 = time.time()
    sent = 0
    dropped = 0
    try:
        while True:
            t = time.time() - t0
            if cfg.duration > 0 and t > cfg.duration:
                break

            # dropout 场景：切换时刻之后完全停发，模拟 PICO 掉线
            if cfg.scenario == "dropout" and t >= cfg.switch_at:
                dropped += 1
                time.sleep(period)
                continue

            tl, tr, q_true = source.wrist_poses(t)
            seq += 1
            pkt = build_packet(seq, t, tl, tr, q_true, cfg)
            line = json.dumps(pkt, separators=(",", ":"))

            if dump_fh:
                dump_fh.write(line + "\n")
            if sock is not None:
                sock.sendto(line.encode("utf-8"), (cfg.host, cfg.port))
            if truth_fh:
                truth_fh.write(json.dumps({
                    "t": round(t, 6),
                    "sequence": seq,
                    "q_true": [float(v) for v in q_true],
                }, separators=(",", ":")) + "\n")
            sent += 1

            if cfg.print_every and seq % cfg.print_every == 0:
                print(f"[sim] t={t:6.2f}s #{seq:5d} mode={pkt['operator_mode']:14s} "
                      f"safe={pkt['safety']['safe_to_execute']} "
                      f"estop={pkt['safety']['emergency_stop_latched']} "
                      f"L={tl[:3, 3].round(3)} R={tr[:3, 3].round(3)}")

            elapsed = time.time() - t0 - t
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\n[sim] 已中断")
    finally:
        if dump_fh:
            dump_fh.close()
        if truth_fh:
            truth_fh.close()
        if sock is not None:
            sock.close()

    print(f"[sim] 完成：发送 {sent} 包" + (f"，停发 {dropped} 拍（dropout）" if dropped else ""))
    if cfg.dump:
        print(f"[sim] 报文已写入 {cfg.dump}（可用 --replay 回放）")
    if cfg.truth_out:
        print(f"[sim] 关节角真值已写入 {cfg.truth_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
