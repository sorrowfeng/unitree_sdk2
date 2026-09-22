"""探测 R1 A5 左臂在「腰部系」下的可达范围，重点看肘关节行为。

思路：不走 IK 正向猜测，而是**指定想让手腕到达的腰部系位置**，
用 alignWristToRobot 的逆变换反算成 PICO 报文位姿，喂给真实管线
（sim/build/pico_pipeline_test，与 r1_dual_arm_loco.cpp 同源），
再用 MuJoCo（独立 FK，真值）核验实际到达位置。

用法：python sim/probe_reach.py [--x 0.25] [--y 0.15] [--z0 0.6] [--z1 -0.45]
"""

import argparse
import json
import math
import pathlib
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from r1_model import R1Sim  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent

# --- 与 r1_xr_pose_alignment.h 完全一致的常量与变换 ---
T_RO = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
T_OR = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])
WAIST_OFFSET = np.array([0.15, 0.0, 0.45])


def hom(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def basis_to_robot(pose):
    return hom(T_RO, np.zeros(3)) @ pose @ hom(T_OR, np.zeros(3))


def head_yaw_rotation(R):
    x = R[:, 0].copy()
    x[2] = 0.0
    n = np.linalg.norm(x)
    if n < 1e-9:
        return np.eye(3)
    x /= n
    y = np.cross(np.array([0.0, 0.0, 1.0]), x)
    ny = np.linalg.norm(y)
    if ny < 1e-9:
        return np.eye(3)
    return np.column_stack([x, y / ny, np.array([0.0, 0.0, 1.0])])


def robot_to_openxr(p_waist, head_xr):
    """alignWristToRobot(kHeadYaw) 的逆：腰部系目标 -> OpenXR 腕部位姿。"""
    h = basis_to_robot(hom(np.eye(3), np.asarray(head_xr, float)))
    ryaw = head_yaw_rotation(h[:3, :3])
    a_t = ryaw @ (p_waist[:3, 3] - WAIST_OFFSET) + h[:3, 3]
    a_r = ryaw @ p_waist[:3, :3]
    return hom(T_OR, np.zeros(3)) @ hom(a_r, a_t) @ hom(T_RO, np.zeros(3))


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


def pose_fields(T):
    R, t = T[:3, :3], T[:3, 3]
    pitch = math.degrees(math.asin(float(-np.clip(R[2, 0], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    qx, qy, qz, qw = mat_to_quat(R)
    return {
        "position": {"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
        "orientation": {"pitch": pitch, "yaw": yaw, "roll": roll},
        "orientation_quat": {"x": qx, "y": qy, "z": qz, "w": qw},
    }


SAFETY = {
    "operator_active": True, "teleop_armed": True, "emergency_stop_latched": False,
    "session_running": True, "session_focused": True, "calibration_ready": True,
    "tracking_valid": True, "sample_fresh": True, "safe_to_execute": True,
    "sample_age_ms": 0, "max_sample_age_ms": 250, "reason": "ok",
}


def make_packet(seq, Lxr, Rxr, hmd_xr):
    return json.dumps({
        "packet_version": 3, "sequence": seq, "timestamp": seq,
        "operator_mode": "active_stream", "safety": SAFETY,
        "controllers": {
            "left": {"quality": "live", "pose": pose_fields(Lxr)},
            "right": {"quality": "live", "pose": pose_fields(Rxr)},
        },
        "hmd": {"quality": "live", "pose": pose_fields(hom(np.eye(3), np.asarray(hmd_xr, float)))},
    })


def main():
    ap = argparse.ArgumentParser(description="探测 R1 A5 左臂可达范围与肘关节行为")
    ap.add_argument("--x", type=float, default=0.25, help="目标手腕 x（前方，腰部系）")
    ap.add_argument("--y", type=float, default=0.15, help="目标手腕 y（左正，腰部系）")
    ap.add_argument("--z0", type=float, default=0.60, help="z 扫描起点")
    ap.add_argument("--z1", type=float, default=-0.45, help="z 扫描终点")
    ap.add_argument("--step", type=float, default=0.02)
    args = ap.parse_args()

    hmd_xr = np.zeros(3)  # 与实机一致：LOCAL space，头显在原点
    zs = np.arange(args.z0, args.z1 - 1e-9, -args.step)

    packets = []
    for i, z in enumerate(zs):
        p_l = hom(np.eye(3), [args.x, args.y, z])
        p_r = hom(np.eye(3), [args.x, -args.y, z])
        packets.append(make_packet(i + 1, robot_to_openxr(p_l, hmd_xr),
                                   robot_to_openxr(p_r, hmd_xr), hmd_xr))

    exe = ROOT / "sim" / "build" / "pico_pipeline_test"
    if not exe.exists():
        raise SystemExit(f"找不到 {exe}，先构建 sim（见 sim/README.md）")

    proc = subprocess.run([str(exe), "--variant", "a5", "--raw"],
                          input="\n".join(packets) + "\n",
                          capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"管线退出码 {proc.returncode}\n{proc.stderr[-800:]}")

    qs = []
    for line in proc.stdout.splitlines():
        if " q=" not in line:
            continue
        qs.append([float(v) for v in line.split(" q=")[1].split()])
    if len(qs) != len(zs):
        raise SystemExit(f"管线返回 {len(qs)} 帧，期望 {len(zs)} 帧")

    sim = R1Sim("a5")
    lo, hi = sim.limits[:, 0], sim.limits[:, 1]  # 链顺序：[左 n, 右 n]

    print(f"固定 x={args.x:+.2f} y={args.y:+.2f}（左腕），目标姿态=单位朝向，z 从 {args.z0:+.2f} 降到 {args.z1:+.2f}")
    print("报文用 LOCAL space（hmd 在原点）；求解走 --raw（跳过 WMA，看裸解）\n")
    print(f"{'目标z':>7} {'实际z':>7} {'位置误差':>9} {'左肘':>8} {'肘距上限':>8} "
          f"{'肩pitch':>8} {'肩roll':>8} {'肩yaw':>8} {'腕roll':>8}")
    print("-" * 82)
    for i, z in enumerate(zs):
        q = np.array(qs[i])
        sim.set_q(q)
        pl, _, _, _ = sim.ee_dual()
        err = float(np.linalg.norm(pl - np.array([args.x, args.y, z]))) * 1000.0
        elbow = math.degrees(q[3])
        elbow_to_hi = math.degrees(hi[3] - q[3])
        elbow_to_lo = math.degrees(q[3] - lo[3])
        mark = ""
        if err > 5.0:
            mark = "  <-- 跟不上"
        if elbow_to_hi < 2.0:
            mark += " [肘顶上限]"
        if elbow_to_lo < 2.0:
            mark += " [肘顶下限]"
        print(f"{z:+7.2f} {pl[2]:+7.3f} {err:8.1f}mm {elbow:+8.1f} {elbow_to_hi:8.1f} "
              f"{math.degrees(q[0]):+8.1f} {math.degrees(q[1]):+8.1f} "
              f"{math.degrees(q[2]):+8.1f} {math.degrees(q[4]):+8.1f}{mark}")

    print("\n肘关节 URDF 限位（A5，左）： "
          f"{math.degrees(lo[3]):+.1f}° .. {math.degrees(hi[3]):+.1f}°")


if __name__ == "__main__":
    main()
