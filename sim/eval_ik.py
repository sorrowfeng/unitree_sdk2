#!/usr/bin/env python3
"""L1 精度验证：把 r1_arm_ik.h 的 IK 结果交给 MuJoCo 用同一份 URDF 独立求正解。

流程（每项都避免"用自家 FK 验证自家 IK"）：
  1. FK 交叉校验：随机关节角 → C++ FK vs MuJoCo FK（应一致到数值精度）
  2. IK 精度：MuJoCo FK 生成目标 EE 位姿 → C++ IK 求解 → MuJoCo FK 复算实际位姿 → 残差
  3. 轨迹跟踪：关节空间平滑轨迹（当作"人手运动"）逐帧下发 → 逐帧 IK → 跟踪误差，
     并对比 r1_arm_ik.h 内置 WMA 平滑（ik）与不平滑（ik-raw）的差异

用法：
  python sim/eval_ik.py                      # a5 + a7 全跑
  python sim/eval_ik.py --variant a5 --n-ik 500
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from r1_model import R1Sim, SIDES, rot_err_deg  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
PROBE = HERE / "build" / "ik_probe"


def run_probe(variant: str, mode: str, rows: list[list[float]]) -> np.ndarray:
    """把一批查询喂给 C++ 探针，返回扁平结果数组。"""
    if not PROBE.exists():
        raise SystemExit(f"探针未编译：{PROBE}\n先运行 sim/build_probe.sh")
    text = "\n".join(" ".join(f"{v:.17g}" for v in r) for r in rows) + "\n"
    out = subprocess.run(
        [str(PROBE), "--variant", variant, "--mode", mode],
        input=text,
        capture_output=True,
        text=True,
    )
    if out.returncode != 0:
        raise SystemExit(f"探针失败({out.returncode})：{out.stderr.strip()[:400]}")
    res = [np.array([float(x) for x in ln.split()]) for ln in out.stdout.strip().splitlines()]
    return np.array(res)


class ProbeSession:
    """与探针建立长连接，逐帧交互（用于 warm start 的轨迹跟踪）。

    注意：必须同一进程内顺序调用，R1DualArmIk 的 WMA 平滑窗口才与实机一致。
    """

    def __init__(self, variant: str, mode: str):
        if not PROBE.exists():
            raise SystemExit(f"探针未编译：{PROBE}\n先运行 sim/build_probe.sh")
        self.proc = subprocess.Popen(
            [str(PROBE), "--variant", variant, "--mode", mode],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )

    def __enter__(self) -> "ProbeSession":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def query(self, row: list[float]) -> np.ndarray:
        assert self.proc.stdin and self.proc.stdout
        self.proc.stdin.write(" ".join(f"{v:.17g}" for v in row) + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise SystemExit(f"探针提前退出：{self.proc.stderr and self.proc.stderr.read()}")
        return np.array([float(x) for x in line.split()])

    def close(self) -> None:
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()


def quat_to_R(x: np.ndarray) -> np.ndarray:
    """(x,y,z,w) → 3x3 旋转矩阵。"""
    w, x, y, z = x[3], x[0], x[1], x[2]
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_row(p: np.ndarray, R: np.ndarray) -> list[float]:
    """EE 位姿 → 探针输入行 [px py pz qx qy qz qw]。"""
    q = _R_to_quat(R)
    return [p[0], p[1], p[2], q[0], q[1], q[2], q[3]]


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def stats(a: np.ndarray) -> dict:
    a = np.asarray(a, dtype=float)
    return {
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "max": float(a.max()),
    }


def eval_fk_crosscheck(sim: R1Sim, rng: np.random.Generator, n: int) -> dict:
    """随机关节角下，C++ FK 与 MuJoCo FK 的一致性。"""
    qs = np.stack([sim.sample_q(rng, margin_frac=0.02) for _ in range(n)])
    cpp = run_probe(sim.variant, "fk", [list(q) for q in qs])
    dpos, drot = [], []
    for q, row in zip(qs, cpp):
        sim.set_q(q)
        for i, side in enumerate(SIDES):
            p, R = sim.ee(side)
            cp = row[i * 7 : i * 7 + 3]
            cR = quat_to_R(row[i * 7 + 3 : i * 7 + 7])
            dpos.append(np.linalg.norm(p - cp) * 1000.0)  # mm
            drot.append(rot_err_deg(R, cR))
    return {"n": n, "pos_mm": stats(np.array(dpos)), "rot_deg": stats(np.array(drot))}


def eval_ik(sim: R1Sim, rng: np.random.Generator, n: int, center=None, span=None,
            tag: str = "") -> dict:
    """随机目标位姿下的 IK 精度（真值 = MuJoCo FK）。"""
    n_arm = sim.n
    q_true = np.stack([sim.sample_q(rng, center=center, span=span) for _ in range(n)])
    q_cur = np.zeros(2 * n_arm)

    rows, info = [], []
    for qt in q_true:
        sim.set_q(qt)
        pl, Rl, pr, Rr = sim.ee_dual()
        rows.append(pose_row(pl, Rl) + pose_row(pr, Rr) + list(q_cur))
        info.append((pl, Rl, pr, Rr))

    sol = run_probe(sim.variant, "ik-raw", rows)

    err_pos = {"left": [], "right": []}
    err_rot = {"left": [], "right": []}
    joint_dev = []
    limit_hits = 0
    for qs_, (pl, Rl, pr, Rr) in zip(sol, info):
        sim.set_q(qs_)
        for side, (pt, Rt) in zip(SIDES, ((pl, Rl), (pr, Rr))):
            p, R = sim.ee(side)
            err_pos[side].append(np.linalg.norm(p - pt) * 1000.0)
            err_rot[side].append(rot_err_deg(R, Rt))
        lo, hi = sim.limits[:, 0], sim.limits[:, 1]
        limit_hits += int(np.any((qs_ < lo + 1e-6) | (qs_ > hi - 1e-6)))
        joint_dev.append(np.abs(qs_ - np.asarray(q_true[len(joint_dev)])).max())

    ep = np.array(err_pos["left"] + err_pos["right"])
    er = np.array(err_rot["left"] + err_rot["right"])
    return {
        "tag": tag,
        "n": n,
        "pos_mm": stats(ep),
        "rot_deg": stats(er),
        "success_5mm": float((ep < 5.0).mean()),
        "success_10mm": float((ep < 10.0).mean()),
        "joint_dev_max_rad": stats(np.array(joint_dev)),
        "limit_hit_frac": float(limit_hits / n),
        "per_side": {
            s: {"pos_mm": stats(np.array(err_pos[s])), "rot_deg": stats(np.array(err_rot[s]))}
            for s in SIDES
        },
        "_err_pos": ep,
        "_err_rot": er,
    }


def eval_trajectory(sim: R1Sim, rng: np.random.Generator, seconds: float = 6.0,
                    hz: float = 100.0, margin: float = 0.05) -> dict:
    """关节空间平滑轨迹逐帧下发（模拟人手运动），对比平滑/不平滑两种出参。

    轨迹以**限位中点**为基准并留 margin 余量，保证真值始终可达——
    否则误差反映的是"目标越限"而非求解精度。
    """
    n_arm = sim.n
    steps = int(seconds * hz)
    t = np.arange(steps) / hz
    lo, hi = sim.limits[:, 0], sim.limits[:, 1]
    center = 0.5 * (lo + hi)
    amp = np.minimum(0.30 * (hi - lo) / 2.0, 0.45)
    freq = rng.uniform(0.18, 0.45, size=2 * n_arm)
    phase = rng.uniform(0, 2 * np.pi, size=2 * n_arm)
    q_true = center + amp * np.sin(2 * np.pi * freq * t[:, None] + phase[None, :])
    q_true = np.clip(q_true, lo + margin, hi - margin)

    rows, targets = [], []
    for i in range(steps):
        sim.set_q(q_true[i])
        pl, Rl, pr, Rr = sim.ee_dual()
        targets.append((pl, Rl, pr, Rr))
        rows.append(pose_row(pl, Rl) + pose_row(pr, Rr) + [0.0] * (2 * n_arm))

    # 逐帧预热：把上一帧解作为下一帧 warm start（第 0 帧用零位，模拟"站立位起步"）
    def solve_seq(mode: str) -> np.ndarray:
        cur = np.zeros(2 * n_arm)
        out = []
        with ProbeSession(sim.variant, mode) as sess:
            for r in rows:
                line = list(r)
                line[14:] = list(cur)
                q = sess.query(line)
                out.append(q)
                cur = q
        return np.array(out)

    res = {}
    for mode in ("ik-raw", "ik"):
        qs = solve_seq(mode)
        ep, er, ee_path = [], [], []
        sat = 0
        for q, (pl, Rl, pr, Rr) in zip(qs, targets):
            sim.set_q(q)
            pl2, Rl2, pr2, Rr2 = sim.ee_dual()
            for p2, R2, pt, Rt in ((pl2, Rl2, pl, Rl), (pr2, Rr2, pr, Rr)):
                ep.append(np.linalg.norm(p2 - pt) * 1000.0)
                er.append(rot_err_deg(R2, Rt))
            ee_path.append((pl2, pl, pr2, pr))
            sat += int(np.any((q <= lo + 1e-6) | (q >= hi - 1e-6)))
        res[mode] = {
            "pos_mm": stats(np.array(ep)),
            "rot_deg": stats(np.array(er)),
            "limit_sat_frac": sat / steps,
            "_q": qs,
            "_ee_path": ee_path,
        }

    res["t"] = t
    res["q_true"] = q_true
    res["steps"] = steps
    res["hz"] = hz
    return res


def plot_report(variant: str, fk: dict, ik_full: dict, ik_task: dict,
                traj: dict, outdir: pathlib.Path) -> list[pathlib.Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    files = []

    # 1) 位置/姿态误差分布
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    data = [ik_full["_err_pos"], ik_task["_err_pos"]]
    axes[0].boxplot(data, tick_labels=["全域采样", "任务空间采样"])
    axes[0].set_ylabel("位置误差 (mm)")
    axes[0].set_title(f"{variant.upper()} 位置误差分布（MuJoCo 真值）")
    axes[0].grid(alpha=0.3)
    data = [ik_full["_err_rot"], ik_task["_err_rot"]]
    axes[1].boxplot(data, tick_labels=["全域采样", "任务空间采样"])
    axes[1].set_ylabel("姿态误差 (deg)")
    axes[1].set_title(f"{variant.upper()} 姿态误差分布")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    p = outdir / f"{variant}_ik_error_box.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    files.append(p)

    # 2) 轨迹跟踪误差 + 平滑代价
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    t = traj["t"]
    for mode, label in (("ik-raw", "无平滑 (solveArm)"), ("ik", "WMA 平滑 (solve)")):
        ep = np.array(traj[mode]["_ee_path"])
        err = np.linalg.norm(ep[:, 0] - ep[:, 1], axis=1) * 1000.0
        axes[0].plot(t, err, lw=1.0, label=label)
    axes[0].set_xlabel("时间 (s)")
    axes[0].set_ylabel("左臂位置跟踪误差 (mm)")
    axes[0].set_title(f"{variant.upper()} 轨迹跟踪误差")
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    q_raw, q_sm = traj["ik-raw"]["_q"], traj["ik"]["_q"]
    axes[1].plot(t, np.abs(q_sm - q_raw).max(axis=1) * 180 / np.pi, lw=1.0,
                 color="#854F0B", label="|q_smoothed - q_raw| (最大关节)")
    axes[1].set_xlabel("时间 (s)")
    axes[1].set_ylabel("关节角偏差 (deg)")
    axes[1].set_title("WMA 平滑引入的关节滞后")
    axes[1].legend()
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    p = outdir / f"{variant}_traj_error.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    files.append(p)

    # 3) 目标 vs 实际 EE 轨迹（水平面投影）
    fig, ax = plt.subplots(figsize=(6, 5.5))
    ep = traj["ik-raw"]["_ee_path"]
    tgt_l = np.array([e[1] for e in ep])
    act_l = np.array([e[0] for e in ep])
    tgt_r = np.array([e[3] for e in ep])
    act_r = np.array([e[2] for e in ep])
    ax.plot(tgt_l[:, 0], tgt_l[:, 1], "--", lw=1.2, color="#5F5E5A", label="目标(左)")
    ax.plot(act_l[:, 0], act_l[:, 1], "-", lw=1.2, color="#185FA5", label="实际(左)")
    ax.plot(tgt_r[:, 0], tgt_r[:, 1], "--", lw=1.2, color="#B4B2A9", label="目标(右)")
    ax.plot(act_r[:, 0], act_r[:, 1], "-", lw=1.2, color="#993C1D", label="实际(右)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{variant.upper()} 末端轨迹：目标 vs 实际（俯视）")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    p = outdir / f"{variant}_traj_xy.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    files.append(p)
    return files


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default="both", choices=["a5", "a7", "both"])
    ap.add_argument("--n-fk", type=int, default=200)
    ap.add_argument("--n-ik", type=int, default=300)
    ap.add_argument("--traj-sec", type=float, default=6.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(HERE / "out"))
    args = ap.parse_args()

    variants = ["a5", "a7"] if args.variant == "both" else [args.variant]
    outdir = pathlib.Path(args.out)
    summary = {}

    for variant in variants:
        rng = np.random.default_rng(args.seed)
        sim = R1Sim(variant)
        print(f"\n{'=' * 70}\n{variant.upper()}  n={sim.n} DOF/臂  模型 {sim.urdf.name}\n{'=' * 70}")

        fk = eval_fk_crosscheck(sim, rng, args.n_fk)
        print(f"[1] FK 交叉校验（C++ vs MuJoCo，n={fk['n']}×2 臂）")
        print(f"    位置差 p95={fk['pos_mm']['p95']:.3e} mm  max={fk['pos_mm']['max']:.3e} mm")
        print(f"    姿态差 p95={fk['rot_deg']['p95']:.3e} deg max={fk['rot_deg']['max']:.3e} deg")

        n_arm = sim.n
        home = np.zeros(2 * n_arm)
        ik_full = eval_ik(sim, rng, args.n_ik, tag="全域采样")
        ik_task = eval_ik(sim, rng, args.n_ik, center=home, span=0.6, tag="任务空间采样(±0.6rad)")
        for r in (ik_full, ik_task):
            print(f"[2] IK 精度 —— {r['tag']}（n={r['n']} 组双臂目标）")
            print(f"    位置误差 mean={r['pos_mm']['mean']:.2f} mm  p95={r['pos_mm']['p95']:.2f} mm"
                  f"  max={r['pos_mm']['max']:.2f} mm")
            print(f"    姿态误差 mean={r['rot_deg']['mean']:.3f} deg p95={r['rot_deg']['p95']:.3f} deg")
            print(f"    成功(≤5mm)={r['success_5mm']*100:.1f}%  (≤10mm)={r['success_10mm']*100:.1f}%"
                  f"  触限位比例={r['limit_hit_frac']*100:.1f}%")

        traj = eval_trajectory(sim, rng, seconds=args.traj_sec)
        print(f"[3] 轨迹跟踪（{traj['steps']} 帧 @ {traj['hz']:.0f} Hz，关节空间真值）")
        for mode in ("ik-raw", "ik"):
            m = traj[mode]
            print(f"    {mode:7s} 位置 mean={m['pos_mm']['mean']:.2f} mm "
                  f"p95={m['pos_mm']['p95']:.2f} mm max={m['pos_mm']['max']:.2f} mm | "
                  f"姿态 mean={m['rot_deg']['mean']:.3f} deg")

        files = plot_report(variant, fk, ik_full, ik_task, traj, outdir)
        print("[4] 图表：")
        for f in files:
            print(f"    {f}")

        def clean(d):
            return {k: v for k, v in d.items() if not k.startswith("_")}

        summary[variant] = {
            "fk_crosscheck": fk,
            "ik_full_range": clean(ik_full),
            "ik_task_space": clean(ik_task),
            "trajectory": {
                "steps": traj["steps"],
                "hz": traj["hz"],
                **{m: clean(traj[m]) for m in ("ik-raw", "ik")},
            },
            "charts": [str(f) for f in files],
        }

    outdir.mkdir(parents=True, exist_ok=True)
    jp = outdir / "eval_summary.json"
    jp.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n结果已写入 {jp}")


if __name__ == "__main__":
    main()
