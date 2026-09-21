#!/usr/bin/env python3
"""L2 可视化：在 MuJoCo 里回放 C++ IK 的遥操结果，肉眼核对准确性。

机器人模型按 **C++ IK 输出的关节角** 驱动（即实机将执行的动作）；
绿色球 = 目标末端位姿（"人手"位置），红色球 = 实际末端位姿。
两者重合说明遥操准确，出现分离即为跟踪误差。

用法：
  python sim/view_traj.py --variant a5                  # 打开交互窗口
  python sim/view_traj.py --variant a5 --record sim/out/a5_teleop.mp4
  python sim/view_traj.py --variant both --sec 10 --fps 30

依赖：mujoco（必需）；写 mp4 另需 imageio + imageio-ffmpeg。

macOS 提示：开窗口需要 `mjpython`（Cocoa 主线程要求），普通 python 直接跑本脚本
会**自动切换**过去，无需手动加环境变量；`--headless` 则完全不涉及窗口。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from eval_ik import ProbeSession, pose_row  # noqa: E402
from gui_boot import ensure_gui_interpreter  # noqa: E402
from r1_model import R1Sim, rot_err_deg  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent


def add_markers(scene, targets, act, lift: float, radius: float = 0.026) -> None:
    """把"目标/实际"末端标记画进 MjvScene（不改模型，viewer 与离屏渲染通用）。

    每只手画两个球，上下错开避免互相遮挡：
      绿球 = 目标末端（人手给的位置，抬高 lift）
      红球 = 实际末端（IK 解算后到达的位置，抬高 lift + 0.05）
    两球在水平方向对齐即跟踪准确；红球相对绿球偏移即为误差（偏移量=真实误差，
    因两球抬高量相同，竖直错开不影响水平偏差的判读）。
    """
    import mujoco

    items = [
        (targets[0], [0.10, 0.75, 0.30, 1.0], lift),                 # 目标-左
        (targets[2], [0.10, 0.75, 0.30, 1.0], lift),                 # 目标-右
        (act[0], [0.95, 0.15, 0.15, 1.0], lift + 0.05),              # 实际-左
        (act[1], [0.95, 0.15, 0.15, 1.0], lift + 0.05),              # 实际-右
    ]
    for p, rgba, dz in items:
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0]),
            np.asarray(p, dtype=float) + np.array([0.0, 0.0, dz]),
            np.eye(3).flatten(),
            np.array(rgba, dtype=np.float32),
        )
        scene.ngeom += 1


def make_trajectory(sim: R1Sim, rng: np.random.Generator, sec: float, hz: float,
                    margin: float = 0.05):
    """限位内的关节空间平滑轨迹（真值），返回 (t, q_true)。"""
    steps = int(sec * hz)
    t = np.arange(steps) / hz
    lo, hi = sim.limits[:, 0], sim.limits[:, 1]
    center = 0.5 * (lo + hi)
    amp = np.minimum(0.28 * (hi - lo) / 2.0, 0.42)
    freq = rng.uniform(0.15, 0.40, size=2 * sim.n)
    phase = rng.uniform(0, 2 * np.pi, size=2 * sim.n)
    q = center + amp * np.sin(2 * np.pi * freq * t[:, None] + phase[None, :])
    return t, np.clip(q, lo + margin, hi - margin)


def run_sim(variant: str, sec: float, hz: float, record: str | None,
            seed: int = 4, show: bool = True, mode: str = "ik-raw",
            width: int = 900, height: int = 640) -> pathlib.Path | None:
    sim = R1Sim(variant)
    rng = np.random.default_rng(seed)
    _, q_true = make_trajectory(sim, rng, sec, hz)

    # 目标 EE 位姿（MuJoCo 真值）+ C++ IK 求解（warm start）
    rows, targets = [], []
    for qt in q_true:
        sim.set_q(qt)
        pl, Rl, pr, Rr = sim.ee_dual()
        targets.append((pl, Rl, pr, Rr))
        rows.append(pose_row(pl, Rl) + pose_row(pr, Rr) + [0.0] * (2 * sim.n))

    q_sol = []
    cur = np.zeros(2 * sim.n)
    with ProbeSession(variant, mode) as sess:
        for r in rows:
            line = list(r)
            line[14:] = list(cur)
            cur = sess.query(line)
            q_sol.append(cur.copy())
    q_sol = np.array(q_sol)

    errs, rerrs, act = [], [], []
    for qs, (pl, Rl, pr, Rr) in zip(q_sol, targets):
        sim.set_q(qs)
        pl2, Rl2, pr2, Rr2 = sim.ee_dual()
        act.append((pl2, pr2))
        errs.append(max(np.linalg.norm(pl2 - pl), np.linalg.norm(pr2 - pr)) * 1000)
        rerrs.append(max(rot_err_deg(Rl2, Rl), rot_err_deg(Rr2, Rr)))
    errs, rerrs = np.array(errs), np.array(rerrs)
    print(f"[view] {variant} 轨迹 {len(q_sol)} 帧 @{hz:.0f}Hz  "
          f"位置误差 mean={errs.mean():.3f} p95={np.percentile(errs, 95):.3f} "
          f"max={errs.max():.3f} mm | 姿态 max={rerrs.max():.3f} deg")

    model, data = sim.model, sim.data
    LIFT = 0.10

    # 离屏渲染缓冲区默认 640x480，按需放大（必须在创建 Renderer 之前设置）
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), height)

    def apply_frame(i: int):
        sim.set_q(q_sol[i])

    out_path = None
    if record:
        import imageio
        import mujoco

        out_path = pathlib.Path(record)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(model, height, width) as rend:
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(model, cam)
            cam.lookat[:] = [0.20, 0.0, 0.15]
            cam.distance = 1.35
            cam.azimuth = 145
            cam.elevation = -12
            writer = imageio.get_writer(str(out_path), fps=int(hz), quality=8)
            for i in range(len(q_sol)):
                apply_frame(i)
                rend.update_scene(data, camera=cam)
                add_markers(rend.scene, (targets[i][0], targets[i][1], targets[i][2],
                                         targets[i][3]), act[i], LIFT)
                writer.append_data(rend.render())
            writer.close()
        print(f"[view] 视频已写入 {out_path}")

    if show:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("[view] 窗口已打开，按 Ctrl+C 或关闭窗口退出")
            i = 0
            while viewer.is_running() and i < len(q_sol):
                t0 = time.time()
                apply_frame(i)
                viewer.user_scn.ngeom = 0
                add_markers(viewer.user_scn, (targets[i][0], targets[i][1],
                                              targets[i][2], targets[i][3]),
                            act[i], LIFT)
                viewer.sync()
                if i % max(1, int(hz)) == 0:
                    print(f"  帧{i:4d}/{len(q_sol)}  误差 {errs[i]:7.3f} mm")
                i += 1
                dt = 1.0 / hz - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
    return out_path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default="a5", choices=["a5", "a7", "both"])
    ap.add_argument("--sec", type=float, default=8.0)
    ap.add_argument("--fps", type=float, default=60.0, help="回放帧率")
    ap.add_argument("--mode", default="ik-raw", choices=["ik-raw", "ik"],
                    help="ik-raw=不平滑; ik=含 WMA 平滑（实机主程序用这个）")
    ap.add_argument("--record", default=None, help="导出 mp4 路径（需 imageio-ffmpeg）")
    ap.add_argument("--headless", action="store_true", help="只算不回放（配 --record 用）")
    ap.add_argument("--seed", type=int, default=4)
    args = ap.parse_args()

    if not args.headless:
        ensure_gui_interpreter()

    variants = ["a5", "a7"] if args.variant == "both" else [args.variant]
    for v in variants:
        rec = None
        if args.record:
            p = pathlib.Path(args.record)
            rec = str(p.with_name(f"{p.stem}_{v}{p.suffix}")) if len(variants) > 1 else str(p)
        run_sim(v, args.sec, args.fps, rec, seed=args.seed, show=not args.headless,
                mode=args.mode)


if __name__ == "__main__":
    main()
