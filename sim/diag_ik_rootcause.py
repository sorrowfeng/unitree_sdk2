#!/usr/bin/env python3
"""IK 发散根因定位：用 MuJoCo 做 FK/Jacobian，逐字复刻 r1_arm_ik.h::solveArm 的更新公式。

对照三组：
  A) as-written —— 与 r1_arm_ik.h 完全一致（权重只加在误差 e 上，不加在雅可比 J 上）
  B) weighted-GN —— 权重同时作用于 J 与 e（正确的加权高斯-牛顿）
  C) as-written + 步长限幅 —— 只在 A 上加 dq 截断，检验"是否只是步长过大"

若 A 复现出与 C++ 探针相同的发散（跳到限位），则根因确认为更新公式，而非测试接线。
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from r1_model import R1Sim, rot_err_deg  # noqa: E402

H_STEP = 1e-6


def rotvec(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 旋转向量（与 Eigen AngleAxis 一致）。"""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-12:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-12:  # 180°
        w, v = np.linalg.eigh((R + np.eye(3)) / 2.0)
        return v[:, -1] * ang
    return axis / n * ang


def set_arm(sim: R1Sim, q: np.ndarray, side: str) -> None:
    """只设置单臂关节（另一臂保持零位，不影响该臂 FK）。"""
    full = np.zeros(2 * sim.n)
    if side == "left":
        full[: sim.n] = q
    else:
        full[sim.n :] = q
    sim.set_q(full)


def fk(sim: R1Sim, q: np.ndarray, side: str):
    set_arm(sim, q, side)
    return sim.ee(side)


def jac(sim: R1Sim, q: np.ndarray, side: str) -> np.ndarray:
    """6 x n 有限差分雅可比（与 r1_arm_ik.h::jacobian 同法）。"""
    n = sim.n
    p0, R0 = fk(sim, q, side)
    J = np.zeros((6, n))
    for i in range(n):
        qp = q.copy()
        qp[i] += H_STEP
        p1, R1 = fk(sim, qp, side)
        J[0:3, i] = (p1 - p0) / H_STEP
        J[3:6, i] = rotvec(R1 @ R0.T) / H_STEP
    return J


def solve(sim: R1Sim, target_p, target_R, q0, side, *, mode="as-written",
          w_pos=50.0, w_rot=0.5, lam=1e-3, iters=15, max_step=None) -> np.ndarray:
    lo, hi = sim.limits[:, 0], sim.limits[:, 1]
    base = 0 if side == "left" else sim.n
    lo_s, hi_s = lo[base : base + sim.n], hi[base : base + sim.n]

    q = np.clip(np.asarray(q0, float).copy(), lo_s, hi_s)
    sw = np.sqrt([w_pos] * 3 + [w_rot] * 3)
    for _ in range(iters):
        p, R = fk(sim, q, side)
        e = np.concatenate([np.sqrt(w_pos) * (target_p - p),
                            np.sqrt(w_rot) * rotvec(R @ target_R.T)])
        if np.linalg.norm(e) < 1e-4:
            break
        J = jac(sim, q, side)
        if mode == "weighted-GN":
            J = sw[:, None] * J          # 权重同时作用于 J
        A = J.T @ J + lam * np.eye(sim.n)
        dq = np.linalg.solve(A, J.T @ e)
        if max_step is not None:
            nrm = np.linalg.norm(dq)
            if nrm > max_step:
                dq = dq * (max_step / nrm)
        q = np.clip(q + dq, lo_s, hi_s)
        if np.linalg.norm(target_p - fk(sim, q, side)[0]) < 5e-4:
            break
    return q


def main() -> None:
    sim = R1Sim("a5")
    n = sim.n
    print(f"关节限位: {np.round(sim.limits[:n] * 180 / np.pi, 2).tolist()} (deg)\n")

    cases = {
        "肘部 +0.02 rad": np.array([0, 0, 0, 0.02, 0]),
        "肘部 +0.10 rad": np.array([0, 0, 0, 0.10, 0]),
    }
    for name, dq_true in cases.items():
        q_true = np.zeros(n)
        q_true[:5] = dq_true
        set_arm(sim, q_true, "left")
        tp, tR = sim.ee("left")          # 目标位姿（MuJoCo 真值）
        print(f"--- {name} ---  目标位置 {np.round(tp, 4).tolist()}")

        lo_s, hi_s = sim.limits[:n, 0], sim.limits[:n, 1]
        variants = [
            ("A as-written(=C++)", dict(mode="as-written")),
            ("B weighted-GN", dict(mode="weighted-GN")),
            ("C as-written+限幅0.1rad", dict(mode="as-written", max_step=0.1)),
        ]
        for label, kw in variants:
            q_sol = solve(sim, tp, tR, np.zeros(n), "left", **kw)
            p, R = fk(sim, q_sol, "left")
            hit = bool(np.any((q_sol <= lo_s + 1e-6) | (q_sol >= hi_s - 1e-6)))
            print(f"  {label:24s} q[deg]={np.round(q_sol * 180 / np.pi, 1).tolist()}")
            print(f"  {'':24s} 位置误差={np.linalg.norm(p - tp) * 1000:9.3f} mm  "
                  f"姿态误差={rot_err_deg(R, tR):8.3f} deg  触限位={hit}")


if __name__ == "__main__":
    main()
