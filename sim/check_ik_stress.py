#!/usr/bin/env python
"""压力对照：随机远目标 + 零 warm start —— 官方目标函数 vs 我方 DLS。

背景：sim/eval_ik.py 的「全域采样」对每个样本都用**零 warm start** 去够一个
取自均匀随机关节构型的远目标。官方目标函数含 `0.1*||q - q_last||²`，而
`q_last` 正是 warm start（官方 robot_arm.py:2419 传的 current_lr_arm_motor_q），
所以在这种人为设定下该平滑项会主动把解往零位拉 —— 这是**目标函数固有的**性质，
不是求解器缺陷。

本脚本用同一批样本、同一 warm start，分别跑：
  A) 官方目标函数（官方 URDF + 官方权重 + SLSQP 30 iter，对齐 IPOPT 预算）
  B) 我方 r1_arm_ik.h 的 DLS（经 sim/build/bridge_ik 行协议）
若两者的位置/姿态误差分布与目标函数值 J 一致 → 移植忠实，行为差异来自目标函数本身。

另外跑一组**贴近实机**的 regime（warm start 在真解附近），展示平滑项在
连续遥操作（warm start ≈ 当前关节角 ≈ 目标附近）下不构成负担。

用法: python sim/check_ik_stress.py [a5|a7] [n_samples]
"""
import importlib.util
import sys

import numpy as np

_spec = importlib.util.spec_from_file_location("chk", "sim/check_ik_vs_official.py")
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)


def run_regime(variant, w, model, br, n, mode, seed, sigma=0.10):
    rng = np.random.default_rng(seed)
    n_arm = model.red.model.nq // 2
    lo, hi = model.lo, model.hi

    tasks, truth, warm = [], [], []
    while len(tasks) < n:
        q_true = rng.uniform(lo, hi)
        p_t, R_t = model.fk_left(q_true)
        if mode == "zero":                    # eval_ik 的全域采样：零 warm start
            q0 = np.zeros(n_arm)
        else:                                 # 贴近实机：warm start 在真解附近
            q0 = np.clip(q_true + rng.normal(0.0, sigma, n_arm), lo, hi)
        tasks.append((p_t, q0, R_t))
        truth.append((q_true, p_t, R_t))
        warm.append(q0)

    ours = br.ik_left_batch(variant, tasks)
    res = []
    for (q_true, p_t, R_t), q0, (qo, eo, ro) in zip(truth, warm, ours):
        qf, ef, rf = m.solve_official(model, w, p_t, q0, R_t, maxiter=30)
        res.append(dict(pos_o=eo, rot_o=ro, pos_f=ef, rot_f=rf,
                        J_o=m.make_cost(model, w, p_t, R_t, q0)(qo),
                        J_f=m.make_cost(model, w, p_t, R_t, q0)(qf)))
    return res


def report(tag, res):
    g = lambda k: np.array([r[k] for r in res])
    q = lambda a, p: float(np.percentile(a, p))
    print(f"\n  【{tag}】 n={len(res)}")
    print(f"  {'':<14}{'位置 mean':>11}{'位置 p95':>11}{'位置 max':>11}"
          f"{'姿态 mean':>11}{'姿态 p95':>11}{'≤5mm 占比':>11}{'J 中位':>10}")
    for name, key in (("官方目标函数", "f"), ("我方 DLS", "o")):
        p, r, J = g(f"pos_{key}"), g(f"rot_{key}"), g(f"J_{key}")
        print(f"  {name:<14}{p.mean():11.2f}{q(p,95):11.2f}{p.max():11.2f}"
              f"{r.mean():11.2f}{q(r,95):11.2f}{100.0*(p<=5).mean():10.1f}%{np.median(J):10.4f}")
    dp = np.abs(g("pos_f") - g("pos_o"))
    dr = np.abs(g("rot_f") - g("rot_o"))
    dJ = np.abs(g("J_f") - g("J_o"))
    print(f"  {'逐样本差异':<14}{'位置':>11}{dp.mean():11.2f} (max {dp.max():.1f}) "
          f"{'姿态':>12}{dr.mean():11.2f}° (max {dr.max():.1f}) "
          f"{'J':>6}{dJ.mean():9.4f} (max {dJ.max():.3f})")


def main():
    variant = sys.argv[1] if len(sys.argv) > 1 else "a5"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 120
    w = m.official_weights(variant)
    model = m.OfficialModel(variant, w)
    br = m.Bridge()

    print(f"=== {variant.upper()}  随机目标压力对照（官方目标函数 vs 我方 DLS）===")
    print(f"    权重 {w['w_pos']}*pos² + {w['w_rot']}*rot² + {w['w_reg']}*||q||² "
          f"+ {w['w_smooth']}*||q−q_last||²    每档 {n} 个样本，双臂只取左臂")

    for mode, tag in (("zero", "零 warm start（= eval_ik 全域采样的人为设定）"),
                      ("near", "warm start 在真解附近（贴近实机连续遥操作）")):
        report(tag, run_regime(variant, w, model, br, n, mode, seed=11))

    # warm start 偏离真解多远时我方 DLS 才开始与官方分叉？（实机手速的量化代理）
    print(f"\n=== warm-start 偏离扫描（σ = 每关节随机偏移 rad，目标同分布）===")
    print(f"  {'σ (rad)':>9}{'等效腕部偏移':>13}{'官方 位置mean':>14}{'我方 位置mean':>14}"
          f"{'官方 J 中位':>12}{'我方 J 中位':>12}{'位置差异 p95':>13}")
    for sg in (0.02, 0.10, 0.30, 0.60, 1.20):
        r = run_regime(variant, w, model, br, n, "near", seed=23, sigma=sg)
        g = lambda k: np.array([x[k] for x in r])
        # 腕部偏移量级：σ 乘典型力臂 0.5 m，仅为量级参考
        print(f"  {sg:9.2f}{sg*0.5*1000:12.0f}mm{g('pos_f').mean():14.2f}"
              f"{g('pos_o').mean():14.2f}{np.median(g('J_f')):12.4f}"
              f"{np.median(g('J_o')):12.4f}"
              f"{np.percentile(np.abs(g('pos_f')-g('pos_o')),95):12.2f}")
    print("\n  注：'等效腕部偏移' 按力臂 0.5 m 估计，仅用于把 σ 换算成直观量级。")


if __name__ == "__main__":
    main()
