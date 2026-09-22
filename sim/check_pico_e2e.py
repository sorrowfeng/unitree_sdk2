#!/usr/bin/env python
"""PICO 报文端到端残差：合成报文 -> 全链路 -> 解出关节角 -> 独立 FK 比对。

补上 sim/README.md「PICO 报文全链路测试」第 3 步缺失的比对脚本：
README 只写了「用 MuJoCo 独立 FK 比对 /tmp/solved.txt 与 /tmp/truth.jsonl」，
但没有可执行的入口，基线数字无法复现。本脚本把它固化下来。

链路：
  1) pico_sim_sender.py --dry-run 造报文（用官方 URDF 做 FK 造可达位姿，
     再按对齐的逆变换打包成 OpenXR 位姿）+ 落盘关节角真值 q_true
  2) pico_pipeline_test 消费报文（解析 -> 安全判据 -> 对齐 -> IK）
  3) 对每一帧：MuJoCo 独立 FK(q_solved) 与 FK(q_true) 比 EE 位姿
     —— MuJoCo 与 C++ 求解器完全独立，不构成自证循环

同时跑 `--raw`（跳过 WMA 平滑）一档：两者之差即 WMA 平滑的滞后贡献。

用法:
  python sim/check_pico_e2e.py [--variant a5|a7] [--duration 3] [--hz 30] [--keep]
"""
import argparse
import json
import pathlib
import subprocess
import sys
import tempfile

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

from r1_model import R1Sim, rot_err_deg  # noqa: E402

PY = sys.executable
SENDER = ROOT / "example/r1/high_level/scripts/pico_sim_sender.py"
PIPE = HERE / "build/pico_pipeline_test"


def parse_q_lines(text):
    """从 pico_pipeline_test 输出里取出每帧解出的关节角（未输出 q 的帧记 None）。"""
    out = []
    for line in text.splitlines():
        if " q=" not in line:
            out.append(None)
            continue
        tail = line.split(" q=", 1)[1].strip()
        vals = []
        for tok in tail.split():
            try:
                vals.append(float(tok))
            except ValueError:
                break
        out.append(np.array(vals) if vals else None)
    return out


def run_pipeline(variant, dump, raw):
    cmd = [str(PIPE), "--variant", variant] + (["--raw"] if raw else [])
    with open(dump, "rb") as fh:
        p = subprocess.run(cmd, stdin=fh, capture_output=True)
    if p.returncode != 0:
        raise SystemExit(f"pico_pipeline_test 失败：{p.stderr.decode()[:400]}")
    return parse_q_lines(p.stdout.decode())


def compare(sim, truth, solved):
    """逐帧比较 FK(q_solved) 与 FK(q_true) 的 EE 位姿。"""
    pe, re_ = [], []
    for q_t, q_s in zip(truth, solved):
        if q_s is None or q_s.size != q_t.size:
            continue
        sim.set_q(q_t)
        pt_l, Rt_l, pt_r, Rt_r = sim.ee_dual()
        sim.set_q(q_s)
        pa_l, Ra_l, pa_r, Ra_r = sim.ee_dual()
        pe.append(max(np.linalg.norm(pa_l - pt_l), np.linalg.norm(pa_r - pt_r)) * 1000.0)
        re_.append(max(rot_err_deg(Ra_l, Rt_l), rot_err_deg(Ra_r, Rt_r)))
    return np.array(pe), np.array(re_)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default="a5", choices=["a5", "a7"])
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--pose-mode", default="wave", choices=["wave", "circle"],
                    help="wave=关节空间正弦；circle=任务空间圆周（半径 3cm）")
    ap.add_argument("--keep", action="store_true", help="保留临时 jsonl")
    args = ap.parse_args()

    if not PIPE.exists():
        raise SystemExit(f"探针未编译：{PIPE}\n先运行 sim/build_probe.sh")

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="pico_e2e_"))
    dump, truthf = tmp / "pico.jsonl", tmp / "truth.jsonl"

    print(f"=== PICO 端到端残差  {args.variant.upper()}  "
          f"{args.duration}s @{args.hz:g}Hz  pose-mode={args.pose_mode} ===")
    subprocess.run([PY, str(SENDER), "--dry-run", "--variant", args.variant,
                    "--dump", str(dump), "--truth-out", str(truthf),
                    "--duration", str(args.duration), "--hz", str(args.hz),
                    "--pose-mode", args.pose_mode],
                   cwd=str(ROOT), check=True, capture_output=True)

    n_pkt = sum(1 for _ in dump.open())
    truth = [np.array(json.loads(l)["q_true"]) for l in truthf.open()
             if json.loads(l).get("q_true")]
    print(f"报文 {n_pkt} 条，真值 {len(truth)} 帧")

    sim = R1Sim(args.variant)
    rows = {}
    for raw, tag in ((True, "--raw（跳过 WMA 平滑）"), (False, "ik.solve（含 WMA，= 实机口径）")):
        solved = run_pipeline(args.variant, dump, raw)
        pe, re_ = compare(sim, truth, solved)
        rows[tag] = (pe, re_)
        q = lambda a, p: float(np.percentile(a, p))
        print(f"\n  {tag}")
        print(f"    EE 位置  mean {pe.mean():7.3f}  p95 {q(pe,95):7.3f}  max {pe.max():7.3f} mm")
        print(f"    EE 姿态  mean {re_.mean():7.3f}  p95 {q(re_,95):7.3f}  max {re_.max():7.3f} °"
              f"   （{len(pe)} 帧）")

    (pr, rr), (pf, rf) = rows["--raw（跳过 WMA 平滑）"], rows["ik.solve（含 WMA，= 实机口径）"]
    print(f"\n  WMA 平滑的净贡献（ik.solve − --raw）：位置 {pf.mean()-pr.mean():+.3f} mm，"
          f"姿态 {rf.mean()-rr.mean():+.3f} °")

    if args.keep:
        print(f"\n  报文与真值保留在 {tmp}")
    else:
        for f in tmp.iterdir():
            f.unlink()
        tmp.rmdir()


if __name__ == "__main__":
    main()
