#!/usr/bin/env python3
"""安全场景回归：模拟发送器造包 -> 全链路测试 -> 断言处置序列。

断言的是 r1_pico_safety_policy.h 定义的优先级在主程序口径下成立
（pico_pipeline_test 与 r1_dual_arm_loco.cpp 共用该策略）。

断言方式刻意**不依赖包序号**（发送器的时间/序号约定可能变），只断言性质：

  normal       全程 teleop
  unsafe       全程 hold（safe_to_execute=false：只停移动、不阻尼）
  estop        出现 emergency_stop，且此后不得再回到 teleop
                —— 急停闩锁是全链路最高优先级，绝不能被执行态"穿过"
  lost         出现 hold（心跳超时 -> 阻尼），且此后不得再回到 teleop
  stop_signal  出现 hold（stop_signal -> 阻尼），且此后不得再回到 teleop
  return_zero  出现 return_zero，且此后不得再回到 teleop；回零帧 q 必须恒为 0

用法（在仓库根目录）：
    .venv/bin/python sim/safety_scenarios.py
    .venv/bin/python sim/safety_scenarios.py --switch-at 1.5 --hz 30
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SENDER = ROOT / "example/r1/high_level/scripts/pico_sim_sender.py"
PIPELINE = ROOT / "sim/build/pico_pipeline_test"
OUT = ROOT / "sim/out"

# 场景 -> 断言规格
#   all       : 每一帧都必须是该处置
#   engage    : 必须出现该处置；且首次出现之后不得再出现 teleop
#   q_zero    : 该处置的每一帧 q 必须恒为 0
SPEC = {
    "normal":      {"all": "teleop"},
    "unsafe":      {"all": "hold"},
    "estop":       {"engage": "emergency_stop"},
    "lost":        {"engage": "hold"},
    "stop_signal": {"engage": "hold"},
    "return_zero": {"engage": "return_zero", "q_zero": True},
}


def run_sender(scenario: str, switch_at: float, hz: float, duration: float,
               dump: pathlib.Path) -> None:
    subprocess.run(
        [sys.executable, str(SENDER), "--dry-run", "--dump", str(dump),
         "--duration", str(duration), "--hz", str(hz),
         "--scenario", scenario, "--switch-at", str(switch_at)],
        check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def run_pipeline(dump: pathlib.Path, rx_age_ms: float = 0.0) -> list[dict]:
    """跑全链路测试，返回每包的 {seq, safe, disp, q}。"""
    cmd = [str(PIPELINE), "--variant", "a5"]
    if rx_age_ms:
        cmd += ["--rx-age-ms", str(rx_age_ms)]
    res = subprocess.run(
        cmd, stdin=open(dump), capture_output=True, text=True, check=True,
    )
    rows: list[dict] = []
    for line in res.stdout.splitlines():
        if not line.startswith("ok=1"):
            continue
        parts = line.split()
        row: dict = {"q": None}
        for p in parts[1:]:
            key, _, val = p.partition("=")
            row[key] = val
        qi = next((i for i, p in enumerate(parts) if p.startswith("q=")), None)
        if qi is not None:
            row["q"] = [float(parts[qi][2:])] + [float(x) for x in parts[qi + 1:]]
        rows.append(row)
    return rows


def check(scenario: str, rows: list[dict], spec: dict) -> tuple[bool, str]:
    if not rows:
        return False, "无有效帧"
    disps = [r["disp"] for r in rows]

    if "all" in spec:
        bad = [r["sequence"] for r, d in zip(rows, disps) if d != spec["all"]]
        if bad:
            return False, f"{len(bad)}/{len(rows)} 帧非 {spec['all']}（如 seq={bad[:3]}）"
        return True, f"{len(rows)} 帧全为 {spec['all']}"

    want = spec["engage"]
    if want not in disps:
        return False, f"从未出现 {want}（实际={sorted(set(disps))}）"
    first = disps.index(want)
    after = disps[first + 1:]
    back = [rows[first + 1 + i]["seq"] for i, d in enumerate(after) if d == "teleop"]
    if back:
        return False, f"进入 {want} 后又回到 teleop（seq={back[:3]}）"

    detail = f"seq>={rows[first]['seq']} 起恒为 {want}"
    if spec.get("q_zero"):
        bad = [r["seq"] for r in rows[first:]
               if r["q"] is None or max(abs(v) for v in r["q"]) > 1e-12]
        if bad:
            return False, f"回零帧 q 非零（seq={bad[:3]}）"
        detail += "，q≡0"
    return True, detail


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--switch-at", type=float, default=1.5)
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=4.0)
    args = ap.parse_args()

    if not PIPELINE.exists():
        print(f"未找到 {PIPELINE}，先运行 ./sim/build_probe.sh")
        return 2
    OUT.mkdir(parents=True, exist_ok=True)

    fails: list[str] = []
    print(f"{'场景':<13}{'结果':<7}说明")
    print("-" * 62)

    for scenario, spec in SPEC.items():
        dump = OUT / f"scenario_{scenario}.jsonl"
        run_sender(scenario, args.switch_at, args.hz, args.duration, dump)
        rows = run_pipeline(dump)
        ok, detail = check(scenario, rows, spec)
        if not ok:
            fails.append(f"{scenario}: {detail}")
        print(f"{scenario:<13}{'OK' if ok else 'FAIL':<7}{detail}")

    # 额外一项：心跳超时。用 --rx-age-ms 700 重放一份完全正常的报文流，
    # 模拟"包内容都很健康、但已经在链路上躺了 700ms"，应全部判 hold。
    dump = OUT / "scenario_normal.jsonl"
    rows = run_pipeline(dump, rx_age_ms=700.0)
    ok, detail = check("stale", rows, {"all": "hold"})
    if not ok:
        fails.append(f"stale: {detail}")
    print(f"{'stale':<13}{'OK' if ok else 'FAIL':<7}{detail}")

    print()
    if fails:
        print("FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print(f"ALL SCENARIOS PASSED ({len(SPEC) + 1} 个，含 stale 重放)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
