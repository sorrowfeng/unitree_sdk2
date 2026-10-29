#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只验证通讯：发 `00 02 50 01` 开启异步反馈，然后看有没有 `0x480+node` 上报。

不发任何运动指令，纯只读验证。

拓扑（两条总线，各一只手，node 都是 1）：

    USB-CANFD #0 ── 左手（node 1）
    USB-CANFD #1 ── 右手（node 1）

    python3 check_comms.py                       # 两条总线各验一次
    python3 check_comms.py --left-bus 0 --right-bus 1
    python3 check_comms.py --scan                # 在每条总线上扫 node 1..N（确认节点号）
"""

import argparse
import sys
import time
from typing import List

from hand_canfd import DEFAULT_NODE_HINT, FEEDBACK_START_FRAME, HandBus, LHandCanfd


def build_bus(args) -> LHandCanfd:
    hands: List[HandBus] = []
    if args.left_bus >= 0:
        hands.append(HandBus("left", device_index=args.left_bus, node=args.node))
    if args.right_bus >= 0:
        hands.append(HandBus("right", device_index=args.right_bus, node=args.node))
    return LHandCanfd(hands=hands,
                      nom_baudrate=args.nom_baudrate,
                      dat_baudrate=args.dat_baudrate,
                      log_frames=(not args.quiet),
                      log=print)


def scan_nodes(args) -> int:
    """在**每条总线**上把 node 1..N 各发一次"开启反馈"，看该节点有无上报。"""
    bus = build_bus(args)
    found = {}
    try:
        bus.open()
        print(f"\n在每条总线上扫 node 1..{args.scan_max}"
              f"（每个发一次开启帧，等 0.8s 看**该节点**有无上报）")
        print("（已开启反馈的节点会持续上报 ⇒ 必须按节点单独计数，不能看总数）")
        for side in bus.sides:
            print(f"\n--- {side}（总线 #{bus.spec(side).device_index}）---")
            for node in range(1, args.scan_max + 1):
                before = bus.fb_by_side.get(side, 0)
                can_id = 0x500 + node
                bus.send_raw_node(side, node, FEEDBACK_START_FRAME, settle_s=0.8)
                got = bus.fb_by_side.get(side, 0) - before
                mark = "✅ 有上报" if got > 0 else "—"
                print(f"  node {node:>3} (CAN 0x{can_id:03X}): 反馈 {got:>5} 帧  {mark}")
                if got > 0:
                    found[side] = node
                    break
    finally:
        print("\n=== 扫描结果 ===")
        if found:
            for side, node in found.items():
                print(f"  {side}: node {node}  ✅")
            print(f"\n  ⇒ 起桥时用 --node <上面这个号>"
                  f"（例：{next(iter(found.values()))}）")
        else:
            print("  一条上报都没有 ⇒ 见下面排查清单")
        bus.close()
    return 0 if found else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手 CANFD 通讯验证（只读，双总线）")
    ap.add_argument("--left-bus", type=int, default=0, help="左手适配器序号（-1=不接）")
    ap.add_argument("--right-bus", type=int, default=1, help="右手适配器序号（-1=不接）")
    ap.add_argument("--node", type=int, default=DEFAULT_NODE_HINT,
                    help=f"节点号（两条总线相同，默认 {DEFAULT_NODE_HINT}）")
    ap.add_argument("--wait", type=float, default=3.0, help="等反馈的秒数")
    ap.add_argument("--nom-baudrate", type=int, default=1_000_000)
    ap.add_argument("--dat-baudrate", type=int, default=5_000_000)
    ap.add_argument("--quiet", action="store_true", help="不打印每一条 RX")
    ap.add_argument("--scan", action="store_true",
                    help="在每条总线上扫 node 1..N，确认节点号")
    ap.add_argument("--scan-max", type=int, default=16)
    args = ap.parse_args()

    if args.left_bus < 0 and args.right_bus < 0:
        print("至少要给一条总线（--left-bus / --right-bus，-1 表示不接）", file=sys.stderr)
        return 2
    if args.scan:
        return scan_nodes(args)

    bus = build_bus(args)
    try:
        bus.open()
        for side in bus.sides:
            bus.start_feedback(side)
        print(f"\n开启反馈后等待 {args.wait:g}s，观察 0x480+node 上报 ...")
        t0 = time.time()
        while time.time() - t0 < args.wait:
            time.sleep(0.25)
    finally:
        fb = bus.fb_by_side
        print("\n=== 通讯验证结果 ===")
        print(f"  发出的开启帧 : {FEEDBACK_START_FRAME.hex(' ').upper()}  →  {bus.sides}")
        print(f"  RX 总帧数    : {bus.rx_count}")
        print(f"  其中 TX 回环 : {bus.echo_count}")
        print(f"  ★设备反馈    : {sum(fb.values())}   {fb if fb else '（0 条 —— 没应答）'}")
        if bus.other_ids:
            print(f"  其它 CAN ID  : {bus.other_ids}")
        for side, data in bus.last_fb.items():
            print(f"  {side} 最后一帧反馈: {data[:32].hex(' ').upper()}"
                  + (" ..." if len(data) > 32 else ""))
        missing = [s for s in bus.sides if not fb.get(s)]
        if not missing:
            print("\n  ✅ 通讯正常：两条总线的设备都在主动上报")
        elif len(missing) < len(bus.sides):
            print(f"\n  ⚠ 部分总线没有反馈：{missing}")
            print(f"     查这几条总线的接线 / 手上电 / 120Ω 终端电阻")
        else:
            print("\n  ❌ 一条设备反馈都没有 ⇒ 通讯不通。按顺序查：")
            print("     ① 手上电了吗（指示灯）② CANH/CANL 接反 ③ 120Ω 终端电阻")
            print(f"     ④ 波特率（--nom-baudrate/--dat-baudrate）⑤ 节点号（默认 {args.node}，"
                  f"用 --scan 确认）")
        bus.close()
    return 0 if fb else 1


if __name__ == "__main__":
    sys.exit(main())
