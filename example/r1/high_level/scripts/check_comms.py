#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只验证通讯：发 `00 02 50 01` 开启异步反馈，然后看有没有 0x480+node 上报。

不发任何运动指令，纯只读验证。

    python3 check_comms.py --nodes 1 2 --wait 3
"""

import argparse
import sys
import time

from hand_canfd import FEEDBACK_START_FRAME, LHandCanfd


def scan_nodes(args) -> int:
    """对 node 1..N 逐个发"开启反馈"，看哪些节点真的会上报。"""
    bus = LHandCanfd(nom_baudrate=args.nom_baudrate,
                     dat_baudrate=args.dat_baudrate,
                     log=(None if args.quiet else print))
    found = []
    try:
        bus.open()
        print(f"\n扫描 node 1..{args.scan_max}（每个发一次开启帧，等 0.8s 看**该节点**有无上报）")
        print("（已开启反馈的节点会持续上报 ⇒ 必须按节点单独计数，不能看总数）")
        for node in range(1, args.scan_max + 1):
            before = bus.fb_by_node.get(node, 0)
            bus.start_feedback(node, settle_s=0.8)
            got = bus.fb_by_node.get(node, 0) - before
            mark = "✅ 有上报" if got > 0 else "—"
            print(f"  node {node:>3} (CAN 0x{0x500 + node:03X}): 反馈 {got:>5} 帧  {mark}")
            if got > 0:
                found.append(node)
    finally:
        print(f"\n=== 扫描结果 ===\n  上报的节点: {found if found else '（一个都没有）'}")
        if found:
            print(f"  ⇒ 请用 --nodes {' '.join(map(str, found))} 做运动测试")
        bus.close()
    return 0 if found else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手 CANFD 通讯验证（只读）")
    ap.add_argument("--nodes", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--wait", type=float, default=3.0, help="等反馈的秒数")
    ap.add_argument("--nom-baudrate", type=int, default=1_000_000)
    ap.add_argument("--dat-baudrate", type=int, default=5_000_000)
    ap.add_argument("--quiet", action="store_true", help="不打印每一条 RX")
    ap.add_argument("--scan", action="store_true",
                    help="扫描 node id 1..N，找出总线上到底有哪几只手上报")
    ap.add_argument("--scan-max", type=int, default=16)
    args = ap.parse_args()

    if args.scan:
        return scan_nodes(args)

    bus = LHandCanfd(nom_baudrate=args.nom_baudrate,
                     dat_baudrate=args.dat_baudrate,
                     log=(None if args.quiet else print))
    try:
        bus.open()
        for node in args.nodes:
            bus.start_feedback(node)
        print(f"\n开启反馈后等待 {args.wait:g}s，观察 0x480+node 上报 ...")
        t0 = time.time()
        while time.time() - t0 < args.wait:
            time.sleep(0.25)
    finally:
        fb = bus.fb_by_node
        print("\n=== 通讯验证结果 ===")
        print(f"  发出的开启帧 : {FEEDBACK_START_FRAME.hex(' ').upper()}  →  {args.nodes}")
        print(f"  RX 总帧数    : {bus.rx_count}")
        print(f"  其中 TX 回环 : {bus.echo_count}")
        print(f"  ★设备反馈    : {sum(fb.values())}   {fb if fb else '（0 条 —— 设备没应答）'}")
        if bus.other_ids:
            print(f"  其它 CAN ID  : {bus.other_ids}")
        for node, data in bus.last_fb.items():
            print(f"  node {node} 最后一帧反馈: {data[:32].hex(' ').upper()}"
                  + (" ..." if len(data) > 32 else ""))
        if fb and set(fb) >= set(args.nodes):
            print("\n  ✅ 通讯正常：设备在主动上报（0x480+node 收到了）")
        elif fb:
            print(f"\n  ⚠ 只收到部分节点反馈：{sorted(fb)}，期望 {sorted(args.nodes)}")
        else:
            print("\n  ❌ 一条设备反馈都没有 ⇒ 通讯不通。按顺序查：")
            print("     ① 手上电了吗（指示灯）② CANH/CANL 接反 ③ 120Ω 终端电阻")
            print("     ④ 波特率（--nom-baudrate/--dat-baudrate）⑤ node id 是否 1/2")
        bus.close()
    return 0 if fb else 1


if __name__ == "__main__":
    sys.exit(main())
