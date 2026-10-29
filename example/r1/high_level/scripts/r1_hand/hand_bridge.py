#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""灵巧手 CANFD 桥 —— 承接 PICO 遥操程序的手部位置，落到 CAN 总线上。

架构
----
    r1_dual_arm_loco.cpp ──UDP 127.0.0.1:9998──> 本进程 ──CANFD──> 两只手

遥操主程序（C++）只把「左右手各 6 个关节的目标位置（0..10000）」用一行文本发过来；
本进程**独占** USB-CANFD 适配器，负责：

  1. **启动时**走完初始化：使能 → 回零(等 --home-wait) → 位置模式 → 速度 → 电流上限
  2. 之后只做**直接映射**：`遥操位置 → set_position`（不再做任何换算）

拓扑（2026-10-29 起：两条总线）
------------------------------
    USB-CANFD #0（--left-bus 0）── 左手，node 1
    USB-CANFD #1（--right-bus 1）── 右手，node 1

两只手的 **node id 都是 1**，靠**总线**区分左右（以前是一条总线挂 1/2 两个节点）。
不确定哪条总线是哪只手时，先 `--list` 看适配器，或跑起来之后只动一只手确认；
搞反了就把 `--left-bus/--right-bus` 对调。

安全语义
--------
主程序只在**遥操分支**发 `POS`；急停/掉包/保持/回零时它**什么都不发**。
所以本进程在 `--timeout` 秒没收到新位置后**停止下发**（手保持在最后位置不动），
而不是归零或松手 —— 与双臂"保持"一致。

用法
----
    # 先看有几个适配器（可选）
    python3 hand_bridge.py --list

    # 起桥（会做初始化，两只手各回零一次，约 12 s）
    python3 hand_bridge.py --left-bus 0 --right-bus 1

    # 再起遥操主程序
    build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --hand canfd

    # 想看设备反馈（确认手在线）时加 --feedback
"""

import argparse
import socket
import sys
import time
from typing import Dict, List, Optional

from hand_canfd import (
    DEFAULT_CURRENT, DEFAULT_VELOCITY, POSITION_MAX, HandBus, LHandCanfd,
)

FRAME_PREFIX = "POS"


def parse_pos(line: str) -> Optional[Dict[str, object]]:
    """解析 `POS l0..l5 r0..r5 mask`。非法返回 None。"""
    parts = line.split()
    if len(parts) != 14 or parts[0] != FRAME_PREFIX:
        return None
    try:
        vals = [int(x) for x in parts[1:13]]
    except ValueError:
        return None
    mask = int(parts[13])
    out: Dict[str, object] = {"mask": mask}
    if mask & 1:
        out["left"] = [max(0, min(POSITION_MAX, v)) for v in vals[0:6]]
    if mask & 2:
        out["right"] = [max(0, min(POSITION_MAX, v)) for v in vals[6:12]]
    return out


def list_adapters() -> int:
    adapters = LHandCanfd._probe_adapters()
    if adapters is None:
        print("拿不到适配器列表（gsusb_canfd 不可用？）", file=sys.stderr)
        return 1
    if not adapters:
        print("没有扫到 USB-CANFD 适配器（检查 USB / udev 权限）", file=sys.stderr)
        return 1
    print(f"扫到 {len(adapters)} 个 USB-CANFD 适配器（device_index 就是下面的序号）：")
    for i, a in enumerate(adapters):
        print(f"  #{i}  serial={getattr(a, 'serial', '?')!r}  "
              f"bus={getattr(a, 'bus', '?')}  addr={getattr(a, 'address', '?')}  "
              f"product={getattr(a, 'product', '?')!r}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手 CANFD 桥（遥操位置 → CAN，双总线）")
    ap.add_argument("--left-bus", type=int, default=0,
                    help="左手所在的 USB-CANFD 适配器序号（-1=不接左手）")
    ap.add_argument("--right-bus", type=int, default=1,
                    help="右手所在的 USB-CANFD 适配器序号（-1=不接右手）")
    ap.add_argument("--node", type=int, default=1,
                    help="两条总线上的节点号（默认 1，两只手相同）")
    ap.add_argument("--port", type=int, default=9998, help="监听 cpp 端发来的位置")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--home-wait", type=float, default=5.0, help="每只手回零等待秒数")
    ap.add_argument("--enable-wait", type=float, default=1.0)
    ap.add_argument("--velocity", type=int, default=DEFAULT_VELOCITY)
    ap.add_argument("--current", type=int, default=DEFAULT_CURRENT)
    ap.add_argument("--rate", type=float, default=50.0, help="位置下发频率上限 Hz")
    ap.add_argument("--timeout", type=float, default=1.0, help="多久没新位置就停止下发（保持）")
    ap.add_argument("--feedback", action="store_true",
                    help="额外开启异步反馈上报（00 02 50 01），用于确认手在线")
    ap.add_argument("--no-init", action="store_true",
                    help="跳过初始化（手已经初始化过时用，避免再次回零）")
    ap.add_argument("--verbose", action="store_true",
                    help="逐帧打印 CAN 收发（排查用；正常别开——开启反馈后约 1000 帧/秒会刷屏）")
    ap.add_argument("--list", action="store_true", help="列出 USB-CANFD 适配器后退出")
    args = ap.parse_args()

    if args.list:
        return list_adapters()

    hands: List[HandBus] = []
    if args.left_bus >= 0:
        hands.append(HandBus("left", device_index=args.left_bus, node=args.node))
    if args.right_bus >= 0:
        hands.append(HandBus("right", device_index=args.right_bus, node=args.node))
    if not hands:
        print("至少要给一条总线（--left-bus / --right-bus，-1 表示不接）", file=sys.stderr)
        return 2
    sides = [h.side for h in hands]

    bus = LHandCanfd(hands=hands, log_frames=args.verbose, log=print)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        print(f"绑定 {args.bind}:{args.port} 失败: {exc}", file=sys.stderr)
        return 1
    sock.settimeout(1.0 / max(1.0, args.rate))
    print(f"[bridge] 监听 udp://{args.bind}:{args.port}（等 cpp 端发 POS）")

    try:
        bus.open()
        if args.no_init:
            print("[bridge] --no-init：跳过初始化（假定手已使能并切到位置模式）")
        else:
            desc = "，".join(f"{h.side}=总线#{h.device_index}/node{h.node}" for h in hands)
            print(f"[bridge] 开始初始化（{desc}）；每只手 使能 → 回零{args.home_wait:g}s → "
                  f"位置模式 → 速度{args.velocity} → 电流{args.current}")
            for side in sides:
                bus.init_hand(side, home_wait=args.home_wait,
                              enable_wait=args.enable_wait,
                              velocity=args.velocity, current=args.current,
                              feedback=args.feedback)
        print(f"[bridge] ✅ 就绪：{sides} 已进入位置模式，开始接收遥操位置 "
              f"（下发 ≤{args.rate:g} Hz，{args.timeout:g}s 无新位置即保持）")

        last_targets: Dict[str, List[int]] = {}
        last_rx = 0.0
        last_send = 0.0
        stale_warned = False
        n_frames = 0

        while True:
            now = time.time()
            try:
                data, _peer = sock.recvfrom(512)
                msg = parse_pos(data.decode("ascii", "ignore").strip())
                if msg is None:
                    continue
                n_frames += 1
                last_rx = now
                stale_warned = False
                if "left" in msg and "left" in sides:
                    last_targets["left"] = msg["left"]           # type: ignore[assignment]
                if "right" in msg and "right" in sides:
                    last_targets["right"] = msg["right"]         # type: ignore[assignment]
            except socket.timeout:
                pass

            fresh = (now - last_rx) < args.timeout if last_rx else False
            if not fresh:
                if last_rx and not stale_warned:
                    stale_warned = True
                    print(f"[bridge] ⚠ {args.timeout:g}s 没收到新位置 ⇒ 停止下发，"
                          f"手保持在最后位置（遥操停了/掉包了都会这样）")
                continue

            if now - last_send < 1.0 / max(1.0, args.rate):
                continue
            last_send = now
            for side, vals in last_targets.items():
                try:
                    bus.set_positions(side, vals)      # 直接映射，不做任何换算
                except Exception as exc:  # noqa: BLE001
                    print(f"[bridge] ✗ {side} 下发失败: {exc}")
                    last_rx = 0.0   # 触发保持，避免刷屏

    except KeyboardInterrupt:
        print("\n[bridge] Ctrl+C 退出")
    except Exception as exc:  # noqa: BLE001
        print(f"\n[bridge] 出错: {exc}", file=sys.stderr)
        return 1
    finally:
        print(f"[bridge] 关闭。RX 帧数={locals().get('n_frames', 0)}，"
              f"设备反馈={bus.fb_by_side}")
        try:
            bus.close()
        finally:
            sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
