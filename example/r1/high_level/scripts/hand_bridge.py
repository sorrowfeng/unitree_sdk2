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

安全语义
--------
主程序只在**遥操分支**发 `POS`；急停/掉包/保持/回零时它**什么都不发**。
所以本进程在 `--timeout` 秒没收到新位置后**停止下发**（手保持在最后位置不动），
而不是归零或松手 —— 与双臂"保持"一致。

用法
----
    # 先起桥（它会做初始化，两只手会回零，约 12 s）
    python3 hand_bridge.py --left 1 --right 2

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
    DEFAULT_CURRENT, DEFAULT_VELOCITY, POSITION_MAX, LHandCanfd,
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


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手 CANFD 桥（遥操位置 → CAN）")
    ap.add_argument("--left", type=int, default=1, help="左手 node id（0=不接左手）")
    ap.add_argument("--right", type=int, default=2, help="右手 node id（0=不接右手）")
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
    args = ap.parse_args()

    nodes: List[int] = [n for n in (args.left, args.right) if n > 0]
    if not nodes:
        print("至少要给一个 --left / --right node id", file=sys.stderr)
        return 2

    bus = LHandCanfd(log_frames=args.verbose, log=print)
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
            print(f"[bridge] 开始初始化 {nodes}（每只手 使能 → 回零{args.home_wait:g}s → "
                  f"位置模式 → 速度{args.velocity} → 电流{args.current}）")
            for node in nodes:
                bus.init_hand(node, home_wait=args.home_wait,
                              enable_wait=args.enable_wait,
                              velocity=args.velocity, current=args.current,
                              feedback=args.feedback)
        print(f"[bridge] ✅ 就绪：{nodes} 已进入位置模式，开始接收遥操位置 "
              f"（下发 ≤{args.rate:g} Hz，{args.timeout:g}s 无新位置即保持）")

        last_targets: Dict[int, List[int]] = {}
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
                if "left" in msg and args.left > 0:
                    last_targets[args.left] = msg["left"]           # type: ignore[assignment]
                if "right" in msg and args.right > 0:
                    last_targets[args.right] = msg["right"]         # type: ignore[assignment]
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
            for node, vals in last_targets.items():
                try:
                    bus.set_positions(node, vals)      # 直接映射，不做任何换算
                except Exception as exc:  # noqa: BLE001
                    print(f"[bridge] ✗ node {node} 下发失败: {exc}")
                    last_rx = 0.0   # 触发保持，避免刷屏

    except KeyboardInterrupt:
        print("\n[bridge] Ctrl+C 退出")
    except Exception as exc:  # noqa: BLE001
        print(f"\n[bridge] 出错: {exc}", file=sys.stderr)
        return 1
    finally:
        print(f"[bridge] 关闭。RX 帧数={locals().get('n_frames', 0)}，"
              f"设备反馈={bus.fb_by_node if hasattr(bus, 'fb_by_node') else {}}")
        try:
            bus.close()
        finally:
            sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
