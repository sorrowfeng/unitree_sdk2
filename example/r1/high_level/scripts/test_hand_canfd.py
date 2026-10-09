#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双手灵巧手 CANFD 直控测试（不依赖 SDK）。

流程与厂商"运动最小示例"完全一致，只是把帧改成自己组：
    使能 → 回零(等5s) → 位置模式 → 速度 → 电流上限 → 位置

用法::

    python3 test_hand_canfd.py --selfcheck          # 只做逐字节自检，不碰硬件
    python3 test_hand_canfd.py --nodes 1 2          # 实发：两只手都动
    python3 test_hand_canfd.py --nodes 1 --sweep    # 单手来回扫
"""

import argparse
import sys
import time

from hand_canfd import (
    AXES, CMD_CURRENT, CMD_MODE, CMD_POSITION, CMD_VELOCITY,
    DEFAULT_CURRENT, DEFAULT_VELOCITY, MODE_ENABLE, MODE_HOME, MODE_POSITION,
    POSITION_MAX, LHandCanfd, build_payload,
)

# 厂商给出的"运动最小示例"原始帧 —— 用来逐字节验证我们的编码器
GOLDEN = [
    ("使能",      CMD_MODE,     [MODE_ENABLE],        "01 06 20 01 20 01 20 01 20 01 20 01 20 01"),
    ("回零",      CMD_MODE,     [MODE_HOME],          "01 06 25 04 25 04 25 04 25 04 25 04 25 04"),
    ("位置模式",  CMD_MODE,     [MODE_POSITION],      "01 06 20 00 20 00 20 00 20 00 20 00 20 00"),
    ("速度2000",  CMD_VELOCITY, [DEFAULT_VELOCITY],   "03 06 D0 07 D0 07 D0 07 D0 07 D0 07 D0 07"),
    ("电流1000",  CMD_CURRENT,  [DEFAULT_CURRENT],    "04 06 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03"),
    ("位置1000",  CMD_POSITION, [1000],               "02 06 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03"),
]


def selfcheck() -> int:
    """用 build_payload 重建厂商示例帧并逐字节比对。"""
    print("=== 编码器逐字节自检（对照厂商示例帧）===")
    bad = 0
    for name, cmd, values, golden_hex in GOLDEN:
        want = bytes.fromhex(golden_hex.replace(" ", ""))
        got = build_payload(cmd, values)
        ok = got == want
        bad += 0 if ok else 1
        print(f"  [{'OK ' if ok else 'FAIL'}] {name:<10} 轴数={AXES} len={len(got)}"
              f"  {got.hex(' ').upper()}")
        if not ok:
            print(f"         期望: {want.hex(' ').upper()}")
    print("  结果:", "全部一致 ✅" if bad == 0 else f"{bad} 条不一致 ❌")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(description="双手灵巧手 CANFD 直控测试")
    ap.add_argument("--nodes", type=int, nargs="+", default=[1, 2],
                    help="手 node id 列表（默认 1 2）")
    ap.add_argument("--selfcheck", action="store_true", help="只做编码器自检，不碰硬件")
    ap.add_argument("--nom-baudrate", type=int, default=1_000_000)
    ap.add_argument("--dat-baudrate", type=int, default=5_000_000)
    ap.add_argument("--home-wait", type=float, default=5.0, help="回零等待秒数")
    ap.add_argument("--velocity", type=int, default=DEFAULT_VELOCITY)
    ap.add_argument("--current", type=int, default=DEFAULT_CURRENT)
    ap.add_argument("--position", type=int, default=1000, help="运动目标（0..10000）")
    ap.add_argument("--sweep", action="store_true", help="来回扫：0→target→0")
    ap.add_argument("--rx-wait", type=float, default=1.0, help="发完后等反馈的秒数")
    ap.add_argument("--feedback", action="store_true",
                    help="额外开启异步反馈上报（00 02 50 01）。运动不需要它，只有想看状态/确认在线时才加")
    args = ap.parse_args()

    if selfcheck() != 0:
        print("\n编码器自检失败，终止（先修 encode 再碰硬件）")
        return 2
    if args.selfcheck:
        return 0

    bus = LHandCanfd(nom_baudrate=args.nom_baudrate,
                     dat_baudrate=args.dat_baudrate)
    try:
        bus.open()

        # ---- 1. 逐手初始化：使能 → 回零 → 位置模式 → 速度/电流 ----
        # 注意：默认**不开**反馈上报（运动不需要）；要读状态加 --feedback
        print(f"反馈上报: {'开启' if args.feedback else '关闭（默认，运动不需要）'}")
        for node in args.nodes:
            bus.init_hand(node, home_wait=args.home_wait,
                          velocity=args.velocity, current=args.current,
                          feedback=args.feedback)

        # ---- 2. 分批下发位置，观察运动 ----
        target = args.position
        plan = [(0, "回零位")] if not args.sweep else [
            (0, "零位"), (target, f"到 {target}"),
            (0, "回零位"), (target // 2, f"到 {target // 2}"), (0, "回零位"),
        ]
        for pos, label in plan:
            for node in args.nodes:
                print(f"[hand {node}] {label}: position={pos}")
                bus.set_position(node, pos)
            print(f"  等待 {args.rx_wait:g}s ...")
            time.sleep(args.rx_wait)

        # ---- 3. 总线统计：区分"自己的回环"与"设备真正的反馈" ----
        time.sleep(0.5)
        fb = bus.fb_by_node
        print(f"\n=== 总线统计 ===")
        print(f"  RX 总帧数     : {bus.rx_count}")
        print(f"  其中 TX 回环  : {bus.echo_count}    ← 适配器把自己的发送回显，**不是**手的反馈")
        print(f"  设备反馈帧    : {sum(fb.values())}   {fb if fb else '（0x480+node / 0x580+node 上一帧都没收到）'}")
        if bus.other_ids:
            print(f"  其它 CAN ID   : {bus.other_ids}")
        for node, data in bus.last_fb.items():
            print(f"  node {node} 最后一帧反馈: {data[:24].hex(' ').upper()}"
                  + (" ..." if len(data) > 24 else ""))
        if not fb:
            print("\n  ⚠ 没收到设备反馈。可能：① 手没接好/没上电；② 本固件只在特定事件才回帧。")
            print("     判据以**肉眼看到手是否运动**为准（回环帧不能当证据）。")

    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)")
    except Exception as exc:  # noqa: BLE001
        print(f"\n出错: {exc}")
        return 1
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
