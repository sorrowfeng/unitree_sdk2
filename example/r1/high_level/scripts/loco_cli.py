#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1 官方运控交互终端（部署于 R1-EDU 开发计算单元 / 算力背包）。

无需记忆命令：运行后进入菜单，列出全部官方 LocoClient 指令（含 FSM 编号），
按序号选择即可；参数会在终端中逐项提示。

底层调用本仓库已编译的 build/bin/r1_loco_client
（对应 example/r1/high_level/r1_loco_client_example.cpp）；
若二进制不存在，会自动调用同目录 build.sh 编译。

用法：
    python3 loco_cli.py                 # 进入交互菜单
    python3 loco_cli.py --iface eth0    # 指定网卡后进入菜单
    python3 loco_cli.py status          # 也可直接执行单条命令（脚本化）
    python3 loco_cli.py -y --dry-run move 0.2 0 0 5
"""

import argparse
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
LOCO_BIN = os.path.join(ROOT, "build", "bin", "r1_loco_client")

FSM_NAMES = {
    0: "ZeroTorque(零力矩)",
    1: "Damp(阻尼)",
    4: "StandUp(站立)",
    811: "Start(主运控)",
}


def fsm_name(fsm_id: int) -> str:
    return FSM_NAMES.get(fsm_id, "Unknown")


class Loco:
    def __init__(self, iface, assume_yes=False, dry_run=False):
        self.iface = iface
        self.yes = assume_yes
        self.dry_run = dry_run

    # ---- 基础 ----
    def detect_iface(self):
        try:
            out = subprocess.run(
                ["ip", "-o", "-4", "addr", "show"],
                capture_output=True, text=True, check=False,
            ).stdout
        except FileNotFoundError:
            return None
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 4 and parts[3].startswith("192.168.123."):
                return parts[1]
        return None

    def ensure_iface(self):
        if not self.iface:
            self.iface = self.detect_iface()
        if not self.iface:
            sys.exit("[loco] 未探测到 192.168.123.x 网卡，请用 --iface <name> 指定（如 --iface eth0）")

    def ensure_bin(self):
        if not (os.path.isfile(LOCO_BIN) and os.access(LOCO_BIN, os.X_OK)):
            print(f"[loco] 未找到 {LOCO_BIN}，正在编译 r1_loco_client ...")
            subprocess.run([os.path.join(SCRIPT_DIR, "build.sh"), "r1_loco_client"], check=True)
        if not os.access(LOCO_BIN, os.X_OK):
            sys.exit(f"[loco] 编译后仍未找到 {LOCO_BIN}")

    def confirm(self, msg: str) -> bool:
        if self.yes:
            return True
        print(msg)
        try:
            return input("确认执行？输入 YES 继续: ").strip() == "YES"
        except EOFError:
            return False

    def run(self, *loco_args) -> int:
        cmd = [LOCO_BIN, f"--network_interface={self.iface}", *loco_args]
        print("[loco] 执行:", " ".join(cmd))
        if self.dry_run:
            return 0
        return subprocess.run(cmd, check=False).returncode

    # ---- 指令 ----
    def status(self):
        print(f"[loco] 网卡: {self.iface}")
        self.run("--get_fsm_id")
        self.run("--get_fsm_mode")

    def watch(self, period=1.0):
        print(f"[loco] 网卡: {self.iface}，每 {period}s 查询 FSM ID（Ctrl+C 返回菜单）")
        try:
            while True:
                print(time.strftime("%H:%M:%S"), end="  ", flush=True)
                self.run("--get_fsm_id")
                time.sleep(period)
        except KeyboardInterrupt:
            print()

    def set_fsm(self, fsm_id: int):
        if fsm_id == 0:
            if not self.confirm(
                "⚠️  即将切换到【零力矩 ZeroTorque】：机器人会失去支撑瘫软，"
                "请确保有吊具或人员保护！"
            ):
                print("[loco] 已取消")
                return
        else:
            if not self.yes:
                print(f"[loco] 切换到 FSM {fsm_id} ({fsm_name(fsm_id)})")
        self.run(f"--set_fsm_id={fsm_id}")

    def move(self, vx, vy, vyaw, duration=1.0):
        if not self.confirm(
            f"⚠️  即将以 vx={vx} vy={vy} vyaw={vyaw} 移动 {duration}s，请确保机器人周围安全！"
        ):
            print("[loco] 已取消")
            return
        self.run(f"--set_velocity={vx} {vy} {vyaw} {duration}")

    # ---- 交互菜单 ----
    MENU = """
=================== R1 官方运控测试 ===================
  1) 查询状态            get_fsm_id / get_fsm_mode
  2) 轮询状态            watch (Ctrl+C 返回)
  3) 阻尼 Damp           fsm 1
  4) 站立 StandUp        fsm 4
  5) 启动主运控 Start    fsm 811
  6) 移动                move <vx> <vy> <vyaw> <duration>
  7) 停止移动            StopMove
  8) 设置速度档位        SetSpeedMode
  9) 零力矩 ZeroTorque   fsm 0   ⚠️ 危险
 10) 设置任意 FSM ID     set_fsm <id>
  0) 退出
======================================================
"""

    def _ask(self, prompt, default=""):
        try:
            v = input(prompt).strip()
        except EOFError:
            return None
        return v or default

    def menu(self):
        print(f"[loco] 网卡: {self.iface}")
        print(f"[loco] 二进制: {LOCO_BIN}")
        while True:
            print(self.MENU)
            choice = self._ask("请选择: ")
            if choice is None:
                print()
                return
            if choice == "0":
                return
            elif choice == "1":
                self.status()
            elif choice == "2":
                p = self._ask("轮询周期(秒, 回车=1): ", "1")
                if p is not None:
                    self.watch(float(p))
            elif choice == "3":
                self.run("--damp")
            elif choice == "4":
                self.run("--stand_up")
            elif choice == "5":
                self.run("--start")
            elif choice == "6":
                a = self._ask("vx (m/s, 回车=0): ", "0")
                b = self._ask("vy (m/s, 回车=0): ", "0")
                d = self._ask("vyaw (rad/s, 回车=0): ", "0")
                e = self._ask("duration (s, 回车=1): ", "1")
                if None not in (a, b, d, e):
                    self.move(a, b, d, e)
            elif choice == "7":
                self.run("--stop_move")
            elif choice == "8":
                m = self._ask("速度档位 mode: ")
                if m is not None and m != "":
                    self.run(f"--set_speed_mode={m}")
            elif choice == "9":
                self.set_fsm(0)
            elif choice == "10":
                f = self._ask("FSM ID: ")
                if f is not None and f != "":
                    self.set_fsm(int(f))
            else:
                print("无效选择")


def main():
    ap = argparse.ArgumentParser(description="R1 官方运控交互终端")
    ap.add_argument("--iface", default=None, help="DDS 网卡（默认自动探测 192.168.123.x 网卡）")
    ap.add_argument("-y", "--yes", action="store_true", help="跳过危险操作确认")
    ap.add_argument("--dry-run", action="store_true", help="只打印命令，不真正下发")
    ap.add_argument("command", nargs="?", default=None,
                    help="直接执行单条命令: status/watch/damp/stand/start/stop/"
                         "zero-torque/set-fsm/move/speed")
    ap.add_argument("args", nargs="*", help="命令参数")
    a = ap.parse_args()

    loco = Loco(a.iface, assume_yes=a.yes, dry_run=a.dry_run)
    loco.ensure_iface()
    loco.ensure_bin()

    c = a.command
    if c is None or c == "menu":
        loco.menu()
        return

    args = a.args
    if c == "status":
        loco.status()
    elif c == "watch":
        loco.watch(float(args[0]) if args else 1.0)
    elif c == "damp":
        loco.run("--damp")
    elif c == "stand":
        loco.run("--stand_up")
    elif c == "start":
        loco.run("--start")
    elif c == "stop":
        loco.run("--stop_move")
    elif c in ("zero-torque", "zero_torque"):
        loco.set_fsm(0)
    elif c in ("set-fsm", "set_fsm"):
        if not args:
            sys.exit("用法: set-fsm <id>")
        loco.set_fsm(int(args[0]))
    elif c == "move":
        if len(args) < 3:
            sys.exit("用法: move <vx> <vy> <vyaw> [duration]")
        loco.move(args[0], args[1], args[2], args[3] if len(args) > 3 else 1.0)
    elif c == "speed":
        if not args:
            sys.exit("用法: speed <mode>")
        loco.run(f"--set_speed_mode={args[0]}")
    else:
        sys.exit(f"未知命令: {c}（用 -h 查看帮助）")


if __name__ == "__main__":
    main()
