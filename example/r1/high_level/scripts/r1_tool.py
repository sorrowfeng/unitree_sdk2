#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R1 官方 SDK 全功能交互终端（部署于 R1-EDU 开发计算单元 / 算力背包）。

把所有官方服务都做成菜单，全部指令和参数提示都显示在终端里，无需记忆：

  运动控制 loco       (sport)          FSM、阻尼/站立/启动/零力矩、移动、速度档位
  运动模式 motion_switcher             check/select/release/silent
  机器人状态 robot_state               服务列表、版本、低功耗、上报频率、服务开关
  语音灯光 voice       (audio)          TTS、音量、RGB 灯、ASR
  配置 config                          读/写/删/元信息
  只读状态 lowstate                    订阅 rt/lowstate 打印 IMU + 35 电机

底层调用已编译的 build/bin/r1_tool；不存在时自动调用同目录 build.sh 编译。

用法：
    python3 r1_tool.py                 # 交互菜单
    python3 r1_tool.py --iface eth0
    python3 r1_tool.py loco get-fsm    # 也可直接执行（脚本化）
"""

import argparse
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
TOOL_BIN = os.path.join(ROOT, "build", "bin", "r1_tool")


class Tool:
    def __init__(self, iface=None, yes=False, dry_run=False):
        self.iface = iface
        self.yes = yes
        self.dry_run = dry_run

    def detect_iface(self):
        try:
            out = subprocess.run(["ip", "-o", "-4", "addr", "show"],
                                 capture_output=True, text=True, check=False).stdout
        except FileNotFoundError:
            return None
        for line in out.splitlines():
            p = line.split()
            if len(p) >= 4 and p[3].startswith("192.168.123."):
                return p[1]
        return None

    def ensure_iface(self):
        if not self.iface:
            self.iface = self.detect_iface()
        if not self.iface:
            sys.exit("[r1] 未探测到 192.168.123.x 网卡，请用 --iface <name> 指定（如 --iface eth0）")

    def ensure_bin(self):
        if not (os.path.isfile(TOOL_BIN) and os.access(TOOL_BIN, os.X_OK)):
            print(f"[r1] 未找到 {TOOL_BIN}，正在编译 r1_tool ...")
            subprocess.run([os.path.join(SCRIPT_DIR, "build.sh"), "r1_tool"], check=True)
        if not os.access(TOOL_BIN, os.X_OK):
            sys.exit(f"[r1] 编译后仍未找到 {TOOL_BIN}")

    def confirm(self, msg):
        if self.yes:
            return True
        print(msg)
        try:
            return input("确认执行？输入 YES 继续: ").strip() == "YES"
        except EOFError:
            return False

    def run(self, *args):
        cmd = [TOOL_BIN, self.iface, *args]
        print("[r1] 执行:", " ".join(cmd))
        if self.dry_run:
            return 0
        return subprocess.run(cmd, check=False).returncode


def ask(prompt, default=""):
    try:
        v = input(prompt).strip()
    except EOFError:
        return None
    return v or default


def submenu(tool, title, entries, back="0"):
    """entries: list of (key, label, handler). handler 返回 False 表示返回上级。"""
    while True:
        print(f"\n----------- {title} -----------")
        for key, label, _ in entries:
            print(f"  {key:>3}) {label}")
        print(f"    0) 返回")
        print("-" * (30 + len(title)))
        c = ask("请选择: ")
        if c is None or c == back:
            return
        for key, _label, handler in entries:
            if c == key:
                handler()
                break
        else:
            print("无效选择")


def menu_loco(tool):
    def guard(msg):
        return tool.confirm(msg)

    def set_fsm(fid):
        if fid == 0 and not guard("⚠️  即将切换到【零力矩 ZeroTorque】：机器人会瘫软，请确保有保护！"):
            print("[r1] 已取消"); return
        tool.run("loco", "set-fsm", str(fid))

    def move():
        vx = ask("vx (m/s, 回车=0): ", "0")
        vy = ask("vy (m/s, 回车=0): ", "0")
        w = ask("vyaw (rad/s, 回车=0): ", "0")
        d = ask("duration (s, 回车=1): ", "1")
        if None in (vx, vy, w, d):
            return
        if not guard(f"⚠️  即将以 vx={vx} vy={vy} vyaw={w} 移动 {d}s，请确保周围安全！"):
            print("[r1] 已取消"); return
        tool.run("loco", "move", vx, vy, w, d)

    entries = [
        ("1", "查询 FSM ID / FSM Mode", lambda: (tool.run("loco", "get-fsm"),
                                                 tool.run("loco", "get-fsm-mode"))),
        ("2", "阻尼 Damp            [fsm 1]", lambda: tool.run("loco", "damp")),
        ("3", "站立 StandUp         [fsm 4]", lambda: tool.run("loco", "stand")),
        ("4", "启动主运控 Start     [fsm 811]", lambda: tool.run("loco", "start")),
        ("5", "移动 move vx vy vyaw [duration]", move),
        ("6", "停止移动 StopMove", lambda: tool.run("loco", "stop")),
        ("7", "设置速度档位 SetSpeedMode", lambda: _ask_run(tool, "速度档位:", "loco", "speed")),
        ("8", "零力矩 ZeroTorque    [fsm 0]  ⚠️", lambda: set_fsm(0)),
        ("9", "设置任意 FSM ID", lambda: _ask_run(tool, "FSM ID:", "loco", "set-fsm")),
    ]
    submenu(tool, "运动控制 loco (sport)", entries)


def menu_msc(tool):
    def release():
        if not tool.confirm("⚠️  Release 会释放当前运动模式（如 ai_sport），机器人将失去该服务控制。"):
            print("[r1] 已取消"); return
        tool.run("msc", "release")

    entries = [
        ("1", "查询当前模式 CheckMode", lambda: tool.run("msc", "check")),
        ("2", "选择模式 SelectMode (如 ai)", lambda: _ask_run(tool, "模式名/别名:", "msc", "select")),
        ("3", "释放模式 ReleaseMode  ⚠️", release),
        ("4", "查询静默 GetSilent", lambda: tool.run("msc", "silent-get")),
        ("5", "设置静默 SetSilent <0|1>", lambda: _ask_run(tool, "0/1:", "msc", "silent-set")),
    ]
    submenu(tool, "运动模式 motion_switcher", entries)


def menu_state(tool):
    entries = [
        ("1", "列出所有服务及状态 ServiceList", lambda: tool.run("state", "services")),
        ("2", "查询版本 GetPkgVersion", lambda: tool.run("state", "version")),
        ("3", "低功耗状态 LowPowerStatus", lambda: tool.run("state", "lowpower-get")),
        ("4", "低功耗开关 LowPowerSwitch <0|1>", lambda: _ask_run(tool, "0/1:", "state", "lowpower-set")),
        ("5", "设置上报频率 SetReportFreq <interval> <duration>", lambda: _ask2_run(tool, "interval(ms):", "duration(ms):", "state", "report")),
        ("6", "服务开关 ServiceSwitch <name> <0|1>", lambda: _ask2_run(tool, "服务名:", "0/1:", "state", "switch")),
    ]
    submenu(tool, "机器人状态 robot_state", entries)


def menu_audio(tool):
    entries = [
        ("1", "语音合成 TTS <text> [speaker]", lambda: _tts(tool)),
        ("2", "查询音量 GetVolume", lambda: tool.run("audio", "vol-get")),
        ("3", "设置音量 SetVolume <0-100>", lambda: _ask_run(tool, "音量 0-100:", "audio", "vol-set")),
        ("4", "RGB 灯 LedControl <r> <g> <b>", lambda: _led(tool)),
        ("5", "监听 ASR 识别 rt/audio_msg [sec]", lambda: _ask_run(tool, "监听秒数(回车=10):", "audio", "asr", allow_empty=True)),
    ]
    submenu(tool, "语音/灯 voice", entries)


def menu_config(tool):
    entries = [
        ("1", "读取配置 config get <name>", lambda: _ask_run(tool, "配置名:", "config", "get")),
        ("2", "写配置 config set <name> <content>", lambda: _ask2_run(tool, "配置名:", "内容:", "config", "set")),
        ("3", "删除配置 config del <name>", lambda: _ask_run(tool, "配置名:", "config", "del")),
        ("4", "配置元信息 config meta <name>", lambda: _ask_run(tool, "配置名:", "config", "meta")),
    ]
    submenu(tool, "配置 config", entries)


def menu_lowstate(tool):
    entries = [
        ("1", "打印 IMU + 35 电机状态 (3s)", lambda: tool.run("lowstate", "dump", "3")),
        ("2", "自定义时长 lowstate dump <sec>", lambda: _ask_run(tool, "秒数(回车=3):", "lowstate", "dump", allow_empty=True)),
    ]
    submenu(tool, "只读状态 lowstate", entries)


# ---- 参数辅助 ----
def _ask_run(tool, prompt, group, action, allow_empty=False):
    v = ask(prompt)
    if v is None:
        return
    args = [group, action]
    if v != "":
        args.append(v)
    elif not allow_empty:
        print("[r1] 未输入，取消"); return
    tool.run(*args)


def _ask2_run(tool, p1, p2, group, action):
    a = ask(p1)
    b = ask(p2)
    if a is None or b is None or a == "" or b == "":
        print("[r1] 输入不完整，取消"); return
    tool.run(group, action, a, b)


def _tts(tool):
    text = ask("要合成的文本: ")
    if not text:
        print("[r1] 空文本，取消"); return
    spk = ask("speaker id (回车=0): ", "0")
    tool.run("audio", "tts", text, spk)


def _led(tool):
    r = ask("R (0-255): ")
    g = ask("G (0-255): ")
    b = ask("B (0-255): ")
    if None in (r, g, b) or "" in (r, g, b):
        print("[r1] 输入不完整，取消"); return
    tool.run("audio", "led", r, g, b)


MENUS = [
    ("1", "运动控制 loco (sport)      阻尼/站立/启动/零力矩/移动/速度", menu_loco),
    ("2", "运动模式 motion_switcher   check/select/release/silent", menu_msc),
    ("3", "机器人状态 robot_state     服务列表/版本/低功耗/上报/开关", menu_state),
    ("4", "语音/灯 voice             TTS/音量/RGB/ASR", menu_audio),
    ("5", "配置 config               读/写/删/元信息", menu_config),
    ("6", "只读状态 lowstate         IMU + 35 电机", menu_lowstate),
]


def main_menu(tool):
    print(f"[r1] 网卡: {tool.iface}")
    print(f"[r1] 二进制: {TOOL_BIN}")
    while True:
        print("\n============== R1 官方 SDK 全功能测试 ==============")
        for k, label, _ in MENUS:
            print(f"  {k}) {label}")
        print("  0) 退出")
        print("====================================================")
        c = ask("请选择: ")
        if c is None or c == "0":
            return
        for k, _label, fn in MENUS:
            if c == k:
                fn(tool)
                break
        else:
            print("无效选择")


def main():
    ap = argparse.ArgumentParser(description="R1 官方 SDK 全功能交互终端")
    ap.add_argument("--iface", default=None, help="DDS 网卡（默认自动探测 192.168.123.x）")
    ap.add_argument("-y", "--yes", action="store_true", help="跳过危险操作确认")
    ap.add_argument("--dry-run", action="store_true", help="只打印命令，不执行")
    ap.add_argument("args", nargs="*", help="直接执行: <group> <action> [args...]")
    a = ap.parse_args()

    tool = Tool(a.iface, yes=a.yes, dry_run=a.dry_run)
    tool.ensure_iface()
    tool.ensure_bin()

    if a.args:
        sys.exit(tool.run(*a.args))
    main_menu(tool)


if __name__ == "__main__":
    main()
