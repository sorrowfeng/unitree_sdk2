#!/usr/bin/env python3
# ============================================================================
# hand_rs485_bringup.py —— 灵巧手 RS485 直连打通测试（不依赖任何第三方库）
#
# 背景：手挂在**背包（PC2）**的串口上，不在 PC1、也不在 Mac。背包上**没有 pyserial、
#       apt 也是坏的、无外网** ⇒ 本脚本只用标准库（os/termios/select）。
#
# 协议（用户实测给出，6 轴 DH116S，从站地址 1）：
#       50 | 01 | CMD | 06 | N×2字节小端数值 | Modbus-CRC16(小端)
#       CMD=0x81 模式/状态(使能/回零/位置模式)  0x82 位置  0x83 速度  0x84 电流
#       ⚠️ 给出的帧**已含 CRC**，所以绝不能再用 --crc 之类的自动补 CRC（会拼两次）。
#
# 用法（在**背包**上，`cd ~/unitree_sdk2`）:
#   python3 example/r1/high_level/scripts/hand_rs485_bringup.py --list
#   python3 example/r1/high_level/scripts/hand_rs485_bringup.py --port /dev/ttyHand1 --dry-run
#   python3 example/r1/high_level/scripts/hand_rs485_bringup.py --port /dev/ttyHand1 --yes
#
# 安全：默认是 dry-run，只打印不发送；必须显式加 --yes 才真的写串口。
#       全过程只是「使能→回零→位置模式→电流/速度/位置」，不会改寄存器配置。
# ============================================================================
import argparse
import glob
import os
import select
import sys
import time

try:
    import termios
except ImportError:
    print('本脚本需要 termios（Linux/macOS）。Windows 请用 WSL 或厂商工具。', file=sys.stderr)
    raise SystemExit(2)

BAUD_DEFAULT = 500000
SLAVE = 0x01
NJOINTS = 6

# 用户实测给出的 6 条帧（已含 Modbus CRC16，小端）。顺序即执行顺序。
SEQUENCE = [
    ('使能 6 轴',            '50 01 81 06 20 01 20 01 20 01 20 01 20 01 20 01 E8 4A', 0.30),
    ('回零 6 轴',            '50 01 81 06 25 04 25 04 25 04 25 04 25 04 25 04 50 3E', 5.00),
    ('切回位置模式',          '50 01 81 06 20 00 20 00 20 00 20 00 20 00 20 00 5C 26', 0.30),
    ('设置电流 1000',         '50 01 84 06 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 97 C0', 0.30),
    ('设置速度 2000',         '50 01 83 06 D0 07 D0 07 D0 07 D0 07 D0 07 D0 07 7A D5', 0.30),
    ('设置位置 5000',         '50 01 82 06 88 13 88 13 88 13 88 13 88 13 88 13 62 7B', 0.30),
]

# 候选串口：背包上那块双路 USB-转-串口板会被 udev 固定成 ttyHand0/1
PORT_PATTERNS = ['/dev/ttyHand*', '/dev/ttyUSB*', '/dev/ttyACM*',
                 '/dev/ttyXR*', '/dev/ttyCH343*', '/dev/ttyCH341*']


def hexs(b):
    return ' '.join('%02X' % x for x in b)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def verify_frames():
    """自检：确保内置帧的 CRC 自洽（防止抄错）。"""
    bad = []
    for name, s, _ in SEQUENCE:
        b = bytes.fromhex(s.replace(' ', ''))
        body, got = b[:-2], int.from_bytes(b[-2:], 'little')
        if crc16_modbus(body) != got or body[1] != SLAVE or body[3] != NJOINTS:
            bad.append(name)
    return bad


def find_ports():
    found = []
    for pat in PORT_PATTERNS:
        found += sorted(glob.glob(pat))
    # 去重：符号链接与真名可能指向同一设备
    seen, out = {}, []
    for p in found:
        real = os.path.realpath(p)
        if real not in seen:
            seen[real] = p
            out.append(p)
    return out


def cmd_list():
    print('== 候选串口 ==')
    ports = find_ports()
    if not ports:
        print('  （一个都没有）')
        print('  ⇒ 手/转接板没插好，或芯片驱动缺失（CH342/CH343 在 5.10 内核上不会出现）')
    for p in ports:
        real = os.path.realpath(p)
        extra = '' if real == p else '  -> %s' % real
        print('  %s%s' % (p, extra))

    print('\n== udev 固定名规则（手部口是谁）==')
    rules = '/etc/udev/rules.d/99-rosmaster-serial.rules'
    if os.path.exists(rules):
        with open(rules) as fh:
            for line in fh:
                line = line.rstrip()
                if line and not line.lstrip().startswith('#'):
                    print('  ' + line)
    else:
        print('  (%s 不存在)' % rules)

    print('\n== 波特率 500000 在本平台可用？ ==')
    ok = getattr(termios, 'B500000', None) is not None
    print('  %s' % ('✅ 可用' if ok else '❌ 不可用（termios 没有 B500000）'))
    return 0 if ports else 1


def open_serial(port, baud=BAUD_DEFAULT, timeout=0.3):
    const = getattr(termios, 'B%d' % baud, None)
    if const is None:
        raise OSError('波特率 %d 本平台不支持（termios 无 B%d）' % (baud, baud))
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        iflag, oflag, cflag, lflag, _is, _os, cc = termios.tcgetattr(fd)
        iflag = 0                      # 原始模式：不翻译 CR/LF、不查奇偶
        oflag = 0
        lflag = 0                      # 去掉 ICANON/ECHO/ISIG
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL   # 8N1，无流控
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, const, const, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def read_reply(fd, timeout):
    """读一帧应答（没有就返回空）。"""
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], max(0.0, deadline - time.time()))
        if not r:
            break
        try:
            chunk = os.read(fd, 256)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
        time.sleep(0.02)
    return bytes(buf)


def cmd_run(args):
    if not args.port:
        print('必须给 --port（先跑 --list 看有哪些）', file=sys.stderr)
        return 2
    if not os.path.exists(args.port):
        print('串口不存在：%s' % args.port, file=sys.stderr)
        return 2

    bad = verify_frames()
    if bad:
        print('内置帧 CRC 自检失败：%s（脚本被改坏了）' % bad, file=sys.stderr)
        return 2

    print('目标串口 : %s' % args.port)
    print('波特率   : %d  8N1' % args.baud)
    print('从站地址 : %d    轴数: %d' % (SLAVE, NJOINTS))
    print('模式     : %s\n' % ('真正发送' if args.yes else 'DRY-RUN（只打印，不发送）'))

    fd = None
    if args.yes:
        try:
            fd = open_serial(args.port, args.baud)
        except OSError as e:
            print('打开串口失败：%s' % e, file=sys.stderr)
            return 1

    try:
        for i, (name, hexstr, wait) in enumerate(SEQUENCE, 1):
            tx = bytes.fromhex(hexstr.replace(' ', ''))
            print('[%d/%d] %-14s TX: %s' % (i, len(SEQUENCE), name, hexs(tx)))
            if fd is not None:
                os.write(fd, tx)
                rx = read_reply(fd, args.timeout)
                print('          RX: %s' % (hexs(rx) if rx else '(空，超时)'))
            if wait > 0:
                # 回零那条要等约 5 s，其余只需帧间小间隔
                print('          等待 %.2f s ...' % wait)
                time.sleep(wait if args.yes else 0)
    finally:
        if fd is not None:
            os.close(fd)

    print('\n完成。判据：')
    print('  · 手应「先回零，再走到位置 5000」——看得见动作就说明串口选对了')
    print('  · 六个手指任一都不动 ⇒ 换一个候选串口重试（或确认地址/线序 A-B）')
    print('  · RX 全为空是正常的：这些是写命令，很多固件不回包')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description='灵巧手 RS485 直连打通（6 轴 DH116S，Modbus-CRC16，500000 8N1）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--list', action='store_true', help='列出候选串口/udev 规则/波特率支持')
    ap.add_argument('--port', help='目标串口，如 /dev/ttyHand1')
    ap.add_argument('--baud', type=int, default=BAUD_DEFAULT)
    ap.add_argument('--timeout', type=float, default=0.3, help='读应答超时（秒）')
    ap.add_argument('--yes', action='store_true', help='确认真正发送（默认 dry-run）')
    args = ap.parse_args()
    if args.list or not args.port:
        return cmd_list()
    return cmd_run(args)


if __name__ == '__main__':
    sys.exit(main())
