#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_modbus_probe.py —— 灵巧手 Modbus RTU 探针（给「适配一款新手」用）

设计前提（2026-09-23 实测）：R1-EDU 背包是 JetPack5 / Ubuntu20.04 / **Python 3.8**，
**没有 pyserial**，也不一定能联网装包。所以本脚本**只用标准库**（termios/select/struct），
直接操作 /dev/ttyXXX，不依赖任何第三方模块。

默认全部只读。写操作必须同时给 `--write` 和 `--yes`，且会先把要发的字节打印出来。

典型用法（在背包上跑）：

    # 0) 看有哪些候选串口、有没有 udev 符号链接、USB 是哪家的
    python3 hand_modbus_probe.py --list

    # 1) 盲扫：扫描「端口 × 波特率 × 从站ID」，看谁应答（只读，安全）
    python3 hand_modbus_probe.py --scan

    # 2) 指定参数读一段寄存器（FC04 与 FC03 都试）
    python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 --id 0x7e \
            --read 0 6

    # 3) 逐地址扫，摸出寄存器地图（只读）
    python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 --id 0x7e \
            --sweep 0 60

    # 4) 发任意原始帧（最后的兜底手段，--tx 里给十六进制，不含 CRC 也行）
    python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 \
            --raw '7e 04 00 00 00 06'

    # 5) 写（危险！）—— 必须自己确认参数正确，写前会打印实际字节
    python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 4000000 --id 0x27 \
            --write 2=0 3=0 4=0 5=0 --yes

三个「新手必踩」的点已经内置处理：

  1. **串口参数不是只有波特率**：数据位/校验位/停止位同样致命。
     三家实测都是 8N1，但 Modbus 标准其实是 8E1 —— 所以 `--scan` 会同时试 8N1 和 8E1。
  2. **设备回「异常帧」也说明它活着**：很多手对没定义的功能码回 `func|0x80` + 异常码，
     这不是超时。本脚本把「正常应答」和「异常应答」分开报告，异常码也翻译出来。
  3. **帧间隔**：Modbus RTU 要求两帧之间至少 3.5 个字符时间，否则从站会丢帧。
     本脚本按波特率自动计算（不低于 1 ms），并发送前清空输入缓冲。
"""

import argparse
import glob
import os
import select
import struct
import sys
import time

# ⚠️ 必须最先做：SSH 非交互会话里 locale 常是 POSIX/ASCII，Python 3.8 会按 ASCII 编码 stdout，
# 一 print 中文就 UnicodeEncodeError 直接崩。这里强制 UTF-8（3.7+ 的 reconfigure）。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError):
        pass

try:
    import termios
except ImportError:  # 理论上只会在非 POSIX 上发生
    print('需要 termios（POSIX 串口），本脚本只在 Linux/背包上运行。', file=sys.stderr)
    raise

# ---------------------------------------------------------------------------
# 常用波特率（按"新手可能用的"排序；4000000 是 Linker O6 用的，460800 是 BrainCo 用的）
# ---------------------------------------------------------------------------
COMMON_BAUD = [115200, 460800, 4000000, 1000000, 2000000, 921600, 500000, 230400, 57600, 9600]

# 三家现成手的默认参数（来自背包上的厂商源码，见 R1_BACKPACK_ARCHITECTURE.md §4.5/§4.6）
KNOWN_PRESETS = [
    ('BrainCo Revo2 (stark-sdk)', 460800, ['n'], 0x7E, 0x7F, '左右手各自一个 ID，同一条串口'),
    ('Linker O6 (libmodbus)', 4000000, ['n'], 0x28, 0x27, '左手 0x28 / 右手 0x27'),
    ('Inspire (私有协议)', 115200, ['n'], 1, 2, '不是 Modbus，本脚本只能验证"有没有应答"'),
]

EXC_TEXT = {
    0x01: '非法功能码（设备不认识这个功能码）',
    0x02: '非法数据地址（设备认识功能码，但这个寄存器不存在）',
    0x03: '非法数据值（地址/数量超范围）',
    0x04: '从站设备故障',
    0x05: '确认（已接受，需后续轮询）',
    0x06: '从站忙',
    0x08: '存储奇偶校验错',
    0x0A: '网关路径不可用',
    0x0B: '网关目标设备无响应',
}


# ---------------------------------------------------------------------------
# Modbus RTU 基础：CRC16 与收帧
# ---------------------------------------------------------------------------
def crc16_modbus(data):
    """标准 Modbus CRC16（多项式 0xA001，初值 0xFFFF），返回 (lo, hi)。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFF, (crc >> 8) & 0xFF


def with_crc(payload):
    """给不含 CRC 的负载补上 CRC（低字节在前）。"""
    lo, hi = crc16_modbus(payload)
    return bytes(payload) + bytes([lo, hi])


def check_crc(frame):
    if len(frame) < 4:
        return False
    lo, hi = crc16_modbus(frame[:-2])
    return frame[-2] == lo and frame[-1] == hi


def frame_gap(baud):
    """Modbus RTU 的 3.5 字符时间（1 字符 = 11 位），下限 1 ms。"""
    return max(0.001, 3.5 * 11.0 / float(baud))


# ---------------------------------------------------------------------------
# 串口：纯 termios 打开
# ---------------------------------------------------------------------------
def baud_constant(baud):
    const = getattr(termios, 'B%d' % baud, None)
    if const is None:
        return None
    return const


def open_serial(port, baud, parity='n', stopbits=1, timeout=0.15):
    """打开并配置串口。返回 (fd, timeout)。失败抛 OSError。"""
    if baud_constant(baud) is None:
        raise OSError('波特率 %d 这个平台不支持（termios 没有 B%d）' % (baud, baud))

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

        iflag = 0                      # 原始模式：不做任何输入处理（不翻译 CR/LF、不查奇偶）
        oflag = 0
        lflag = 0                      # 不加工（去掉 ICANON/ECHO/ISIG）
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL

        p = parity.lower()
        if p == 'e':
            cflag |= termios.PARENB
        elif p == 'o':
            cflag |= termios.PARENB | termios.PARODD
        # 'n'：不加校验位

        if int(stopbits) == 2:
            cflag |= termios.CSTOPB

        b = baud_constant(baud)
        # cc 数组长度是平台相关的（Linux NCCS=32，macOS NCCS=20），必须**原样保持长度**，
        # 多补或少给都会 tcsetattr 报错。
        nccs = len(cc)
        cc = list(cc)
        while len(cc) < nccs:
            cc.append(b'\x00')
        cc[termios.VMIN] = b'\x00'
        cc[termios.VTIME] = b'\x00'

        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, b, b, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd, timeout


def drain(fd):
    """把输入缓冲里的陈旧字节丢掉（避免上次超时残留的半个帧被当成这次的应答）。"""
    try:
        while True:
            r, _, _ = select.select([fd], [], [], 0)
            if not r:
                return
            if not os.read(fd, 4096):
                return
    except OSError:
        return


def read_frame(fd, timeout, idle_stop=0.005):
    """等到「一段时间没有新字节」或超时；返回收到的全部字节。"""
    buf = bytearray()
    deadline = time.time() + timeout
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            break
        r, _, _ = select.select([fd], [], [], remain)
        if not r:
            break
        chunk = os.read(fd, 256)
        if not chunk:
            break
        buf += chunk
        deadline = time.time() + idle_stop   # 收到字节后按"静默间隙"收尾
    return bytes(buf)


def transact(fd, payload, timeout, gap):
    """发一帧（自动补 CRC）并收应答。返回 (发送字节, 接收字节)。"""
    frame = with_crc(payload)
    drain(fd)
    os.write(fd, frame)
    time.sleep(gap)
    return frame, read_frame(fd, timeout)


# ---------------------------------------------------------------------------
# Modbus 请求构造
# ---------------------------------------------------------------------------
def req_read(slave, func, addr, count):
    return [slave & 0xFF, func & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF,
            (count >> 8) & 0xFF, count & 0xFF]


def req_write_single(slave, addr, value):
    return [slave & 0xFF, 0x06, (addr >> 8) & 0xFF, addr & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF]


def req_write_multi(slave, addr, values):
    n = len(values)
    out = [slave & 0xFF, 0x10, (addr >> 8) & 0xFF, addr & 0xFF,
           (n >> 8) & 0xFF, n & 0xFF, (n * 2) & 0xFF]
    for v in values:
        out += [(v >> 8) & 0xFF, v & 0xFF]
    return out


def parse_response(rx, expect_slave, expect_func):
    """把应答分类。返回 dict(kind=..., ...)。kind ∈
       'empty' / 'crc_bad' / 'echo' / 'exception' / 'data' / 'garbage'"""
    if not rx:
        return {'kind': 'empty'}
    if len(rx) < 4:
        return {'kind': 'garbage', 'raw': rx}
    if not check_crc(rx):
        return {'kind': 'crc_bad', 'raw': rx}

    slave, func = rx[0], rx[1]
    if func == (expect_func | 0x80):
        return {'kind': 'exception', 'slave': slave, 'func': func,
                'code': rx[2], 'text': EXC_TEXT.get(rx[2], '未知异常码 %d' % rx[2])}
    if func != expect_func:
        return {'kind': 'garbage', 'raw': rx}
    if expect_func in (0x03, 0x04):
        nbytes = rx[2]
        data = rx[3:3 + nbytes]
        if len(data) < nbytes:
            return {'kind': 'garbage', 'raw': rx}
        regs = list(struct.unpack('>%dH' % (nbytes // 2), data))
        return {'kind': 'data', 'slave': slave, 'regs': regs}
    if expect_func in (0x06, 0x10):
        # 0x06 回显：[id][06][addr][value][crc]；0x10 回显：[id][10][addr][count][crc]
        addr = struct.unpack('>H', rx[2:4])[0]
        val = struct.unpack('>H', rx[4:6])[0]
        return {'kind': 'echo', 'slave': slave, 'addr': addr,
                'count': val if expect_func == 0x10 else 1,
                'value': val if expect_func == 0x06 else None}
    return {'kind': 'garbage', 'raw': rx}


def hexs(b):
    return ' '.join('%02X' % c for c in b)


# ---------------------------------------------------------------------------
# --list
# ---------------------------------------------------------------------------
def cmd_list():
    pats = ['/dev/ttyHand*', '/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyCH343*',
            '/dev/ttyCH341*']
    found = []
    for p in pats:
        found += sorted(glob.glob(p))
    # 去重（符号链接与真名可能重复指向同一设备）
    seen = {}
    for p in found:
        real = os.path.realpath(p)
        seen.setdefault(real, []).append(p)

    print('== udev 符号链接规则（谁被固定了名字）==')
    rules = '/etc/udev/rules.d/99-rosmaster-serial.rules'
    if os.path.exists(rules):
        with open(rules, 'r') as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith('#'):
                    print('  ' + line)
    else:
        print('  (没有 %s)' % rules)

    print('\n== 候选串口设备 ==')
    if not seen:
        print('  一个都没有。⇒ 手没接上，或者 USB 转串口线没被内核认出来。')
        print('  排查：dmesg | tail -30 看有没有 ttyUSB/ttyACM 的枚举日志。')
    jetson_uarts = sorted(glob.glob('/dev/ttyS*'))
    if jetson_uarts:
        print('  （另有 %s —— 这些是 Jetson 板载 UART，不是灵巧手，--scan 会主动跳过）'
              % ', '.join(jetson_uarts))
    for real, aliases in sorted(seen.items()):
        print('  %s' % real)
        for a in aliases:
            if a != real:
                print('      ← 符号链接 %s' % a)
        # 从 sysfs 里挖 USB VID:PID 与物理端口路径 —— 这是识别"是哪家的手"的关键
        name = os.path.basename(real)
        base = '/sys/class/tty/%s/device' % name
        for _ in range(6):
            if not os.path.exists(base):
                break
            if os.path.exists(os.path.join(base, 'idVendor')):
                try:
                    vid = open(os.path.join(base, 'idVendor')).read().strip()
                    pid = open(os.path.join(base, 'idProduct')).read().strip()
                    man = ''
                    mf = os.path.join(base, 'manufacturer')
                    if os.path.exists(mf):
                        man = open(mf).read().strip()
                    print('      USB %s:%s  %s' % (vid, pid, man))
                except OSError:
                    pass
                break
            base = os.path.dirname(base)

    print('\n== 已知三家手的默认参数（供对照）==')
    for name, baud, par, lid, rid, note in KNOWN_PRESETS:
        print('  %-28s baud=%-8d 8%s1  id 左=%#04x 右=%#04x  %s'
              % (name, baud, par[0].upper(), lid, rid, note))

    print('\n== 已经在跑的桥进程（要先停掉才能独占串口）==')
    running = []
    for pid in glob.glob('/proc/[0-9]*'):
        try:
            exe = os.readlink(os.path.join(pid, 'exe'))
        except OSError:
            continue
        if any(k in exe for k in ('hand_server', 'hand_service', 'inspire_hand', 'stark')):
            running.append((os.path.basename(pid), exe))
    if running:
        for pid, exe in running:
            print('  PID %-8s %s' % (pid, exe))
        print('  ⚠️ 同一个串口不能被两个进程同时打开。测之前先')
        print('     sudo systemctl stop brainco_hand.service    # 如有')
        print('     sudo pkill -f linker_hand_server            # 如有')
    else:
        print('  (没有)')
    return 0


# ---------------------------------------------------------------------------
# --scan：盲扫，只读
# ---------------------------------------------------------------------------
def candidate_ports():
    """会被 --scan 自动扫的端口。
    ⚠️ 故意**不含 /dev/ttyS***：背包上有 ttyS0..ttyS3 是 Jetson 板载 UART（ttyS0 还是串口控制台），
    对着它们发 Modbus 帧没有任何意义，还可能把控制台搞乱。灵巧手只会出现在下面这几种名字上。"""
    pats = ['/dev/ttyHand*', '/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyCH343*', '/dev/ttyCH341*']
    out = []
    for p in pats:
        for f in sorted(glob.glob(p)):
            if f not in out:
                out.append(f)
    return out


def try_probe(port, baud, parity, slave, timeout, funcs=(0x04, 0x03)):
    """对一组参数发"读 1 个寄存器"探测帧。返回非 None 就说明设备活着。"""
    try:
        fd, to = open_serial(port, baud, parity, 1, timeout)
    except OSError as exc:
        return {'error': str(exc)}
    try:
        gap = frame_gap(baud)
        for func in funcs:
            payload = req_read(slave, func, 0, 1)
            tx, rx = transact(fd, payload, to, gap)
            r = parse_response(rx, slave, func)
            if r['kind'] in ('data', 'exception', 'echo'):
                r['tx'] = tx
                r['rx'] = rx
                r['func'] = func
                return r
        return {'kind': 'empty'}
    finally:
        os.close(fd)


def cmd_scan(args):
    ports = args.port if args.port else candidate_ports()
    if not ports:
        print('没有候选串口。先跑 --list。')
        return 2

    id_list = [int(x, 0) for x in args.id.split(',')] if args.id else \
              [1, 2, 3, 0x27, 0x28, 0x7E, 0x7F]

    if args.baud:
        bauds = [int(args.baud)]
    else:
        bauds = COMMON_BAUD
    parities = [args.parity] if args.parity else ['n', 'e']

    print('扫描：%d 个端口 × %d 个波特率 × %d 个从站ID × %d 种校验 = 最多 %d 次探测'
          % (len(ports), len(bauds), len(id_list), len(parities),
             len(ports) * len(bauds) * len(id_list) * len(parities)))
    print('单次超时 %.0f ms —— 全扫可能要几分钟，耐心等。\n' % (args.timeout * 1000))
    print('⚠️ 同一串口若已被桥进程占用，本扫描会全部超时（不是手坏了）。\n')

    hits = []
    for port in ports:
        for baud in bauds:
            for parity in parities:
                for sid in id_list:
                    r = try_probe(port, baud, parity, sid, args.timeout)
                    if r is None or r.get('kind') in (None, 'empty') or 'error' in r:
                        continue
                    if r['kind'] == 'exception':
                        desc = '异常应答 0x%02X = %s' % (r['code'], r['text'])
                    elif r['kind'] == 'data':
                        desc = '正常应答 reg0=%d' % r['regs'][0]
                    else:
                        desc = r['kind']
                    print('  ✔ %s @ %d 8%s1  id=%#04x  func=%#04x  %s'
                          % (port, baud, parity.upper(), sid, r.get('func', 0), desc))
                    print('       TX: %s' % hexs(r['tx']))
                    print('       RX: %s' % hexs(r['rx']))
                    hits.append((port, baud, parity, sid, r))
                # 这一组波特率/校验位已经有应答了，就不必再试另一种校验（省一半时间）
                if hits and hits[-1][0] == port and hits[-1][1] == baud and hits[-1][2] == parity:
                    continue

    print('\n== 小结 ==')
    if not hits:
        print('  没有任何应答。可能原因（按概率排序）：')
        print('   1) 串口被占用（先 --list 看有没有桥进程在跑）')
        print('   2) 波特率/校验位不在试过的组合里 —— 用 --baud/--parity 指定')
        print('   3) 手没有上电 / 手是 CAN 或 EtherCAT 而不是串口')
        print('   4) 手不发 Modbus，是私有协议（例如 Inspire）')
        return 1
    for port, baud, parity, sid, r in hits:
        print('  ✔ %s  波特率 %-8d  8%s1  从站ID %#04x'
              % (port, baud, parity.upper(), sid))
    print('\n  ⇒ 下一步：python3 %s --port <上面某个> --baud <上面某个> --id <上面某个> --read 0 12'
          % os.path.basename(__file__))
    return 0


# ---------------------------------------------------------------------------
# --read / --sweep / --raw / --write
# ---------------------------------------------------------------------------
def print_regs(base, regs, func):
    print('  addr(hex) addr(dec)  u16      int16    /1000')
    for i, v in enumerate(regs):
        a = base + i
        s16 = v - 0x10000 if v >= 0x8000 else v
        print('    %04X     %4d      %-8d %-8d %.3f'
              % (a, a, v, s16, v / 1000.0))


def cmd_read(args):
    fd, to = open_serial(args.port, args.baud, args.parity, args.stopbits, args.timeout)
    try:
        gap = frame_gap(args.baud)
        addr, count = args.read
        func = args.func
        tx, rx = transact(fd, req_read(args.id, func, addr, count), to, gap)
        print('端口 %s  波特率 %d  8%s%d  从站 %#04x  功能码 %#04x'
              % (args.port, args.baud, args.parity.upper(), args.stopbits, args.id, func))
        print('  TX: %s' % hexs(tx))
        print('  RX: %s' % (hexs(rx) if rx else '(空，超时)'))
        r = parse_response(rx, args.id, func)
        other = 0x03 if func == 0x04 else 0x04
        if r['kind'] == 'data':
            print_regs(addr, r['regs'], func)
            print('  → 读通了。注意同一个数值可能同时是「电流(0..255)」或「归一化位置(0..1000)」，')
            print('    量纲只能靠"动动手看它怎么变"来定：捏住手指看哪个地址在变。')
        elif r['kind'] == 'exception':
            print('  → 异常应答：%s' % r['text'])
            print('    **能回异常帧说明设备是活的**，只是这个功能码/地址它不认。')
            print('    换个功能码试：--func %#04x' % other)
        elif r['kind'] == 'crc_bad':
            print('  → CRC 不对。多半是波特率/校验位不对（收到的字节是错位的）。')
        elif r['kind'] == 'empty':
            print('  → 超时无应答。换个功能码试：--func %#04x' % other)
        else:
            print('  → 收到无法解析的字节：%s' % hexs(r.get('raw', b'')))
    finally:
        os.close(fd)
    return 0


def cmd_sweep(args):
    """逐地址读，摸出"哪些寄存器存在"。纯读操作，安全。"""
    fd, to = open_serial(args.port, args.baud, args.parity, args.stopbits, args.timeout)
    try:
        gap = frame_gap(args.baud)
        a0, a1 = args.sweep
        print('逐地址扫描 %d..%d，功能码 %#04x（只读）' % (a0, a1, args.func))
        ok = {}
        for a in range(a0, a1 + 1):
            payload = req_read(args.id, args.func, a, 1)
            tx, rx = transact(fd, payload, to, gap)
            r = parse_response(rx, args.id, args.func)
            if r['kind'] == 'data':
                ok[a] = r['regs'][0]
                print('  %4d (%04X): %-8d  int16 %-8d  /1000 %.3f'
                      % (a, a, r['regs'][0],
                         r['regs'][0] - 0x10000 if r['regs'][0] >= 0x8000 else r['regs'][0],
                         r['regs'][0] / 1000.0))
            elif r['kind'] == 'exception':
                print('  %4d (%04X): 异常 0x%02X (%s)' % (a, a, r['code'], r['text']))
            else:
                pass  # 超时/坏帧就不刷屏了
        print('\n可读地址共 %d 个：%s' % (len(ok), ', '.join('%d' % k for k in sorted(ok))))
        if len(ok) >= 2:
            # 相邻地址值相同很常见（版本号、常量），提示哪些像"会变的状态量"
            print('提示：连续地址里那些"值在 0..255 或 0..1000 之间且不像版本号"的，')
            print('      通常就是「当前角度 / 速度 / 转矩」。温度一般在 0..70。')
    finally:
        os.close(fd)
    return 0


def cmd_raw(args):
    payload = [int(x, 16) for x in args.raw.replace(',', ' ').split()]
    fd, to = open_serial(args.port, args.baud, args.parity, args.stopbits, args.timeout)
    try:
        gap = frame_gap(args.baud)
        append_crc = not args.no_crc
        if append_crc:
            tx = with_crc(payload)
        else:
            tx = bytes(payload)
        drain(fd)
        os.write(fd, tx)
        time.sleep(gap)
        rx = read_frame(fd, to)
        print('  TX: %s' % hexs(tx))
        print('  RX: %s' % (hexs(rx) if rx else '(空，超时)'))
        if rx:
            print('  CRC %s' % ('OK' if check_crc(rx) else 'BAD'))
    finally:
        os.close(fd)
    return 0


def cmd_write(args):
    """写寄存器。**默认拒绝执行**，必须 --yes。"""
    pairs = []
    for item in args.write:
        if '=' not in item:
            print('--write 的格式是 地址=值，例如 2=0', file=sys.stderr)
            return 2
        k, v = item.split('=', 1)
        pairs.append((int(k, 0), int(v, 0)))
    if not pairs:
        print('没有要写的寄存器。', file=sys.stderr)
        return 2

    sid = int(args.id, 0)
    parity = args.parity or 'n'

    pairs.sort()
    contiguous = all(pairs[i + 1][0] == pairs[i][0] + 1 for i in range(len(pairs) - 1))
    addr = pairs[0][0]
    values = [v for _, v in pairs]

    print('!! 即将写入 !!')
    print('   端口 %s  波特率 %d  8%s%d  从站 %#04x'
          % (args.port, args.baud, parity.upper(), args.stopbits, sid))
    for a, v in pairs:
        print('   寄存器 %d (%04X) ← %d (%04X)' % (a, a, v, v))
    if contiguous and len(values) > 1:
        frame = req_write_multi(sid, addr, values)
        print('   用功能码 0x10（写多个保持寄存器），负载：%s' % hexs(with_crc(frame)))
    else:
        for a, v in pairs:
            frame = req_write_single(sid, a, v)
            print('   用功能码 0x06（写单个保持寄存器）地址 %d，负载：%s'
                  % (a, hexs(with_crc(frame))))
        if not contiguous:
            print('   （地址不连续，逐条用 0x06 发）')

    if not args.yes:
        print('\n缺少 --yes，**没有发送任何数据**。确认上面字节无误后再加 --yes 重跑。')
        return 3

    fd, to = open_serial(args.port, args.baud, parity, args.stopbits, args.timeout)
    try:
        gap = frame_gap(args.baud)
        if contiguous and len(values) > 1:
            tx, rx = transact(fd, req_write_multi(sid, addr, values), to, gap)
            print('  TX: %s' % hexs(tx))
            print('  RX: %s' % (hexs(rx) if rx else '(空，超时)'))
        else:
            for a, v in pairs:
                tx, rx = transact(fd, req_write_single(sid, a, v), to, gap)
                print('  TX: %s' % hexs(tx))
                print('  RX: %s' % (hexs(rx) if rx else '(空，超时)'))
                time.sleep(gap)
        print('\n写完了。**手会真的动** —— 眼睛盯着它，手边留断电开关。')
        print('若回读值没变，可能是：使能寄存器没置位 / 单位模式不对 / 速度上限为 0。')
    finally:
        os.close(fd)
    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description='灵巧手 Modbus RTU 探针（纯标准库，给背包 Python 3.8 用）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='默认全部只读。写寄存器必须 --write 且 --yes。')
    ap.add_argument('--list', action='store_true', help='列出候选串口/udev/USB 信息（默认动作）')
    ap.add_argument('--scan', action='store_true', help='盲扫 端口×波特率×从站ID（只读）')
    ap.add_argument('--port', help='串口设备，如 /dev/ttyHand0（可重复：--port a --port b）',
                    action='append')
    ap.add_argument('--baud', type=int, default=0, help='波特率（不给则用常用表）')
    ap.add_argument('--parity', default='', choices=['', 'n', 'e', 'o'],
                    help='校验位 n/e/o（Modbus 标准是 e，实测三家手都是 n）')
    ap.add_argument('--stopbits', type=int, default=1, choices=[1, 2])
    ap.add_argument('--id', default='', help='从站 ID，可给多个用逗号分隔，如 0x27,0x28')
    ap.add_argument('--timeout', type=float, default=0.15, help='单次应答超时秒数（默认 0.15）')
    ap.add_argument('--func', type=lambda x: int(x, 0), default=0x04,
                    help='功能码：0x04 读输入寄存器（Linker 用这个）/ 0x03 读保持寄存器')
    ap.add_argument('--read', nargs=2, type=lambda x: int(x, 0), metavar=('ADDR', 'COUNT'),
                    help='读一段寄存器')
    ap.add_argument('--sweep', nargs=2, type=lambda x: int(x, 0), metavar=('A0', 'A1'),
                    help='逐地址扫描（只读）')
    ap.add_argument('--raw', help="发原始十六进制帧，如 '7e 04 00 00 00 06'")
    ap.add_argument('--no-crc', action='store_true', help='--raw 时不要自动补 CRC')
    ap.add_argument('--write', action='append', metavar='ADDR=VAL',
                    help='写寄存器（危险，需要 --yes）')
    ap.add_argument('--yes', action='store_true', help='确认执行 --write')
    args = ap.parse_args()

    # --port 是 append 收集的列表。--scan 要列表（多端口盲扫），其余模式只要一个。
    ports = args.port or []
    if args.scan:
        args.port = ports                      # 保持列表
        return cmd_scan(args)
    if len(ports) > 1:
        print('除 --scan 外只支持一个 --port。', file=sys.stderr)
        return 2
    args.port = ports[0] if ports else None

    if args.write:
        if not args.port:
            print('--write 必须显式给 --port（不要用通配）。', file=sys.stderr)
            return 2
        if not args.baud:
            print('--write 必须显式给 --baud。', file=sys.stderr)
            return 2
        if not args.id:
            print('--write 必须显式给 --id。', file=sys.stderr)
            return 2
        return cmd_write(args)

    if args.scan:
        return cmd_scan(args)

    if args.raw:
        if not args.port or not args.baud:
            print('--raw 需要 --port 和 --baud。', file=sys.stderr)
            return 2
        if not args.parity:
            args.parity = 'n'
        return cmd_raw(args)

    if args.read or args.sweep:
        if not args.port or not args.baud or not args.id:
            print('--read/--sweep 需要 --port、--baud、--id。', file=sys.stderr)
            return 2
        if not args.parity:
            args.parity = 'n'
        if args.read and args.read[0] is not None:
            args.id = int(args.id, 0)
            return cmd_read(args)
        args.id = int(args.id, 0)
        return cmd_sweep(args)

    return cmd_list()


if __name__ == '__main__':
    sys.exit(main())
