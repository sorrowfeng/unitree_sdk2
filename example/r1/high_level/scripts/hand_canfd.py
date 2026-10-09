#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""灵巧手 CANFD 直控封装（不依赖 LHandPro SDK）。

为什么不用 SDK
--------------
厂商 SDK（`libLHandProLib.so`）是在 **Ubuntu 22.04 / GCC 11** 上编的，要求
`GLIBCXX_3.4.30` + **`GLIBC_2.34`**；而 R1 算力背包是 **Ubuntu 20.04 / glibc 2.31**，
根本加载不了。但手的 CAN 帧协议极简单，直接发原始帧即可，**完全绕开 SDK**。

协议（由厂商示例帧逐字节解出，已自检）
--------------------------------------
CAN ID      = `0x500 + node_id`（node_id 默认：左手 1、右手 2）
数据段 14B  = `<cmd:1> <轴数:1=0x06> <6×2B 小端数值>`

    cmd = 0x01  模式/状态：0x0120 使能 · 0x0425 回零 · 0x0020 切回位置模式
    cmd = 0x02  位置      ：0..10000（总行程系数 10000 ⇒ 0x03E8 = 1000 = 10% 行程）
    cmd = 0x03  速度      ：0x07D0 = 2000（≈0.5 s 走完全行程）
    cmd = 0x04  电流上限  ：0x03E8 = 1000（千分比 ⇒ 100% 额定电流）

反馈与回环（**必须分清，否则会误判"手在应答"**）
------------------------------------------------
    `0x500 + node`  ← 适配器把自己的发送**回环**回来，**不是**手的反馈
    `0x480 + node`  ← 真正的控制反馈
    `0x580 + node`  ← 真正的 SDO 反馈

用法
----
    from hand_canfd import LHandCanfd
    bus = LHandCanfd()
    bus.open()
    bus.init_hand(1)                   # 使能 → 回零(等5s) → 位置模式 → 速度/电流
    bus.set_position(1, 1000)          # 走到 10% 行程
    bus.close()

    # 想看反馈（确认手在线）时才开：
    bus.start_feedback(1)              # 发 00 02 50 01，之后持续收 0x480+node
"""

import time
from typing import Callable, Dict, Iterable, Optional, Sequence

import canfd_lib
from canfd_lib import CANFD, CANFDException

__all__ = ["LHandCanfd", "CANFDException"]

# ---------------------------------------------------------------------------
# 协议常量（照抄厂商示例，别再改）
# ---------------------------------------------------------------------------
CMD_BASE = 0x500          # 发送基地址：CAN ID = 0x500 + node_id
FB_CMD_BASE = 0x480       # 控制反馈：0x480 + node_id
FB_SDO_BASE = 0x580       # SDO 反馈：0x580 + node_id
FB_SPAN = 0x80            # 每个节点区间宽度

AXES = 6                  # 一条帧一次写完 6 个轴
DATA_LEN = 2 + AXES * 2   # 14 字节 = cmd(1) + 轴数(1) + 6×2B

CMD_MODE = 0x01           # 模式 / 状态
CMD_POSITION = 0x02       # 位置
CMD_VELOCITY = 0x03       # 速度
CMD_CURRENT = 0x04        # 电流上限

MODE_ENABLE = 0x0120      # 每个轴使能
MODE_HOME = 0x0425        # 每个轴回零
MODE_POSITION = 0x0020    # 切回位置模式

# 开启**异步反馈上报**的报文（厂商提供）：`00 02 50 01`。
# 实测：**运动本身不需要它**（真机 ID 1 未开反馈也能动）；
# 只有想读状态、确认"手在线"时才发。发了之后设备持续上报 `0x480+node`；
# 不发则只看到自己的回环帧，很容易误判成"通讯没通 / 手没应答"。
FEEDBACK_START_FRAME = bytes.fromhex("00025001")

DEFAULT_VELOCITY = 2000   # ≈0.5 s 走完全行程
DEFAULT_CURRENT = 1000    # 100% 额定电流
POSITION_MAX = 10000      # 总行程系数


def build_payload(cmd: int, values: Sequence[int]) -> bytes:
    """组一帧 14 字节数据段：<cmd> <06> <6×2B 小端>。

    values 长度 1 ⇒ 6 个轴同值；长度 6 ⇒ 逐轴。
    """
    if len(values) == 1:
        values = list(values) * AXES
    if len(values) != AXES:
        raise ValueError(f"values 必须是 1 个或 {AXES} 个，收到 {len(values)}")
    out = bytearray([cmd & 0xFF, AXES])
    for v in values:
        v = int(v)
        if not 0 <= v <= 0xFFFF:
            raise ValueError(f"数值 {v} 超出 uint16")
        out += v.to_bytes(2, "little")
    return bytes(out)


class LHandCanfd:
    """一条 CANFD 总线上的多只灵巧手。

    只负责"发帧 + 分类收帧"，不做轮询调度（CANFD 由设备自行上报反馈，
    与 RS485 需要共享总线轮询不同）。
    """

    def __init__(self, driver: str = "gsusb",
                 nom_baudrate: int = 1_000_000, dat_baudrate: int = 5_000_000,
                 device_index: Optional[int] = None,
                 log_frames: bool = True,
                 log: Optional[Callable[[str], None]] = print):
        self._adapter = CANFD(driver=driver)
        self._nom = nom_baudrate
        self._dat = dat_baudrate
        self._device_index = device_index
        self._log = log if log is not None else (lambda _m: None)
        # ⚠️ 逐帧打印 TX/RX。开启反馈后设备约 1000 帧/秒持续上报，正常使用务必设 False
        #    （长跑的 hand_bridge.py 就是）；生命周期日志（扫描/连接/初始化/关闭）不受影响。
        self._log_frames = log_frames
        self._opened = False

        self._rx_count = 0                       # 收到的全部帧
        self._echo_count = 0                     # 其中：自己的回环
        self._tx_ids = set()                     # 我们发过的 CAN ID
        self._fb_by_node: Dict[int, int] = {}    # 真正的设备反馈（按节点）
        self._last_fb: Dict[int, bytes] = {}
        self._other_ids: Dict[int, int] = {}     # 其它 ID（便于发现未知设备）

    # ------------------------------------------------------------------ 连接
    def open(self) -> int:
        """扫描并连接适配器。返回扫到的设备数（0 表示没插好）。"""
        n = self._adapter.scan()
        self._log(f"[canfd] 扫描到设备数: {n}")
        if n == 0:
            raise CANFDException("未找到 CANFD 适配器（检查 USB 连接 / udev 权限）")
        idx = 0 if self._device_index is None else self._device_index
        if not 0 <= idx < n:
            raise CANFDException(f"设备索引 {idx} 无效（共 {n} 个）")
        self._log(f"[canfd] 连接 index={idx} nom={self._nom} data={self._dat}")
        if not self._adapter.connect(device_index=idx, channel_index=0,
                                     nom_baudrate=self._nom,
                                     dat_baudrate=self._dat):
            raise CANFDException("适配器连接失败")
        self._adapter.set_receive_callback(self._on_frame)
        self._opened = True
        self._log("[canfd] 已连接，接收回调已注册")
        return n

    def close(self) -> None:
        if not self._opened:
            return
        try:
            self._adapter.set_receive_callback(None)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._adapter.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self._opened = False
        self._log(f"[canfd] 已断开。RX 总计 {self._rx_count}"
                  f"（回环 {self._echo_count}，设备反馈 {sum(self._fb_by_node.values())}"
                  f"{'，其它 ' + str(sum(self._other_ids.values())) if self._other_ids else ''}）")

    def _on_frame(self, msg: dict) -> None:
        """接收回调：把"自己的回环"与"设备反馈"分开，避免误判。"""
        can_id = int(msg.get("id", 0))
        data = bytes(msg.get("data", b""))
        self._rx_count += 1

        if can_id in self._tx_ids:
            self._echo_count += 1
            kind, node = "回环(自己发的)", can_id - CMD_BASE
        elif FB_CMD_BASE <= can_id < FB_CMD_BASE + FB_SPAN:
            node = can_id - FB_CMD_BASE
            kind = "★控制反馈"
            self._fb_by_node[node] = self._fb_by_node.get(node, 0) + 1
            self._last_fb[node] = data
        elif FB_SDO_BASE <= can_id < FB_SDO_BASE + FB_SPAN:
            node = can_id - FB_SDO_BASE
            kind = "★SDO反馈"
            self._fb_by_node[node] = self._fb_by_node.get(node, 0) + 1
            self._last_fb[node] = data
        else:
            kind, node = "?", -1
            self._other_ids[can_id] = self._other_ids.get(can_id, 0) + 1

        if self._log_frames:
            self._log(f"[canfd] RX id=0x{can_id:03X} ({kind} node={node}) "
                      f"len={len(data)} data={data[:16].hex(' ').upper()}"
                      + (" ..." if len(data) > 16 else ""))

    # ------------------------------------------------------------------ 统计
    @property
    def rx_count(self) -> int:
        return self._rx_count

    @property
    def echo_count(self) -> int:
        return self._echo_count

    @property
    def fb_by_node(self) -> Dict[int, int]:
        """真正的设备反馈（0x480+node / 0x580+node），按节点计数。"""
        return dict(self._fb_by_node)

    @property
    def last_fb(self) -> Dict[int, bytes]:
        return dict(self._last_fb)

    @property
    def other_ids(self) -> Dict[int, int]:
        return dict(self._other_ids)

    # ------------------------------------------------------------------ 发送
    def can_id(self, node: int) -> int:
        return CMD_BASE + int(node)

    def send_cmd(self, node: int, cmd: int, values: Sequence[int],
                 settle_s: float = 0.0) -> int:
        """发一条命令帧，返回 CAN ID。"""
        payload = build_payload(cmd, values)
        can_id = self.can_id(node)
        self._tx_ids.add(can_id)
        self._adapter.send(can_id, payload)
        if self._log_frames:
            self._log(f"[canfd] TX node={node} id=0x{can_id:03X} "
                  f"cmd=0x{cmd:02X} data={payload.hex(' ').upper()}")
        if settle_s > 0:
            time.sleep(settle_s)
        return can_id

    def send_raw(self, node: int, payload: bytes, settle_s: float = 0.0) -> int:
        """直接发原始数据段（14 字节），用于逐字节对照厂商示例。"""
        can_id = self.can_id(node)
        self._tx_ids.add(can_id)
        self._adapter.send(can_id, bytes(payload))
        if self._log_frames:
            self._log(f"[canfd] TX(raw) node={node} id=0x{can_id:03X} "
                  f"data={bytes(payload).hex(' ').upper()}")
        if settle_s > 0:
            time.sleep(settle_s)
        return can_id

    # ------------------------------------------------------------ 语义化 API
    def start_feedback(self, node: int, settle_s: float = 0.3) -> int:
        """开启该节点的**异步反馈上报**（发 `00 02 50 01`）。

        必须在任何其它命令之前调用。发完设备会持续上报 `0x480+node`，
        这才有办法确认"通讯真的通了、手真的在"。
        """
        self._log(f"[hand {node}] 开启异步反馈上报 ({FEEDBACK_START_FRAME.hex(' ').upper()})")
        return self.send_raw(node, FEEDBACK_START_FRAME, settle_s)

    def enable(self, node: int, settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_MODE, [MODE_ENABLE], settle_s)

    def home(self, node: int, settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_MODE, [MODE_HOME], settle_s)

    def position_mode(self, node: int, settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_MODE, [MODE_POSITION], settle_s)

    def set_velocity(self, node: int, value: int = DEFAULT_VELOCITY,
                     settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_VELOCITY, [value], settle_s)

    def set_current(self, node: int, value: int = DEFAULT_CURRENT,
                    settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_CURRENT, [value], settle_s)

    def set_position(self, node: int, value: int, settle_s: float = 0.0) -> int:
        return self.send_cmd(node, CMD_POSITION, [value], settle_s)

    def set_positions(self, node: int, values: Sequence[int],
                      settle_s: float = 0.0) -> int:
        """6 个轴各自的位置。"""
        return self.send_cmd(node, CMD_POSITION, values, settle_s)

    def init_hand(self, node: int, home_wait: float = 5.0,
                  enable_wait: float = 1.0,
                  velocity: int = DEFAULT_VELOCITY,
                  current: int = DEFAULT_CURRENT,
                  feedback: bool = False) -> None:
        """初始化三步 + 运动参数：使能 → 回零(等 home_wait) → 位置模式 → 速度/电流。

        `feedback=True` 时**额外**先发 `00 02 50 01` 开启异步反馈上报。
        实测：**运动本身不需要开反馈**（真机 ID 1 在没开反馈时也动了）；
        只有想读状态、确认"手在线"时才需要开 —— 开了会持续上报 `0x480+node`。
        """
        if feedback:
            self.start_feedback(node)
        self._log(f"[hand {node}] 使能 ...")
        self.enable(node)
        time.sleep(enable_wait)
        self._log(f"[hand {node}] 回零（等 {home_wait:g}s）...")
        self.home(node)
        time.sleep(home_wait)
        self._log(f"[hand {node}] 切回位置模式 ...")
        self.position_mode(node)
        self.set_velocity(node, velocity)
        self.set_current(node, current)
        self._log(f"[hand {node}] 初始化完成")

    def init_hands(self, nodes: Iterable[int], **kw) -> None:
        for n in nodes:
            self.init_hand(n, **kw)
