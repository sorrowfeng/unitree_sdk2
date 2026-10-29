#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""灵巧手 CANFD 直控封装（不依赖 LHandPro SDK）—— **多总线版**。

拓扑（2026-10-29 起）
---------------------
**两条独立的 USB-CANFD 总线，各挂一只手，两只手的节点号都是 1**：

    USB-CANFD #0 ── CANFD 总线 A ── 左手（node 1）
    USB-CANFD #1 ── CANFD 总线 B ── 右手（node 1）

> 以前是"一条总线挂两个节点（1/2）"，靠 node id 区分左右；
> 现在两条总线上的 node 都是 1，**靠总线区分左右**（`HandBus.device_index`）。
> 所以 CAN ID 在两条总线上是相同的（都 `0x501`/`0x481`），互不干扰。

协议（不变，由厂商示例帧逐字节解出，脚本内有自检）
--------------------------------------------------
    CAN ID    = `0x500 + node_id`
    14B 数据  = `<cmd:1> <轴数:1=0x06> <6×2 字节小端数值>`
      cmd 0x01 模式：0x0120 使能 · 0x0425 回零 · 0x0020 切回位置模式
      cmd 0x02 位置（0..10000，总行程系数 10000 ⇒ 0x03E8=1000=10% 行程）
      cmd 0x03 速度（0x07D0=2000 ≈0.5 s 走完全行程）
      cmd 0x04 电流上限（0x03E8=1000 = 100% 额定，千分比）
    反馈      = `0x480+node`（控制）/ `0x580+node`（SDO）

反馈与回环（**必须分清，否则会误判"手在应答"**）
    `0x500+node` ← 适配器把自己的发送**回环**回来，**不是**手的反馈

用法
----
    from hand_canfd import HandBus, LHandCanfd

    bus = LHandCanfd(hands=[HandBus("left",  device_index=0),
                            HandBus("right", device_index=1)])
    bus.open()
    bus.init_hand("left")                  # 使能 → 回零 → 位置模式 → 速度/电流
    bus.set_position("left", 1000)         # 走到 10% 行程
    bus.close()

    # 想看反馈（确认手在线）时才开：
    bus.start_feedback("left")             # 发 00 02 50 01，之后持续收 0x480+node
"""

import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence

import canfd_lib
from canfd_lib import CANFD, CANFDException

__all__ = ["HandBus", "LHandCanfd", "CANFDException", "build_payload"]

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
# 实测：**运动本身不需要它**；只有想读状态、确认"手在线"时才发。
FEEDBACK_START_FRAME = bytes.fromhex("00025001")

DEFAULT_VELOCITY = 2000   # ≈0.5 s 走完全行程
DEFAULT_CURRENT = 1000    # 100% 额定电流
POSITION_MAX = 10000      # 总行程系数
DEFAULT_NODE_HINT = 1     # 两条总线上的节点号（现在两只手都是 1）


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


@dataclass(frozen=True)
class HandBus:
    """一只手 = 一条 CANFD 总线 + 该总线上的节点号。

    `device_index` 是 USB-CANFD 适配器的序号（0-based，按 `scan_adapters()` 顺序）。
    两条总线上的手**节点号都是 1**，靠总线本身区分左右。
    """
    side: str                 # "left" / "right"
    device_index: int = 0     # 哪个 USB-CANFD 适配器
    node: int = 1             # 该总线上的节点号


class LHandCanfd:
    """多总线灵巧手控制器：每条总线一个 `CANFD` 实例，按 side 路由。

    只负责"发帧 + 分类收帧"，不做轮询调度（CANFD 由设备自行上报反馈）。
    """

    def __init__(self,
                 hands: Optional[Sequence[HandBus]] = None,
                 driver: str = "gsusb",
                 nom_baudrate: int = 1_000_000,
                 dat_baudrate: int = 5_000_000,
                 log_frames: bool = True,
                 log: Optional[Callable[[str], None]] = print):
        if hands is None:
            hands = [HandBus("left", device_index=0), HandBus("right", device_index=1)]
        self._hands: Dict[str, HandBus] = {}
        for h in hands:
            if h.side in self._hands:
                raise ValueError(f"side 重复: {h.side}")
            self._hands[h.side] = h
        self._driver = driver
        self._nom = nom_baudrate
        self._dat = dat_baudrate
        self._log = log if log is not None else (lambda _m: None)
        # ⚠️ 逐帧打印 TX/RX。开启反馈后约 1000 帧/秒持续上报，长跑务必设 False。
        self._log_frames = log_frames

        self._adapters: Dict[str, CANFD] = {}
        self._opened = False

        self._rx_count = 0                            # 收到的全部帧
        self._echo_count = 0                          # 其中：自己的回环
        self._fb_by_side: Dict[str, int] = {}         # 真正的设备反馈（按 side）
        self._last_fb: Dict[str, bytes] = {}
        self._other: Dict[str, Dict[int, int]] = {}   # 其它 CAN ID（按 side）
        self._tx_ids: Dict[str, set] = {}             # 每条总线发过的 CAN ID

    # ------------------------------------------------------------------ 属性
    @property
    def sides(self) -> List[str]:
        return list(self._hands)

    def spec(self, side: str) -> HandBus:
        try:
            return self._hands[side]
        except KeyError:
            raise KeyError(f"未知的 side {side!r}（有: {self.sides}）") from None

    # ------------------------------------------------------------------ 连接
    def open(self) -> int:
        """打开每一条总线。返回成功打开的条数（不是 len(sides) 就说明适配器有问题）。"""
        adapters = self._probe_adapters()
        if adapters is not None:
            self._log(f"[canfd] 扫描到 {len(adapters)} 个 USB-CANFD 适配器：")
            for i, a in enumerate(adapters):
                self._log(f"[canfd]   #{i}  serial={getattr(a, 'serial', '?')!r} "
                          f"bus={getattr(a, 'bus', '?')} addr={getattr(a, 'address', '?')} "
                          f"product={getattr(a, 'product', '?')!r}")
        else:
            self._log("[canfd] （无法列出适配器详情，继续按索引连接）")

        opened = 0
        for side, spec in self._hands.items():
            adapter = CANFD(driver=self._driver)
            n = adapter.scan()
            if n == 0:
                raise CANFDException(f"[{side}] 未找到 CANFD 适配器（检查 USB 连接 / udev 权限）")
            if not 0 <= spec.device_index < n:
                raise CANFDException(
                    f"[{side}] device_index={spec.device_index} 无效（只扫到 {n} 个适配器）")
            self._log(f"[canfd] [{side}] 连接 device_index={spec.device_index} "
                      f"node={spec.node} nom={self._nom} data={self._dat}")
            if not adapter.connect(device_index=spec.device_index, channel_index=0,
                                   nom_baudrate=self._nom, dat_baudrate=self._dat):
                raise CANFDException(f"[{side}] 适配器连接失败")
            adapter.set_receive_callback(lambda msg, s=side: self._on_frame(s, msg))
            self._adapters[side] = adapter
            self._tx_ids.setdefault(side, set())
            opened += 1
        self._opened = True
        self._log(f"[canfd] 已连接 {opened} 条总线，接收回调已注册")
        return opened

    @staticmethod
    def _probe_adapters():
        """尽力列出适配器详情（可选；拿不到就返回 None）。"""
        try:
            import gsusb_canfd
            return list(gsusb_canfd.scan_adapters())
        except Exception:  # noqa: BLE001
            return None

    def close(self) -> None:
        if not self._opened:
            return
        for side, adapter in self._adapters.items():
            try:
                adapter.set_receive_callback(None)
            except Exception:  # noqa: BLE001
                pass
            try:
                adapter.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._log(f"[canfd] [{side}] 已断开")
        self._opened = False
        self._log(f"[canfd] RX 总计 {self._rx_count}（回环 {self._echo_count}，"
                  f"设备反馈 {sum(self._fb_by_side.values())}）")

    def _on_frame(self, side: str, msg: dict) -> None:
        """接收回调：把"自己的回环"与"设备反馈"分开，避免误判。"""
        can_id = int(msg.get("id", 0))
        data = bytes(msg.get("data", b""))
        self._rx_count += 1

        if can_id in self._tx_ids.get(side, set()):
            self._echo_count += 1
            kind, node = "回环(自己发的)", can_id - CMD_BASE
        elif FB_CMD_BASE <= can_id < FB_CMD_BASE + FB_SPAN:
            node = can_id - FB_CMD_BASE
            kind = "★控制反馈"
            self._fb_by_side[side] = self._fb_by_side.get(side, 0) + 1
            self._last_fb[side] = data
        elif FB_SDO_BASE <= can_id < FB_SDO_BASE + FB_SPAN:
            node = can_id - FB_SDO_BASE
            kind = "★SDO反馈"
            self._fb_by_side[side] = self._fb_by_side.get(side, 0) + 1
            self._last_fb[side] = data
        else:
            kind, node = "?", -1
            per = self._other.setdefault(side, {})
            per[can_id] = per.get(can_id, 0) + 1

        if self._log_frames:
            self._log(f"[canfd] [{side}] RX id=0x{can_id:03X} ({kind} node={node}) "
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
    def fb_by_side(self) -> Dict[str, int]:
        """真正的设备反馈（0x480+node / 0x580+node），按 side 计数。"""
        return dict(self._fb_by_side)

    @property
    def last_fb(self) -> Dict[str, bytes]:
        return dict(self._last_fb)

    @property
    def other_ids(self) -> Dict[str, Dict[int, int]]:
        return {k: dict(v) for k, v in self._other.items()}

    # ------------------------------------------------------------------ 发送
    def can_id(self, side: str) -> int:
        return CMD_BASE + self.spec(side).node

    def send_cmd(self, side: str, cmd: int, values: Sequence[int],
                 settle_s: float = 0.0) -> int:
        """在 side 对应的那条总线上发一条命令帧，返回 CAN ID。"""
        payload = build_payload(cmd, values)
        can_id = self.can_id(side)
        adapter = self._adapters.get(side)
        if adapter is None:
            raise CANFDException(f"[{side}] 总线未打开")
        self._tx_ids.setdefault(side, set()).add(can_id)
        adapter.send(can_id, payload)
        if self._log_frames:
            self._log(f"[canfd] [{side}] TX id=0x{can_id:03X} "
                      f"cmd=0x{cmd:02X} data={payload.hex(' ').upper()}")
        if settle_s > 0:
            time.sleep(settle_s)
        return can_id

    def send_raw(self, side: str, payload: bytes, settle_s: float = 0.0) -> int:
        """直接发原始数据段，用于逐字节对照厂商示例 / 发非 14B 的特殊帧。"""
        return self.send_raw_node(side, self.spec(side).node, payload, settle_s)

    def send_raw_node(self, side: str, node: int, payload: bytes,
                      settle_s: float = 0.0) -> int:
        """在 side 那条总线上，按**指定 node** 发原始帧。

        用于扫描节点号（此时还不知道节点是几，不能用自带的 node）。
        """
        can_id = CMD_BASE + node
        adapter = self._adapters.get(side)
        if adapter is None:
            raise CANFDException(f"[{side}] 总线未打开")
        self._tx_ids.setdefault(side, set()).add(can_id)
        adapter.send(can_id, bytes(payload))
        if self._log_frames:
            self._log(f"[canfd] [{side}] TX(raw) id=0x{can_id:03X} "
                      f"data={bytes(payload).hex(' ').upper()}")
        if settle_s > 0:
            time.sleep(settle_s)
        return can_id

    # ------------------------------------------------------------ 语义化 API
    def start_feedback(self, side: str, settle_s: float = 0.3) -> int:
        """开启该手所在总线的**异步反馈上报**（发 `00 02 50 01`）。"""
        self._log(f"[{side}] 开启异步反馈上报 ({FEEDBACK_START_FRAME.hex(' ').upper()})")
        return self.send_raw(side, FEEDBACK_START_FRAME, settle_s)

    def enable(self, side: str, settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_MODE, [MODE_ENABLE], settle_s)

    def home(self, side: str, settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_MODE, [MODE_HOME], settle_s)

    def position_mode(self, side: str, settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_MODE, [MODE_POSITION], settle_s)

    def set_velocity(self, side: str, value: int = DEFAULT_VELOCITY,
                     settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_VELOCITY, [value], settle_s)

    def set_current(self, side: str, value: int = DEFAULT_CURRENT,
                    settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_CURRENT, [value], settle_s)

    def set_position(self, side: str, value: int, settle_s: float = 0.0) -> int:
        return self.send_cmd(side, CMD_POSITION, [value], settle_s)

    def set_positions(self, side: str, values: Sequence[int],
                      settle_s: float = 0.0) -> int:
        """6 个轴各自的位置。"""
        return self.send_cmd(side, CMD_POSITION, values, settle_s)

    def init_hand(self, side: str, home_wait: float = 5.0,
                  enable_wait: float = 1.0,
                  velocity: int = DEFAULT_VELOCITY,
                  current: int = DEFAULT_CURRENT,
                  feedback: bool = False) -> None:
        """初始化三步 + 运动参数：使能 → 回零(等 home_wait) → 位置模式 → 速度/电流。

        `feedback=True` 时**额外**先发 `00 02 50 01` 开启异步反馈上报。
        """
        if feedback:
            self.start_feedback(side)
        self._log(f"[{side}] 使能 ...")
        self.enable(side)
        time.sleep(enable_wait)
        self._log(f"[{side}] 回零（等 {home_wait:g}s）...")
        self.home(side)
        time.sleep(home_wait)
        self._log(f"[{side}] 切回位置模式 ...")
        self.position_mode(side)
        self.set_velocity(side, velocity)
        self.set_current(side, current)
        self._log(f"[{side}] 初始化完成")

    def init_hands(self, sides: Optional[Iterable[str]] = None, **kw) -> None:
        for s in (sides if sides is not None else self.sides):
            self.init_hand(s, **kw)
