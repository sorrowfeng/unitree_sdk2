#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CANFD通信库封装
提供扫描、连接、断开、发送以及接收回调功能

平台与驱动（默认优先 gsusb-canfd，不可用或无设备时自动回退到平台原生后端）:
  Windows:    gsusb-canfd（pyusb + 随示例附带的 libusb-1.0.dll）
              → HCanbus.dll（ctypes）
  Linux:      gsusb-canfd（pyusb）
              → "socketcan"（python-can）或 "libcanbus"（厂商 .so）
              优先级: USE_LIBCANBUS 宏 > driver 参数

driver 参数可强制指定后端（不做回退，失败即抛 CANFDException）:
  "gsusb" / "hcanbus" / "socketcan" / "libcanbus"

gsusb-canfd 是纯用户态 gs_usb 实现，与 C++ 示例的默认后端同源（上游
gsusb-canfd 仓库的 Python 包），Windows / Linux / macOS 共用同一段逻辑。
"""

# 设为 True 时优先使用 gsusb-canfd（用户态 gs_usb 驱动，与 C++ 默认后端一致）。
# 置为 False 表示跳过 gsusb，直接使用平台原生后端。
USE_GSUSB_CANFD = True

# 设为 True 时使用 Linux 厂商 libcanbus.so；False 时使用 socketcan（python-can）
USE_LIBCANBUS = False

import os
import sys
import threading
import time
import ctypes
from typing import Optional, Callable

# 常量定义
STATUS_OK = 0
DLC2LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
IS_WINDOWS = sys.platform.startswith("win")


class CANFDException(Exception):
    """CANFD操作异常"""
    pass


def _len_to_dlc(length: int) -> int:
    if length <= 8:
        return length
    elif length <= 12:
        return 9
    elif length <= 16:
        return 10
    elif length <= 20:
        return 11
    elif length <= 24:
        return 12
    elif length <= 32:
        return 13
    elif length <= 48:
        return 14
    else:
        return 15


# =============================================================================
# gsusb-canfd 后端：上游纯 Python 包 + pyusb（三平台通用）
# =============================================================================
# 与 C++ 示例的默认后端同源——同一个 gs_usb 用户态实现，不依赖厂商 DLL。
# 依赖两项外部条件，任一缺失即视为不可用并回退到平台原生后端：
#   1) pyusb（pip install "pyusb>=1.2.1"）
#   2) gsusb_canfd 包（pip install gsusb-canfd，或随示例提供的源码）
# 运行时还需要 libusb-1.0.dll 对 pyusb 可见，见 _ensure_libusb_visible()。

_LIBUSB_DLL_NAME = "libusb-1.0.dll"
_GSUSB_PACKAGE_NAME = "gsusb_canfd"

_gsusb_module = None
_gsusb_error = None
_gsusb_loaded = False
_dll_dir_handles = []


def _example_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _gsusb_source_roots() -> list:
    """按优先级列出可能存放 gsusb_canfd 包的源码根目录。"""
    base = _example_dir()
    return [
        # 安装包布局：examples/CANFD_python/gsusb_canfd/
        base,
        # 仓库内开发布局：LHandProLib_CANFD_Test_cpp/gsusb-canfd/python/src/
        os.path.join(base, os.pardir, "LHandProLib_CANFD_Test_cpp",
                     "gsusb-canfd", "python", "src"),
        # 安装包布局（回退）：examples/CANFD_cpp/gsusb-canfd/python/src/
        os.path.join(base, os.pardir, "CANFD_cpp",
                     "gsusb-canfd", "python", "src"),
    ]


def _ensure_libusb_visible() -> None:
    """把随示例附带的 libusb-1.0.dll 暴露给 pyusb（Windows）。

    pyusb 通过 ctypes.util.find_library('libusb-1.0.dll') 定位后端，而该函数在
    Windows 上只遍历 PATH（不认 os.add_dll_directory），因此必须在第一次
    usb.core.find() 之前把 DLL 所在目录写进 PATH，否则只会得到
    NoBackendError: No backend available。
    """
    if not IS_WINDOWS:
        return

    base = _example_dir()
    candidates = []
    if sys.maxsize <= 2 ** 32:  # 32 位解释器优先取 lib/x86
        candidates.append(os.path.join(base, "lib", "x86"))
    candidates += [os.path.join(base, "lib"), base]

    for directory in candidates:
        if not os.path.isfile(os.path.join(directory, _LIBUSB_DLL_NAME)):
            continue
        entries = os.environ.get("PATH", "").split(os.pathsep)
        if directory not in entries:
            os.environ["PATH"] = directory + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            try:
                # 保留句柄：add_dll_directory 返回的句柄一旦被回收即失效
                _dll_dir_handles.append(
                    os.add_dll_directory(os.path.abspath(directory)))
            except OSError:
                pass
        return


def _load_gsusb():
    """惰性加载 gsusb_canfd 包（结果缓存）。返回 (module, error)。"""
    global _gsusb_loaded, _gsusb_module, _gsusb_error
    if _gsusb_loaded:
        return _gsusb_module, _gsusb_error
    _gsusb_loaded = True

    _ensure_libusb_visible()

    try:
        import gsusb_canfd as module  # 已 pip 安装到 site-packages
        _gsusb_module = module
        return _gsusb_module, None
    except Exception as exc:  # noqa: BLE001
        _gsusb_error = exc

    for root in _gsusb_source_roots():
        if not os.path.isfile(
                os.path.join(root, _GSUSB_PACKAGE_NAME, "__init__.py")):
            continue
        if root not in sys.path:
            sys.path.insert(0, root)
        try:
            import gsusb_canfd as module
            _gsusb_module = module
            _gsusb_error = None
            return _gsusb_module, None
        except Exception as exc:  # noqa: BLE001
            _gsusb_error = exc

    return None, _gsusb_error


class _GsUsbCANFD:
    """gs_usb 用户态实现（上游 gsusb_canfd + pyusb），三平台共用。

    与 C++ 侧 gsusb 后端保持一致的语义：
      * 采样点固定 1Mbps@80% / 5Mbps@75%，使能 CAN FD；
      * 发送固定 64 字节帧（brs=1），实际数据放在前 N 字节；
      * 用 open(adapter) 精确打开扫描到的适配器，并开启 drop_echo；
      * 接收交给库自带的工作线程持续排空——gs_usb 每个 USB 包只承载一帧，
        而灵巧手反馈流约 2kHz；若每收一帧就 sleep，接收端追不上到达速率，
        适配器 RX FIFO 会持续溢出并随机丢弃 SDO 应答（0x581），使 SDK 在
        50ms 超时后报 LER_COMM_RECV(7)。

    需要 gsusb_canfd >= 0.1.2：open(adapter) 与 BusConfig.drop_echo 由该版本引入。
    """

    _SAMPLE_POINT = 0.80
    _DATA_SAMPLE_POINT = 0.75
    _FRAME_SIZE = 64

    def __init__(self):
        module, error = _load_gsusb()
        if module is None:
            raise CANFDException(f"gsusb_canfd 后端不可用: {error}")
        self._gsusb = module
        self._bus = None
        self._is_connected = False
        self._adapters = []
        self._receive_callback: Optional[Callable] = None
        self._rx_started = False

    def _scan_adapters(self) -> list:
        self._adapters = list(self._gsusb.scan_adapters())
        return self._adapters

    def scan(self) -> int:
        try:
            return len(self._scan_adapters())
        except CANFDException:
            raise
        except Exception as exc:  # noqa: BLE001
            raise CANFDException(f"gsusb 扫描设备异常: {exc}")

    def connect(self, device_index: int = 0, channel_index: int = 0,
                nom_baudrate: int = 1000000, dat_baudrate: int = 5000000,
                nom_sampling: int = 0, dat_sampling: int = 0) -> bool:
        if self._is_connected:
            self.disconnect()

        try:
            adapters = self._scan_adapters()
        except Exception as exc:  # noqa: BLE001
            raise CANFDException(f"gsusb 扫描设备异常: {exc}")

        if not 0 <= device_index < len(adapters):
            raise CANFDException(
                f"gsusb 设备索引无效: {device_index}"
                f"（检测到 {len(adapters)} 个适配器）")

        # 按扫描结果精确打开该适配器:open(adapter) 优先用 serial(跨端口稳定),
        # 无 serial 时回退 USB bus/address。channel 默认 0（单通道型号）。
        target = adapters[device_index]
        config = self._gsusb.BusConfig(
            bitrate=int(nom_baudrate),
            sample_point=self._SAMPLE_POINT,
            data_bitrate=int(dat_baudrate),
            data_sample_point=self._DATA_SAMPLE_POINT,
            fd=True,
            # 本示例不 echo-tag 发送（send 默认），开启后库在接收路径
            # 直接丢弃回环帧。
            drop_echo=True,
        )

        bus = self._gsusb.CanFdBus()
        try:
            bus.open(target)
            bus.configure(config)
        except Exception as exc:  # noqa: BLE001
            try:
                bus.close()
            except Exception:
                pass
            raise CANFDException(f"gsusb 连接设备异常: {exc}")

        self._bus = bus
        self._is_connected = True
        return True

    def disconnect(self) -> bool:
        self._rx_started = False
        if self._bus is not None:
            try:
                self._bus.close()  # close() 内部会先 stop() 收帧线程
            except Exception:
                pass
            self._bus = None
        self._is_connected = False
        self._adapters = []
        self._receive_callback = None
        return True

    def send(self, id: int, data: bytes, frame_type: int = 0x04,
             extern_flag: int = 0, remote_flag: int = 0) -> bool:
        if not self._is_connected or self._bus is None:
            raise CANFDException("设备未连接")
        if len(data) > self._FRAME_SIZE:
            raise CANFDException("数据长度不能超过64字节")

        # 与其它后端一致：固定发送 64 字节 CAN FD 帧，实际数据在前 N 字节。
        payload = bytes(data) + bytes(self._FRAME_SIZE - len(data))
        frame = self._gsusb.CanFrame(
            id=id,
            data=payload,
            extended=bool(extern_flag),
            fd=True,
            brs=True,
            remote=bool(remote_flag),
            channel=self._bus.channel,
        )
        try:
            self._bus.send(frame)
        except Exception as exc:  # noqa: BLE001
            raise CANFDException(f"发送数据失败: {exc}")
        return True

    def set_receive_callback(self, callback: Optional[Callable[[dict], None]]) -> None:
        self._receive_callback = callback
        if callback is None:
            if self._rx_started and self._bus is not None:
                self._bus.stop()
                self._rx_started = False
            return
        if not self._is_connected or self._bus is None:
            raise CANFDException("设备未连接")
        if not self._rx_started:
            # 库自带工作线程：循环 receive() 期间不 sleep，持续排空适配器缓存
            self._bus.start(self._dispatch)
            self._rx_started = True

    def _dispatch(self, frame) -> None:
        # drop_echo 已在 BusConfig 中开启，回环帧由库的接收路径丢弃，
        # 这里不会收到 echo 帧。
        callback = self._receive_callback
        if callback is None:
            return

        data = bytes(frame.data)
        canfd_msg = {
            "id":          frame.id,
            # 库保证 timestamp 始终有效（取不到硬件时间戳时回退宿主单调时钟）。
            "timestamp":   int(frame.timestamp * 1000),
            "frame_type":  0x04,
            "dlc":         self._gsusb.length_to_dlc(len(data), True),
            "data_len":    len(data),
            "extern_flag": 1 if frame.extended else 0,
            "remote_flag": 1 if frame.remote else 0,
            "bus_status":  0,
            "err_status":  0,
            "te_counter":  0,
            "re_counter":  0,
            "data":        data,
        }
        try:
            callback(canfd_msg)
        except Exception as exc:  # noqa: BLE001
            print(f"CANFD接收回调异常: {exc}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def __del__(self):
        try:
            if self._is_connected:
                self.disconnect()
        except Exception:
            pass


# =============================================================================
# Windows 实现：通过 ctypes 调用 HCanbus.dll
# =============================================================================

if IS_WINDOWS:
    import ctypes.wintypes

    class DevInfo(ctypes.Structure):
        _fields_ = [
            ("HW_Type", ctypes.c_char * 32),
            ("HW_Ser",  ctypes.c_char * 32),
            ("HW_Ver",  ctypes.c_char * 32),
            ("FW_Ver",  ctypes.c_char * 32),
            ("MF_Date", ctypes.c_char * 32),
        ]

    class CanFDConfig(ctypes.Structure):
        _fields_ = [
            ("NomBaud",  ctypes.c_uint),
            ("DatBaud",  ctypes.c_uint),
            ("NomPre",   ctypes.c_ushort),
            ("NomTseg1", ctypes.c_ubyte),
            ("NomTseg2", ctypes.c_ubyte),
            ("NomSJW",   ctypes.c_ubyte),
            ("DatPre",   ctypes.c_ubyte),
            ("DatTseg1", ctypes.c_ubyte),
            ("DatTseg2", ctypes.c_ubyte),
            ("DatSJW",   ctypes.c_ubyte),
            ("Config",   ctypes.c_ubyte),
            ("Model",    ctypes.c_ubyte),
            ("Cantype",  ctypes.c_ubyte),
        ]

    class CanFDMsg(ctypes.Structure):
        _fields_ = [
            ("ID",         ctypes.c_uint),
            ("TimeStamp",  ctypes.c_uint),
            ("FrameType",  ctypes.c_ubyte),
            ("DLC",        ctypes.c_ubyte),
            ("ExternFlag", ctypes.c_ubyte),
            ("RemoteFlag", ctypes.c_ubyte),
            ("BusSatus",   ctypes.c_ubyte),
            ("ErrSatus",   ctypes.c_ubyte),
            ("TECounter",  ctypes.c_ubyte),
            ("RECounter",  ctypes.c_ubyte),
            ("Data",       ctypes.c_ubyte * 64),
        ]

    class _WindowsCANFD:
        """Windows CANFD 实现（HCanbus.dll）"""

        _RECV_BUF_SIZE = 500
        _RECV_TIMEOUT_MS = 50
        _RECV_SLEEP_MS = 0.005

        def __init__(self):
            self._dll = self._load_hcanbus_dll()
            self._dev_index = -1
            self._is_connected = False
            self._receive_callback: Optional[Callable] = None
            self._receive_thread: Optional[threading.Thread] = None
            self._receive_running = False

        @staticmethod
        def _load_hcanbus_dll() -> ctypes.WinDLL:
            search_paths = [
                os.path.join(os.path.dirname(__file__), "lib", "HCanbus.dll"),
                os.path.join(os.path.dirname(__file__), "HCanbus.dll"),
                "HCanbus.dll",
            ]
            for path in search_paths:
                if os.path.exists(path):
                    return ctypes.WinDLL(os.path.abspath(path))
            raise CANFDException("找不到 HCanbus.dll，请将其放在 lib/ 目录下")

        def scan(self) -> int:
            try:
                return int(self._dll.CAN_ScanDevice())
            except Exception as e:
                raise CANFDException(f"扫描设备异常: {e}")

        def connect(self, device_index: int = 0, channel_index: int = 0,
                    nom_baudrate: int = 1000000, dat_baudrate: int = 5000000,
                    nom_sampling: int = 0, dat_sampling: int = 0) -> bool:
            if self._is_connected:
                self.disconnect()

            try:
                ret = self._dll.CAN_OpenDevice(ctypes.c_uint(device_index))
                if ret != 0:
                    raise CANFDException(f"CAN_OpenDevice 失败，返回值: {ret}")

                cfg = CanFDConfig()
                cfg.Model    = 0
                cfg.NomBaud  = nom_baudrate
                cfg.DatBaud  = dat_baudrate
                cfg.Config   = 0x01 | 0x02 | 0x04
                cfg.Cantype  = 1

                if nom_sampling == 0:
                    cfg.NomPre = 2; cfg.NomTseg1 = 31; cfg.NomTseg2 = 8;  cfg.NomSJW = 5
                else:
                    cfg.NomPre = 2; cfg.NomTseg1 = 29; cfg.NomTseg2 = 10; cfg.NomSJW = 6

                if dat_sampling == 0:
                    cfg.DatPre = 1; cfg.DatTseg1 = 11; cfg.DatTseg2 = 4;  cfg.DatSJW = 2

                ret = self._dll.CANFD_Init(ctypes.c_uint(device_index), ctypes.byref(cfg))
                if ret != 0:
                    self._dll.CAN_CloseDevice(ctypes.c_uint(device_index))
                    raise CANFDException(f"CANFD_Init 失败，返回值: {ret}")

                self._dev_index = device_index
                self._is_connected = True

                self._receive_running = True
                self._receive_thread = threading.Thread(
                    target=self._receive_loop, daemon=True)
                self._receive_thread.start()

                return True

            except CANFDException:
                raise
            except Exception as e:
                raise CANFDException(f"连接设备异常: {e}")

        def disconnect(self) -> bool:
            if not self._is_connected:
                return True

            self._receive_running = False
            if self._receive_thread and self._receive_thread.is_alive():
                self._receive_thread.join(timeout=1.0)

            try:
                self._dll.CAN_CloseDevice(ctypes.c_uint(self._dev_index))
            except Exception:
                pass

            self._is_connected = False
            self._dev_index = -1
            self._receive_callback = None
            return True

        def send(self, id: int, data: bytes, frame_type: int = 0x04,
                 extern_flag: int = 0, remote_flag: int = 0) -> bool:
            if not self._is_connected:
                raise CANFDException("设备未连接")
            if len(data) > 64:
                raise CANFDException("数据长度不能超过64字节")

            msg = CanFDMsg()
            msg.ID         = id
            msg.FrameType  = frame_type
            msg.DLC        = _len_to_dlc(64)
            msg.ExternFlag = extern_flag
            msg.RemoteFlag = remote_flag

            ctypes.memset(msg.Data, 0, 64)
            for i, b in enumerate(data[:64]):
                msg.Data[i] = b

            ret = self._dll.CANFD_Transmit(
                ctypes.c_uint(self._dev_index),
                ctypes.byref(msg),
                ctypes.c_uint(1),
                ctypes.c_int(100)
            )
            return ret == 1

        def set_receive_callback(self, callback: Optional[Callable[[dict], None]]) -> None:
            self._receive_callback = callback

        def _receive_loop(self) -> None:
            MsgArray = CanFDMsg * self._RECV_BUF_SIZE
            msgs = MsgArray()
            while self._receive_running:
                if not self._is_connected:
                    time.sleep(0.01)
                    continue

                count = self._dll.CANFD_Receive(
                    ctypes.c_uint(self._dev_index),
                    msgs,
                    ctypes.c_uint(self._RECV_BUF_SIZE),
                    ctypes.c_int(self._RECV_TIMEOUT_MS)
                )

                if count > 0 and self._receive_callback:
                    for i in range(count):
                        m = msgs[i]
                        data_len = DLC2LEN[m.DLC] if m.DLC < len(DLC2LEN) else 64
                        canfd_msg = {
                            "id":          m.ID,
                            "timestamp":   m.TimeStamp,
                            "frame_type":  m.FrameType,
                            "dlc":         m.DLC,
                            "data_len":    data_len,
                            "extern_flag": m.ExternFlag,
                            "remote_flag": m.RemoteFlag,
                            "bus_status":  m.BusSatus,
                            "err_status":  m.ErrSatus,
                            "te_counter":  m.TECounter,
                            "re_counter":  m.RECounter,
                            "data":        bytes(m.Data[:data_len]),
                        }
                        try:
                            self._receive_callback(canfd_msg)
                        except Exception as e:
                            print(f"CANFD接收回调异常: {e}")

                time.sleep(self._RECV_SLEEP_MS)

        @property
        def is_connected(self) -> bool:
            return self._is_connected

        def __del__(self):
            try:
                if self._is_connected:
                    self.disconnect()
            except Exception:
                pass


# =============================================================================
# Linux 实现 A: socketcan（python-can，默认）
# =============================================================================

else:
    class _LinuxSocketcanCANFD:
        """Linux CANFD 实现 — socketcan（python-can）"""

        def __init__(self):
            self._is_connected = False
            self._device_index = 0
            self._interface = ""
            self._nom_baudrate = 1000000
            self._dat_baudrate = 5000000
            self._bus = None
            self._receive_thread: Optional[threading.Thread] = None
            self._receive_stop_event = threading.Event()
            self._receive_callback: Optional[Callable] = None

        def scan(self) -> int:
            try:
                can_interfaces = []
                if os.path.exists("/sys/class/net"):
                    for ifname in os.listdir("/sys/class/net"):
                        if ifname.startswith("can"):
                            can_interfaces.append(ifname)
                return len(can_interfaces)
            except Exception as e:
                raise CANFDException(f"扫描设备异常: {e}")

        def _setup_can_interface(self, ifname: str, nom_baudrate: int,
                                 dat_baudrate: int) -> bool:
            import subprocess
            try:
                subprocess.run(["modprobe", "-r", "gs_usb"], capture_output=True)
                subprocess.run(["modprobe",  "gs_usb"],      capture_output=True)
                subprocess.run(
                    ["bash", "-c",
                     "echo 'a8fa 8598' | sudo tee /sys/bus/usb/drivers/gs_usb/new_id"],
                    capture_output=True)
                subprocess.run(["ip", "link", "set", ifname, "down"], capture_output=True)
                subprocess.run(
                    ["ip", "link", "set", ifname, "type", "can",
                     "bitrate", str(nom_baudrate),
                     "dbitrate", str(dat_baudrate),
                     "fd", "on", "loopback", "off", "listen-only", "off"],
                    capture_output=True)
                subprocess.run(["ip", "link", "set", ifname, "up"], capture_output=True)
                return True
            except Exception as e:
                print(f"设置CAN接口失败: {e}")
                return False

        def connect(self, device_index: int = 0, channel_index: int = 0,
                    nom_baudrate: int = 1000000, dat_baudrate: int = 5000000,
                    nom_sampling: int = 0, dat_sampling: int = 0) -> bool:
            try:
                self._device_index = device_index
                self._nom_baudrate = nom_baudrate
                self._dat_baudrate = dat_baudrate

                can_interfaces = []
                if os.path.exists("/sys/class/net"):
                    for ifname in os.listdir("/sys/class/net"):
                        if ifname.startswith("can"):
                            can_interfaces.append(ifname)

                if device_index < 0 or device_index >= len(can_interfaces):
                    raise CANFDException(f"设备索引无效: {device_index}")

                self._interface = can_interfaces[device_index]

                if not self._setup_can_interface(self._interface, nom_baudrate, dat_baudrate):
                    raise CANFDException("设置CAN接口失败")

                import can
                self._bus = can.Bus(
                    interface='socketcan',
                    channel=self._interface,
                    bitrate=nom_baudrate,
                    fd=True,
                    can_filters=[]
                )

                self._is_connected = True
                return True

            except CANFDException:
                raise
            except Exception as e:
                if self._bus:
                    try:
                        self._bus.shutdown()
                    except Exception:
                        pass
                    self._bus = None
                raise CANFDException(f"连接设备异常: {e}")

        def disconnect(self) -> bool:
            if not self._is_connected:
                return True

            if self._receive_thread and self._receive_thread.is_alive():
                self._receive_stop_event.set()
                self._receive_thread.join(timeout=1.0)

            if self._bus:
                try:
                    self._bus.shutdown()
                except Exception:
                    pass
                self._bus = None

            if self._interface:
                import subprocess
                try:
                    subprocess.run(
                        ["ip", "link", "set", self._interface, "down"],
                        capture_output=True)
                except Exception:
                    pass

            self._is_connected = False
            self._interface = ""
            self._receive_callback = None
            return True

        def send(self, id: int, data: bytes, frame_type: int = 0x04,
                 extern_flag: int = 0, remote_flag: int = 0) -> bool:
            if not self._is_connected:
                raise CANFDException("设备未连接")
            if len(data) > 64:
                raise CANFDException("数据长度不能超过64字节")

            import can
            msg = can.Message(
                arbitration_id=id,
                data=data,
                is_extended_id=bool(extern_flag),
                is_remote_frame=bool(remote_flag),
                is_fd=True,
                bitrate_switch=True
            )
            try:
                self._bus.send(msg)
                return True
            except Exception as e:
                raise CANFDException(f"发送数据失败: {e}")

        def set_receive_callback(self, callback: Optional[Callable[[dict], None]]) -> None:
            self._receive_callback = callback
            if callback and not (self._receive_thread and self._receive_thread.is_alive()):
                if not self._is_connected:
                    raise CANFDException("设备未连接")
                self._receive_stop_event.clear()
                self._receive_thread = threading.Thread(
                    target=self._receive_loop, daemon=True)
                self._receive_thread.start()

        def _receive_loop(self) -> None:
            try:
                while not self._receive_stop_event.is_set():
                    try:
                        msg = self._bus.recv(timeout=0.1)
                        if msg and self._receive_callback:
                            data_len = len(msg.data)
                            dlc = next(
                                (i for i, v in enumerate(DLC2LEN) if v == data_len), 15)
                            canfd_msg = {
                                "id":          msg.arbitration_id,
                                "timestamp":   int(msg.timestamp * 1000),
                                "frame_type":  0x04,
                                "dlc":         dlc,
                                "data_len":    data_len,
                                "extern_flag": 1 if msg.is_extended_id else 0,
                                "remote_flag": 1 if msg.is_remote_frame else 0,
                                "bus_status":  0,
                                "err_status":  0,
                                "te_counter":  0,
                                "re_counter":  0,
                                "data":        bytes(msg.data),
                            }
                            try:
                                self._receive_callback(canfd_msg)
                            except Exception as e:
                                print(f"CANFD接收回调异常: {e}")
                    except Exception:
                        time.sleep(0.01)
                    time.sleep(0.001)
            except Exception as e:
                print(f"CANFD接收线程异常: {e}")
                if self._is_connected:
                    try:
                        self.disconnect()
                    except Exception:
                        pass

        @property
        def is_connected(self) -> bool:
            return self._is_connected

        def __del__(self):
            try:
                if self._is_connected:
                    self.disconnect()
            except Exception:
                pass


    # =========================================================================
    # Linux 实现 B: 厂商 libcanbus.so（通过 USE_LIBCANBUS / driver 参数切换）
    # =========================================================================

    from ctypes import (
        CDLL, POINTER, RTLD_GLOBAL, Structure, byref,
        c_char, c_int, c_ubyte, c_uint, c_uint16, c_uint32, c_ushort, cast, cdll,
    )

    class _CanFD_Config(Structure):
        _fields_ = [
            ("NomBaud", c_uint),
            ("DatBaud", c_uint),
            ("NomPre", c_ushort),
            ("NomTseg1", c_ubyte),
            ("NomTseg2", c_ubyte),
            ("NomSJW", c_ubyte),
            ("DatPre", c_ubyte),
            ("DatTseg1", c_ubyte),
            ("DatTseg2", c_ubyte),
            ("DatSJW", c_ubyte),
            ("Config", c_ubyte),
            ("Model", c_ubyte),
            ("Cantype", c_ubyte),
        ]

    class _CanFD_Msg(Structure):
        _fields_ = [
            ("ID", c_uint),
            ("TimeStamp", c_uint),
            ("FrameType", c_ubyte),
            ("DLC", c_ubyte),
            ("ExternFlag", c_ubyte),
            ("RemoteFlag", c_ubyte),
            ("BusSatus", c_ubyte),
            ("ErrSatus", c_ubyte),
            ("TECounter", c_ubyte),
            ("RECounter", c_ubyte),
            ("Data", c_ubyte * 64),
        ]

    class _LinuxLibCanBusCANFD:
        """Linux CANFD 实现 — 厂商 libcanbus.so"""

        _RECV_BUF_SIZE = 500
        _RECV_TIMEOUT_MS = 50

        def __init__(self):
            self._load_library()
            self._is_connected = False
            self._device_index = 0
            self._channel_index = 0
            self._receive_thread: Optional[threading.Thread] = None
            self._receive_stop_event = threading.Event()
            self._receive_callback: Optional[Callable] = None

        def _load_library(self):
            try:
                CDLL("/usr/local/lib/libusb-1.0.so", RTLD_GLOBAL)
                self._libcan = cdll.LoadLibrary("/usr/local/lib/libcanbus.so")
            except Exception as exc:
                raise CANFDException(f"加载 CANFD 动态库失败: {exc}")
            self._bind_libcanbus_symbols()

        def _bind_libcanbus_symbols(self) -> None:
            """Bind Linux libcanbus.so ABI (matches CANFDMaster.cpp)."""
            lib = self._libcan
            lib.CAN_ScanDevice.argtypes = []
            lib.CAN_ScanDevice.restype = c_int

            lib.CAN_OpenDevice.argtypes = [c_int, c_int]
            lib.CAN_OpenDevice.restype = c_int

            lib.CAN_CloseDevice.argtypes = [c_int, c_int]
            lib.CAN_CloseDevice.restype = c_int

            lib.CANFD_Init.argtypes = [c_int, c_int, POINTER(_CanFD_Config)]
            lib.CANFD_Init.restype = c_int

            lib.CANFD_Transmit.argtypes = [
                c_int, c_int, POINTER(_CanFD_Msg), c_int, c_int]
            lib.CANFD_Transmit.restype = c_int

            lib.CANFD_Receive.argtypes = [
                c_int, c_int, POINTER(_CanFD_Msg), c_int, c_int]
            lib.CANFD_Receive.restype = c_int

        def scan(self) -> int:
            try:
                ret = self._libcan.CAN_ScanDevice()
                if ret < 0:
                    raise CANFDException(f"扫描设备失败，错误码: {ret}")
                return ret
            except Exception as exc:
                raise CANFDException(f"扫描设备异常: {exc}")

        def connect(self, device_index: int = 0, channel_index: int = 0,
                    nom_baudrate: int = 1000000, dat_baudrate: int = 5000000,
                    nom_sampling: int = 0, dat_sampling: int = 0) -> bool:
            try:
                self._device_index = device_index
                self._channel_index = channel_index

                ret = self._libcan.CAN_OpenDevice(device_index, channel_index)
                if ret != STATUS_OK:
                    raise CANFDException(f"打开设备失败，错误码: {ret}")

                can_initconfig = _CanFD_Config()
                can_initconfig.NomBaud = nom_baudrate
                can_initconfig.DatBaud = dat_baudrate
                can_initconfig.Config = 0x01 | 0x02 | 0x04
                can_initconfig.Cantype = 1
                can_initconfig.Model = 0
                can_initconfig.NomPre = 2
                can_initconfig.NomTseg1 = 31
                can_initconfig.NomTseg2 = 8
                can_initconfig.NomSJW = 5
                can_initconfig.DatPre = 1
                can_initconfig.DatTseg1 = 11
                can_initconfig.DatTseg2 = 4
                can_initconfig.DatSJW = 2

                ret = self._libcan.CANFD_Init(
                    device_index, channel_index, byref(can_initconfig))
                if ret != STATUS_OK:
                    self._libcan.CAN_CloseDevice(device_index, channel_index)
                    raise CANFDException(f"初始化 CANFD 失败，错误码: {ret}")

                self._is_connected = True
                return True

            except Exception as exc:
                raise CANFDException(f"连接设备异常: {exc}")

        def disconnect(self) -> bool:
            try:
                if not self._is_connected:
                    return True

                if self._receive_thread and self._receive_thread.is_alive():
                    self._receive_stop_event.set()
                    self._receive_thread.join(timeout=1.0)

                ret = self._libcan.CAN_CloseDevice(
                    self._device_index, self._channel_index)
                if ret != STATUS_OK:
                    raise CANFDException(f"关闭设备失败，错误码: {ret}")

                self._is_connected = False
                self._receive_callback = None
                return True

            except Exception as exc:
                raise CANFDException(f"断开设备异常: {exc}")

        def send(self, id: int, data: bytes, frame_type: int = 0x04,
                 extern_flag: int = 0, remote_flag: int = 0) -> bool:
            try:
                if not self._is_connected:
                    raise CANFDException("设备未连接")
                if len(data) > 64:
                    raise CANFDException("数据长度不能超过64字节")

                send_canmsg = _CanFD_Msg()
                send_canmsg.ID = id
                send_canmsg.FrameType = frame_type
                send_canmsg.DLC = _len_to_dlc(64)
                send_canmsg.ExternFlag = extern_flag
                send_canmsg.RemoteFlag = remote_flag

                for index in range(64):
                    send_canmsg.Data[index] = 0
                for index, value in enumerate(data[:64]):
                    send_canmsg.Data[index] = value

                ret = self._libcan.CANFD_Transmit(
                    self._device_index,
                    self._channel_index,
                    byref(send_canmsg),
                    1,
                    100,
                )
                if ret != 1:
                    raise CANFDException(f"发送数据失败，错误码: {ret}")
                return True

            except Exception as exc:
                raise CANFDException(f"发送数据异常: {exc}")

        def set_receive_callback(self, callback: Optional[Callable[[dict], None]]) -> None:
            self._receive_callback = callback
            if callback and not (self._receive_thread and self._receive_thread.is_alive()):
                if not self._is_connected:
                    raise CANFDException("设备未连接")
                self._receive_stop_event.clear()
                self._receive_thread = threading.Thread(
                    target=self._receive_loop, daemon=True)
                self._receive_thread.start()

        def _receive_loop(self) -> None:
            try:
                msg_array_type = _CanFD_Msg * self._RECV_BUF_SIZE
                receive_canmsg = msg_array_type()

                while not self._receive_stop_event.is_set():
                    ret = self._libcan.CANFD_Receive(
                        self._device_index,
                        self._channel_index,
                        receive_canmsg,
                        self._RECV_BUF_SIZE,
                        self._RECV_TIMEOUT_MS,
                    )

                    if ret > 0 and self._receive_callback:
                        for index in range(ret):
                            msg = receive_canmsg[index]
                            data_len = DLC2LEN[msg.DLC] if msg.DLC < len(DLC2LEN) else 64
                            canfd_msg = {
                                "id":          msg.ID,
                                "timestamp":   msg.TimeStamp,
                                "frame_type":  msg.FrameType,
                                "dlc":         msg.DLC,
                                "data_len":    data_len,
                                "extern_flag": msg.ExternFlag,
                                "remote_flag": msg.RemoteFlag,
                                "bus_status":  msg.BusSatus,
                                "err_status":  msg.ErrSatus,
                                "te_counter":  msg.TECounter,
                                "re_counter":  msg.RECounter,
                                "data":        bytes(msg.Data[:data_len]),
                            }
                            try:
                                self._receive_callback(canfd_msg)
                            except Exception as exc:
                                print(f"CANFD接收回调异常: {exc}")
                    time.sleep(0.001)

            except Exception as exc:
                print(f"CANFD接收线程异常: {exc}")
                if self._is_connected:
                    try:
                        self.disconnect()
                    except Exception:
                        pass

        @property
        def is_connected(self) -> bool:
            return self._is_connected

        def __del__(self):
            try:
                if self._is_connected:
                    self.disconnect()
            except Exception:
                pass


# =============================================================================
# CANFD 工厂：根据平台 / driver 参数返回具体实现
# =============================================================================

class _AutoCANFD:
    """默认后端选择器：优先 gsusb-canfd，不可用或无设备时回退平台原生后端。

    C++ 侧用编译期宏（LHANDPRO_USE_GSUSB_CANFD / LHANDPRO_USE_HCANBUS）二选一，
    Python 侧改为运行期择优——无需重新构建即可兼容非 gs_usb 适配器（例如仅
    提供 HCanbus.dll 的型号）。

    回退判定发生在 scan()：只有当 gsusb 一条适配器都枚举不到（或 pyusb /
    gsusb_canfd 缺失）时，才切换到平台原生后端重扫。
    """

    def __init__(self):
        self._impl = None

    @staticmethod
    def _fallback_factories():
        """平台原生后端候选（按顺序尝试，构造失败即跳过）。"""
        if IS_WINDOWS:
            return (_WindowsCANFD,)
        if USE_LIBCANBUS:
            return (_LinuxLibCanBusCANFD,)
        return (_LinuxSocketcanCANFD,)

    def _build_fallback(self):
        for factory in self._fallback_factories():
            try:
                return factory()
            except Exception:  # noqa: BLE001
                continue
        return None

    def scan(self) -> int:
        if USE_GSUSB_CANFD:
            gsusb = None
            gsusb_error = None
            try:
                gsusb = _GsUsbCANFD()
            except Exception as exc:  # noqa: BLE001  pyusb / gsusb_canfd 缺失
                gsusb = None
                gsusb_error = exc
            if gsusb is not None:
                try:
                    count = gsusb.scan()
                except Exception as exc:  # noqa: BLE001  例如 pyusb 无可用后端
                    count = 0
                    gsusb_error = exc
                if count > 0:
                    self._impl = gsusb
                    return count
                gsusb.disconnect()  # 未选中，及时释放，避免白占 USB 句柄
                gsusb_error = gsusb_error or "未发现 gs_usb 适配器"
                print(f"[CANFD] gsusb 后端未命中({gsusb_error})，回退平台原生后端")
            else:
                # 依赖缺失（pyusb / gsusb_canfd / libusb）时最容易误判为“没有设备”，
                # 这里显式打印原因，避免静默回退到 Linux socketcan 后在 macOS 上得到 0。
                print(f"[CANFD] gsusb 后端不可用: {gsusb_error}；回退平台原生后端")

        self._impl = self._build_fallback()
        if self._impl is None:
            raise CANFDException("没有可用的 CANFD 后端")
        return int(self._impl.scan())

    def _require(self):
        if self._impl is None:
            raise CANFDException("未选择 CANFD 后端，请先调用 scan()")
        return self._impl

    def connect(self, *args, **kwargs) -> bool:
        return self._require().connect(*args, **kwargs)

    def disconnect(self) -> bool:
        if self._impl is None:
            return True
        return self._impl.disconnect()

    def send(self, *args, **kwargs) -> bool:
        return self._require().send(*args, **kwargs)

    def set_receive_callback(self, callback) -> None:
        self._require().set_receive_callback(callback)

    @property
    def is_connected(self) -> bool:
        return bool(self._impl is not None and self._impl.is_connected)

    def __del__(self):
        try:
            if self._impl is not None and self._impl.is_connected:
                self._impl.disconnect()
        except Exception:
            pass


def CANFD(driver: Optional[str] = None):
    """创建 CANFD 实例

    driver=None（默认，推荐）: 优先 gsusb-canfd；若 pyusb / gsusb_canfd 缺失
                              或扫不到 gs_usb 适配器，则自动回退:
                              Windows -> HCanbus.dll
                              Linux   -> USE_LIBCANBUS 决定 libcanbus / socketcan

    driver 显式指定时强制使用该后端，不回退，失败即抛 CANFDException:
                              "gsusb" / "hcanbus" / "socketcan" / "libcanbus"

    现场调试可免改代码切换后端：设置环境变量 LHANDPRO_CANFD_DRIVER
    （取值同 driver，例如 hcanbus），其优先级高于默认的自动择优。
    """
    if driver is None:
        driver = os.environ.get("LHANDPRO_CANFD_DRIVER") or None

    if driver is None:
        if USE_LIBCANBUS:
            return _LinuxLibCanBusCANFD()
        return _AutoCANFD()

    key = str(driver).strip().lower()
    if key in ("gsusb", "gsusb-canfd", "gsusb_canfd"):
        return _GsUsbCANFD()
    if key == "hcanbus":
        if not IS_WINDOWS:
            raise CANFDException("HCanbus 后端仅在 Windows 可用")
        return _WindowsCANFD()
    if key == "socketcan":
        if IS_WINDOWS:
            raise CANFDException("socketcan 后端仅在 Linux 可用")
        return _LinuxSocketcanCANFD()
    if key == "libcanbus":
        if IS_WINDOWS:
            raise CANFDException("libcanbus 后端仅在 Linux 可用")
        return _LinuxLibCanBusCANFD()
    raise CANFDException(
        f"未知的 CANFD driver: {driver}"
        "（可选 gsusb / hcanbus / socketcan / libcanbus）")
