#!/usr/bin/env python3
"""MuJoCo 窗口在 macOS 上的启动适配（被 view_traj.py / view_pico.py 共用）。

macOS 上 ``mujoco.viewer.launch_passive()`` 要求脚本运行在 ``mjpython`` 之下，
否则抛 ``RuntimeError: `launch_passive` requires that the Python script be run
under `mjpython` on macOS``——因为 Cocoa 的事件循环必须占据真正的 macOS 主线程。

本模块在普通解释器里检测到这种情况后，用 ``os.execve`` 把当前命令重新拉起到
同目录的 ``mjpython``，调用方无需感知：

    from gui_boot import ensure_gui_interpreter
    ensure_gui_interpreter()      # 需要窗口时调一次即可

另外处理一个本机特有的坑：managed python 的 venv 里 ``bin/python3`` 是符号链接，
``mjpython`` 用 ``_NSGetExecutablePath()`` 取到的是 venv 路径，dlopen libpython 时
``@loader_path`` 就解析到了 ``venv/bin``（而不是真正的 ``versions/<ver>/lib``），
于是报 ``Library not loaded: @rpath/libpython3.13.dylib``。这里显式把真
libpython 所在目录塞进 ``DYLD_FALLBACK_LIBRARY_PATH`` 解决。

离屏渲染（``--headless --record``）**不需要**本模块，普通 python 即可。
"""

from __future__ import annotations

import os
import pathlib
import sys

_ENV_FLAG = "_R1_SIM_MJPYTHON"


def ensure_gui_interpreter() -> None:
    """确保当前进程能创建 MuJoCo 窗口；必要时重拉到 mjpython。"""
    if sys.platform != "darwin" or os.environ.get(_ENV_FLAG) == "1":
        return

    try:
        import mujoco.viewer
    except Exception:
        return
    if mujoco.viewer._MJPYTHON is not None:  # 已经跑在 mjpython 下
        return

    mjpython = pathlib.Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file():
        raise SystemExit(
            f"[gui] macOS 上开窗口需要 mjpython，但 {mjpython.parent} 下没有。\n"
            f"      可改用离屏导出：--headless --record sim/out/xxx.mp4"
        )

    env = dict(os.environ)
    env[_ENV_FLAG] = "1"  # 防止重拉后再次进入本函数
    libdir = pathlib.Path(sys.base_prefix) / "lib"
    if libdir.is_dir():
        old = env.get("DYLD_FALLBACK_LIBRARY_PATH", "")
        env["DYLD_FALLBACK_LIBRARY_PATH"] = f"{libdir}:{old}" if old else str(libdir)

    print(f"[gui] 普通解释器无法开窗，切换到 mjpython：{mjpython.name}", flush=True)
    os.execve(str(mjpython), [str(mjpython), os.path.abspath(sys.argv[0]), *sys.argv[1:]], env)
