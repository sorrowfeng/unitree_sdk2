#!/usr/bin/env python3
"""接 PICO 的 GUI 可视化：UDP 报文 -> 全链路 -> MuJoCo 实时显示。

这是"接了 PICO"那条路径的可视化。数据通路与实机一致：

    PICO 头显（或 pico_sim_sender 模拟器）
        --UDP-JSON v3--> 本机 --pico_pipeline_test--> q --> MuJoCo 窗口

链路本身（解析 / 安全判据 / 坐标对齐 / IK）跑在 `build/pico_pipeline_test` 里，
和 `r1_dual_arm_loco.cpp --pico` **同源同参**，只是去掉 DDS。所以窗口里看到的
动作就是实机会执行的动作；区别仅在于这里把关节角喂给 MuJoCo 而不是真实电机。

与"没接 PICO"的那条路径（`view_traj.py`，轨迹由 MuJoCo 自己生成）互补。

用法：
    # 终端 1：开窗等待（本脚本会自动切换到 mjpython）
    python sim/view_pico.py --variant a5 --port 9999

    # 终端 2：真 PICO 直接发到本机 9999；没有头显时用模拟发送器
    python example/r1/high_level/scripts/pico_sim_sender.py \
        --host 127.0.0.1 --port 9999 --duration 30

窗口里：
    绿球 = PICO 给出的目标腕位置（经对齐换算到机器人腰部系）
    红球 = IK 解算后实际到达的腕位置（MuJoCo 独立 FK）
    左上角文字 = 当前处置（teleop / hold / return_zero / emergency_stop）与报文序号

`set_texts` 的官方注释说第一个参数是 mjtFontScale，那是错的：`mjr_overlay` 只认
mjtFont（NORMAL=0 / SHADOW=1 / BIG=2），传 mjFONTSCALE_* 会被静默降成最小号字。
默认 NORMAL（原始字号，一行约 444x22 px）；`--font big` 是 2 倍大字（约 856x38）。

HUD 文案必须是纯 ASCII：MuJoCo 内置点阵字体不含中文字形，中文会渲染成实心白块
（小号）或直接消失（大号）—— 原来的"误差 / 包 / 失联"就是这么变成方块的。

两个球水平贴合说明遥操作跟得准；红球偏移量就是误差。若把 `safe_to_execute`
置 false、或触发急停闩锁，窗口里手臂会立刻停住（进入 hold / emergency_stop），
这是可视化确认安全判据最直观的方式。
"""

from __future__ import annotations

import argparse
import pathlib
import socket
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gui_boot import ensure_gui_interpreter  # noqa: E402
from r1_model import R1Sim  # noqa: E402
from view_traj import add_markers  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
PIPELINE = HERE / "build" / "pico_pipeline_test"
LIFT = 0.10


def _floats(line: str) -> list[float]:
    """挑出字符串里所有能转成 float 的 token（用来解析 [pose] 行）。"""
    out: list[float] = []
    for tok in line.replace("=", " ").split():
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


class PipelineClient:
    """把报文逐行喂给 `pico_pipeline_test`，读回处置与关节角。

    输出契约（见 sim/pico_pipeline_test.cpp）：
        ok=<0|1> seq=<n> safe=<0|1> disp=<...> [q=<2n 个关节角>]
    `--dump-pose` 还会把对齐后的目标腕位置打到 stderr，这里用后台线程收着，
    用来画绿球。
    """

    def __init__(self, variant: str, raw: bool):
        if not PIPELINE.is_file():
            raise SystemExit(f"[pico-view] 找不到 {PIPELINE}\n           请先执行 ./sim/build_probe.sh")
        cmd = [str(PIPELINE), "--variant", variant, "--dump-pose"]
        if raw:
            cmd.append("--raw")
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,  # 行缓冲
        )
        self._lock = threading.Lock()
        self._target: tuple[np.ndarray, np.ndarray] | None = None
        threading.Thread(target=self._drain_stderr, daemon=True).start()

    def _drain_stderr(self) -> None:
        stderr = self.proc.stderr
        if stderr is None:
            return
        for line in stderr:  # `[pose] L=x y z R=x y z`
            if not line.startswith("[pose]"):
                continue
            nums = _floats(line)
            if len(nums) >= 6:
                with self._lock:
                    self._target = (np.array(nums[0:3]), np.array(nums[3:6]))

    def target(self) -> tuple[np.ndarray, np.ndarray] | None:
        with self._lock:
            return self._target

    def query(self, packet: str) -> tuple[str, int, np.ndarray | None]:
        """喂一包报文，返回 (处置, 序号, 关节角或 None)。"""
        assert self.proc.stdin is not None and self.proc.stdout is not None
        self.proc.stdin.write(packet + "\n")
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise SystemExit("[pico-view] 全链路进程已退出（检查报文是否合法）")

        toks = line.split()
        disp = "unknown"
        seq = 0
        q: np.ndarray | None = None
        for i, tok in enumerate(toks):
            if tok.startswith("disp="):
                disp = tok[5:]
            elif tok.startswith("seq="):
                try:
                    seq = int(tok[4:])
                except ValueError:
                    pass
            elif tok.startswith("q="):
                vals = [t for t in ([tok[2:]] + toks[i + 1:]) if t]
                try:
                    q = np.array([float(v) for v in vals])
                except ValueError:
                    q = None
                break
        return disp, seq, q

    def close(self) -> None:
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


def main() -> None:
    ap = argparse.ArgumentParser(description="接 PICO 的实时可视化（UDP -> 全链路 -> MuJoCo）")
    ap.add_argument("--variant", default="a5", choices=["a5", "a7"])
    ap.add_argument("--port", type=int, default=9999, help="监听 UDP 端口（对应 --pico-port）")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--raw", action="store_true", help="跳过 WMA 平滑（看求解器裸响应）")
    ap.add_argument("--fps", type=float, default=60.0, help="窗口刷新率")
    ap.add_argument("--stale", type=float, default=1.0, help="超过该秒数无报文即视为失联")
    ap.add_argument("--font", default="normal", choices=["normal", "big"],
                    help="HUD 字号：normal 为原始字号（默认），big 是 2 倍大字")
    args = ap.parse_args()

    ensure_gui_interpreter()

    sim = R1Sim(args.variant)
    n = sim.n
    client = PipelineClient(args.variant, args.raw)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.004)

    print(f"[pico-view] 监听 udp://{args.bind}:{args.port}  variant={args.variant}  raw={args.raw}")
    print(f"[pico-view] 报文源：真 PICO，或 python "
          f"example/r1/high_level/scripts/pico_sim_sender.py "
          f"--host 127.0.0.1 --port {args.port} --duration 30")
    print("[pico-view] 窗口已打开，关窗退出")

    import mujoco
    import mujoco.viewer

    # 注意：mjr_overlay 的第一个参数是 mjtFont（NORMAL / SHADOW / BIG），
    # 不是 mjtFontScale（50/100/.../300）。二者是不同枚举，传 mjFONTSCALE_150
    # 属于非法值，会被静默当成最小号字 —— 在高 DPI 屏上 framebuffer 是 2x，
    # 小号字只有 ~7pt，这正是窗口里文字看不清的原因。
    # Simulate 的 set_texts 文档写的是 mjtFontScale，属上游注释错误。
    font = {"big": mujoco.mjtFont.mjFONT_BIG,
            "normal": mujoco.mjtFont.mjFONT_NORMAL}[args.font]
    print(f"[pico-view] HUD 字体 = {args.font} ({int(font)})"
          f"{'  （偏小可加 --font big）' if args.font == 'normal' else ''}")

    q = np.zeros(2 * n)
    disp, seq, err = "waiting", 0, float("nan")
    target: tuple[np.ndarray, np.ndarray] | None = None
    last_rx = 0.0
    n_pkt = 0

    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                t0 = time.time()

                # 只取最新一包：GUI 不需要回放积压，丢旧包比排队延迟更重要
                pkt = None
                while True:
                    try:
                        raw, _ = sock.recvfrom(65535)
                    except (socket.timeout, OSError):
                        break
                    pkt = raw.decode("utf-8", "replace").strip()

                if pkt:
                    last_rx = time.time()
                    disp, seq, q_new = client.query(pkt)
                    if q_new is not None and q_new.size == 2 * n:
                        q = q_new
                    target = client.target()
                    n_pkt += 1

                sim.set_q(q)
                pl, _, pr, _ = sim.ee_dual()
                if target is not None:
                    err = float(np.linalg.norm(np.asarray(pl) - target[0])) * 1000.0

                viewer.user_scn.ngeom = 0
                if target is not None:
                    add_markers(
                        viewer.user_scn,
                        (target[0], None, target[1], None),
                        (pl, pr),
                        LIFT,
                    )
                # MuJoCo 的 overlay 用的是内置点阵字体，只覆盖 ASCII —— 中文会渲染成
                # 空白方块或直接消失，所以 HUD 文案必须是英文。
                if n_pkt == 0:
                    extra = "  [waiting for packets]"
                elif time.time() - last_rx > args.stale:
                    extra = "  [LINK LOST]"
                else:
                    extra = ""
                viewer.set_texts((
                    font,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    f"disp={disp}  seq={seq}{extra}",
                    f"err {'--' if np.isnan(err) else f'{err:.2f}'} mm   pkts {n_pkt}",
                ))
                viewer.sync()

                dt = 1.0 / args.fps - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        sock.close()
        print(f"[pico-view] 退出，共处理 {n_pkt} 包")


if __name__ == "__main__":
    main()
