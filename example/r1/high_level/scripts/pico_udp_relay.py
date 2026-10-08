#!/usr/bin/env python3
# ============================================================================
# pico_udp_relay.py —— 单向 UDP 中继：PICO 头显 -> Mac -> 算力背包
#
# 为什么需要它（方案 C2，2026-10-08 落地）：
#   背包**没有可用的无线网卡**（内置无 WiFi；外接的 AR9271 一旦关联就触发内核
#   挂死 + 看门狗重启，实测 4 次，已弃用）。而 Mac 同时有两条链路：
#       en5 = 192.168.123.200  —— 有线，接在机器人内网交换机上，到背包 0.4 ms
#       en0 = 172.16.23.x      —— 无线，PICO 也能连上同一张 WiFi
#   于是让 PICO 把报文发给 Mac 的**无线**地址，Mac 再原样转发给背包的**有线**地址。
#
#   遥操数据是**单向**的（PICO -> 背包；接收端绑 INADDR_ANY:9999 且从不回包），
#   所以一条用户态中继就够：
#       ✅ 不需要 root     ✅ 不需要 NAT / ip_forward     ✅ 不需要 iptables
#       ⚠️ Mac 必须在场且不能休眠（caffeinate -i 可防睡）
#
# 用法（在 Mac 上）:
#   python3 pico_udp_relay.py                                  # 默认转发到 192.168.123.164:9999
#   python3 pico_udp_relay.py --target-host 192.168.123.164 --target-port 9999
#   python3 pico_udp_relay.py --stats-every 30                 # 每 30 包打一次计数
#   python3 pico_udp_relay.py --dump                           # 额外打印每个包的大小/来源
#
# 在 PICO 端要填的地址 = 本脚本启动时打印的那一行「PICO 端目标」。
# 自检判据（文档 §5.5 的"客户端隔离"陷阱）:
#   启动后只要看到计数在涨，就说明 PICO -> Mac 这一段通了；
#   计数**一直不涨** ⇒ 多半是办公 WiFi 开了**客户端隔离**（PICO 与 Mac 互相看不见），
#   此时改用方案 C1（Mac 开热点）或方案 D（头显走有线）。
# ============================================================================
import argparse
import socket
import sys
import time

DEFAULT_TARGET_HOST = "192.168.123.164"   # 算力背包（PC2）的 eth10
DEFAULT_TARGET_PORT = 9999                # r1_pico_udp.h 里 INADDR_ANY:9999


def local_ipv4_addrs():
    """列出本机所有非回环 IPv4（用 UDP connect 技巧探测，不发任何包）。"""
    out = []
    for probe in ("192.168.123.164", "172.16.23.1", "8.8.8.8"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((probe, 9))
            ip = s.getsockname()[0]
            if ip and not ip.startswith("127.") and ip not in out:
                out.append(ip)
        except OSError:
            pass
        finally:
            s.close()
    return out


def main():
    ap = argparse.ArgumentParser(
        description="单向 UDP 中继：PICO 头显 -> Mac -> 算力背包",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--listen-host", default="0.0.0.0",
                    help="监听地址；0.0.0.0 = 有线/无线哪张网卡进来的包都收")
    ap.add_argument("--listen-port", type=int, default=9999, help="监听端口")
    ap.add_argument("--target-host", default=DEFAULT_TARGET_HOST, help="背包地址")
    ap.add_argument("--target-port", type=int, default=DEFAULT_TARGET_PORT, help="背包端口")
    ap.add_argument("--stats-every", type=int, default=30, help="每多少个包打印一次计数")
    ap.add_argument("--dump", action="store_true", help="打印每个包的大小与来源")
    args = ap.parse_args()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind((args.listen_host, args.listen_port))
    except OSError as e:
        print(f"[relay] ✗ 无法绑定 {args.listen_host}:{args.listen_port} —— {e}", file=sys.stderr)
        print("        （端口被占？用 lsof -nP -iUDP:%d 查）" % args.listen_port, file=sys.stderr)
        return 1

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target_host, args.target_port)

    print("[relay] 单向 UDP 中继已启动")
    print(f"[relay]   监听      : {args.listen_host}:{args.listen_port}")
    print(f"[relay]   转发到    : {target[0]}:{target[1]}  （算力背包）")
    ips = local_ipv4_addrs()
    if ips:
        print("[relay] PICO 端目标请填下列之一（挑与头显同网段的那个）:")
        for ip in ips:
            print(f"[relay]     {ip}:{args.listen_port}")
    else:
        print("[relay] ⚠ 没探测到本机 IPv4，请手动 ipconfig getifaddr en0")
    print("[relay] 判据: 计数在涨 = PICO 到 Mac 这一段通了；一直为 0 = 多半是 WiFi 客户端隔离")
    print("[relay] Ctrl+C 退出。\n")

    n = 0
    bytes_total = 0
    first_src = None
    t0 = time.time()
    try:
        while True:
            data, src = rx.recvfrom(65535)
            n += 1
            bytes_total += len(data)
            if first_src is None:
                first_src = src
                print(f"[relay] 首包 {len(data)} B 来自 {src[0]}:{src[1]}", flush=True)
            if args.dump:
                print(f"[relay] #{n} {len(data)} B <- {src[0]}:{src[1]}", flush=True)
            try:
                tx.sendto(data, target)
            except OSError as e:
                # 只报错不退出：背包偶尔不可达不应让中继死掉
                print(f"[relay] ⚠ 转发失败（#{n} -> {target[0]}:{target[1]}）: {e}", flush=True)
                continue
            if args.stats_every > 0 and n % args.stats_every == 0:
                dt = time.time() - t0
                rate = n / dt if dt > 0 else 0.0
                print(f"[relay] {n} 包  {bytes_total} B  {rate:.1f} 包/秒  "
                      f"最近来源 {src[0]}:{src[1]} -> {target[0]}:{target[1]}", flush=True)
    except KeyboardInterrupt:
        dt = time.time() - t0
        print(f"\n[relay] 已停止。共转发 {n} 包 / {bytes_total} B，历时 {dt:.1f} s"
              + (f"，首包来自 {first_src[0]}:{first_src[1]}" if first_src else "（一个包都没收到）"))
    finally:
        rx.close()
        tx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
