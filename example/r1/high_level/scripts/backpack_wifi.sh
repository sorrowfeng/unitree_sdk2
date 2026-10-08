#!/usr/bin/env bash
# 在 R1-EDU 算力背包（PC2，Jetson Orin，Ubuntu 20.04，NetworkManager）上「体检 / 扫描 / 连接 WiFi」。
#
# 目的：让背包接入头显所在 WiFi，从而 ① 无线 SSH ② PICO 报文可直接到达背包。
# 前提认知（重要，别搞反）：
#   - 背包接 WiFi **不会**改变它与机器人内部网段 192.168.123.x 的连通 —— 那是接在
#     机器人交换机上的有线网卡（eth0/enP8p1s0 之类），仍然要在主程序第 1 个参数里传。
#   - PICO 的 UDP 收端绑的是 0.0.0.0:9999（`r1_pico_udp.h` 里 INADDR_ANY），
#     所以从 WiFi 网卡进来的包照样收得到，**不用改代码**，只需把 PICO 端目标地址
#     改成背包的 WiFi IP。
#   - 反过来：加了 WiFi 会多一条默认路由，但 192.168.123.x 是「直连网段路由」，
#     优先级天然高于默认路由，DDS 不会跑偏。
#
# ⚠️ 本脚本是给**背包（Linux）**跑的，不是在 macOS 上跑。
#
# 用法（在背包上）:
#   ./backpack_wifi.sh check                # 只读体检：硬件/驱动/rfkill/NM 托管状态（默认）
#   ./backpack_wifi.sh scan                 # 列出可见 SSID
#   ./backpack_wifi.sh connect <SSID>       # 连接：nmcli --ask 交互式输入 WiFi 密码（不落盘）
#   WIFI_PASS=*** ./backpack_wifi.sh connect <SSID>   # 或用环境变量传密码
#   ./backpack_wifi.sh status               # 当前连接 + WiFi IP + 该怎么 ssh
#   ./backpack_wifi.sh usbprobe             # 【无内置无线网卡时用】查内核有没有编无线子系统、
#                                           # 外接 USB 网卡（AX1800 等）有没有现成驱动/固件
#
# 从 macOS 上直接跑（不用 scp，ssh 密码交互式输入）:
#   ssh -t unitree@192.168.123.164 'bash -s' < example/r1/high_level/scripts/backpack_wifi.sh
#   ssh -t unitree@192.168.123.164 'bash -s connect "你的SSID"' < example/r1/high_level/scripts/backpack_wifi.sh
#   （-t 必须加：nmcli --ask 要 tty 才能提示输密码）
#
# 注意：连接 WiFi 不会断开你当前这条有线 SSH，可以放心操作。
set -uo pipefail

PORT="${PICO_PORT:-9999}"

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
warn() { printf '\033[33m! %s\033[0m\n' "$*"; }
ok()   { printf '\033[32m✓ %s\033[0m\n' "$*"; }
bad()  { printf '\033[31m✗ %s\033[0m\n' "$*"; }

need_sudo() { sudo -n true 2>/dev/null || true; }

# 找出第一个无线网卡名（没有则空）
wlan_iface() {
  local d
  for d in /sys/class/net/*/wireless; do
    [ -e "$d" ] && basename "$(dirname "$d")" && return 0
  done
  return 1
}

has_nm() { command -v nmcli >/dev/null 2>&1; }

print_ip_hint() {
  local ifc="$1" ip4
  ip4="$(ip -4 -o addr show "$ifc" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)"
  if [ -n "$ip4" ]; then
    say "下一步"
    ok "背包 WiFi 地址：$ip4"
    echo "  无线 SSH：      ssh unitree@$ip4"
    echo "  PICO 端目标：   改成 $ip4:$PORT"
    echo "  防火墙放行：    sudo ufw allow from ${ip4%.*}.0/24 to any port 22 proto tcp"
    echo "                  sudo ufw allow ${PORT}/udp"
    echo "  自动重连：      sudo nmcli con mod \"<连接名>\" connection.autoconnect yes"
    echo "  ⚠️ 主程序第 1 个网卡参数仍传 192.168.123.x 那张有线网卡，不要传 WiFi 的 wlan0。"
  else
    warn "$ifc 还没拿到 IPv4 地址（DHCP 未成功？）——用 '$0 status' 复查"
  fi
}

cmd_check() {
  say "0. 系统"
  # shellcheck disable=SC1091
  . /etc/os-release 2>/dev/null || true
  echo "  ${PRETTY_NAME:-unknown}   kernel $(uname -r)   $(uname -m)"
  echo "  hostname: $(hostname)"

  say "1. NetworkManager"
  if has_nm; then
    ok "nmcli: $(nmcli -v 2>&1)"
    echo "  NetworkManager.service: $(systemctl is-active NetworkManager 2>/dev/null || echo unknown)"
  else
    bad "没有 nmcli —— 该系统不是 NetworkManager 管理网络（可能用 netplan/wpa_supplicant 裸配）"
    echo "  备选路径见脚本末尾「无 NetworkManager 时」注释块。"
  fi

  say "2. 无线设备"
  local ifc; ifc="$(wlan_iface || true)"
  if [ -n "$ifc" ]; then
    ok "发现无线网卡：$ifc"
  else
    bad "在 /sys/class/net/*/wireless 下没找到无线网卡"
  fi
  command -v iw >/dev/null 2>&1 && iw dev 2>/dev/null || warn "没装 iw（sudo apt install iw 可看更详细信息）"
  has_nm && nmcli -f DEVICE,TYPE,STATE,CONNECTION dev status 2>/dev/null

  say "3. 硬件 / 驱动"
  lspci -nn 2>/dev/null | grep -iE 'network|wireless|802\.11' || echo "  lspci 无匹配（Jetson 上 WiFi 常走 M.2/UART，lspci 可能看不到）"
  lsusb 2>/dev/null | grep -iE 'wireless|802\.11|realtek|mediatek|intel|broadcom' || echo "  lsusb 无匹配"
  echo "  --- 内核日志里的无线相关行（末尾 5 行）---"
  (dmesg 2>/dev/null || sudo dmesg 2>/dev/null) | grep -iE 'wlan|ath[0-9]|iwl|rtl|brcm|mt76' | tail -5 || echo "  无"

  say "4. 射频开关（rfkill / radio）"
  rfkill list 2>/dev/null || warn "无 rfkill 命令"
  has_nm && nmcli radio 2>/dev/null

  say "5. 托管状态"
  if [ -n "$ifc" ] && has_nm; then
    nmcli -f GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,GENERAL.CONNECTION dev show "$ifc" 2>/dev/null
    local st; st="$(nmcli -g GENERAL.STATE dev show "$ifc" 2>/dev/null)"
    case "$st" in
      *unmanaged*) bad "$ifc 被标记为 unmanaged —— NM 管不到它。检查 /etc/NetworkManager/NetworkManager.conf
      与 /etc/netplan/*.yaml 是否把 $ifc 排除在外（常见于出厂镜像）。改前先备份，改完 sudo netplan apply / sudo systemctl restart NetworkManager" ;;
      *disconnected*|*"not connected"*) warn "$ifc 已托管但未连接 —— 直接跑 '$0 connect <SSID>'" ;;
      *connected*) ok "$ifc 已连接：$(nmcli -g GENERAL.CONNECTION dev show "$ifc" 2>/dev/null)" ;;
    esac
  fi

  say "6. 结论"
  if [ -n "$ifc" ]; then
    if has_nm; then
      ok "这台背包**有**无线网卡且由 NetworkManager 管理 —— 可以直接 '$0 connect <SSID>'"
    else
      warn "有无线网卡，但没 NM —— 走 wpa_supplicant 手工配（见脚本末尾注释）"
    fi
  else
    bad "没探到无线网卡。Jetson Orin 工业版有**无 WiFi 模块**的配置。
      往下自动跑「外接 USB 网卡可行性」探测（usbprobe）—— 它会告诉你加网卡有没有戏：
        ① 若能上外接网卡 → 选**主线内核自带驱动**的芯片；
        ② 若内核根本没编无线子系统 → 加网卡也是死的，直接改拓扑；
        ③ 笔记本做 UDP 中继 / 跳板：ssh -J <笔记本> unitree@192.168.123.164。"
    cmd_usbprobe
  fi
}

cmd_scan() {
  has_nm || { bad "没有 nmcli，无法扫描"; exit 1; }
  sudo nmcli dev wifi rescan >/dev/null 2>&1 || true
  echo "扫描中（3 秒）..."
  sleep 3
  nmcli -f SSID,SIGNAL,CHAN,SECURITY dev wifi list 2>/dev/null | sort -u
}

cmd_connect() {
  local ssid="${1:-}"
  [ -n "$ssid" ] || { echo "用法: $0 connect <SSID>" >&2; exit 1; }
  has_nm || { bad "没有 nmcli，请看脚本末尾 wpa_supplicant 备选"; exit 1; }
  local ifc; ifc="$(wlan_iface || true)"
  [ -n "$ifc" ] || { bad "没探到无线网卡，连接无从谈起（先跑 '$0 check'）"; exit 1; }

  command -v rfkill >/dev/null 2>&1 && sudo rfkill unblock wifi 2>/dev/null || true
  sudo nmcli radio wifi on 2>/dev/null || true

  local rc=0
  if [ -n "${WIFI_PASS:-}" ]; then
    sudo nmcli dev wifi connect "$ssid" password "$WIFI_PASS" || rc=$?
  else
    # --ask 会在 tty 上交互式提示 WiFi 密码，密码不经过参数/历史/环境变量
    sudo nmcli --ask dev wifi connect "$ssid" || rc=$?
  fi

  if [ $rc -ne 0 ]; then
    bad "连接失败（rc=$rc）。排查："
    echo "  - 5GHz/信道：Jetson 部分模块只支持 2.4GHz 或受地区监管域限制，试试 2.4G 热点"
    echo "  - 密码错 / 隐藏 SSID：隐藏网络要 'sudo nmcli con add type wifi ... ssid <SSID> hidden yes'"
    echo "  - WPA3/企业认证：NM 可能不支持，改用 WPA2 个人版热点试"
    echo "  - 看详细报错：'sudo nmcli dev wifi connect <SSID>' 的完整输出，或 journalctl -u NetworkManager -n 50"
    print_ip_hint "$ifc"
    exit $rc
  fi

  ok "已连接 $ssid"
  sudo nmcli con mod "$ssid" connection.autoconnect yes 2>/dev/null || true
  print_ip_hint "$ifc"
}

cmd_status() {
  local ifc; ifc="$(wlan_iface || true)"
  if [ -z "$ifc" ]; then bad "没有无线网卡"; return 0; fi
  has_nm && nmcli -f DEVICE,TYPE,STATE,CONNECTION dev status 2>/dev/null
  ip -4 -o addr show "$ifc" 2>/dev/null || true
  print_ip_hint "$ifc"
}

# 没有内置无线网卡时，判断「外接 USB WiFi 网卡」有没有戏。
# 结论取决于两件事：① 内核有没有编无线子系统 ② 那块网卡的芯片驱动在不在这个内核里。
cmd_usbprobe() {
  local kver; kver="$(uname -r)"
  local wl="/lib/modules/$kver/kernel/net/wireless"
  local mac="/lib/modules/$kver/kernel/net/mac80211"
  local drv="/lib/modules/$kver/kernel/drivers/net/wireless"

  say "A. 内核 / 头文件"
  echo "  kernel:  $kver"
  echo "  arch:    $(uname -m)"
  if [ -d "/lib/modules/$kver/build" ]; then
    ok "/lib/modules/$kver/build 存在 → 可以编 out-of-tree 驱动"
  else
    warn "/lib/modules/$kver/build 不存在 → **没有内核头文件**，编 out-of-tree 驱动会卡在这里
      （Jetson 需另下 L4T 内核源码/public_sources，或装 nvidia-l4t-kernel-headers，成本不小）"
  fi

  say "B. 无线协议栈（决定 USB 网卡有没有可能被识别）"
  if [ -d "$wl" ] || [ -d "$mac" ]; then
    ok "内核编了无线子系统"
    [ -d "$wl" ]  && { echo "  net/wireless:";  ls "$wl"  2>/dev/null | sed 's/^/    /'; }
    [ -d "$mac" ] && { echo "  net/mac80211:"; ls "$mac" 2>/dev/null | sed 's/^/    /'; }
  else
    bad "$wl 与 $mac 都不存在 → 这个内核**根本没编无线子系统（cfg80211/mac80211）**。
      插任何 USB WiFi 网卡都不会被识别，modinfo 也查不到任何 wifi 驱动。
      唯一出路是换内核/重编内核 —— 代价极高，**请直接改用旅行路由器方案**。"
  fi

  say "C. 内核里已编的无线厂商驱动"
  if [ -d "$drv" ]; then
    (cd "$drv" && find . -name '*.ko*' 2>/dev/null | sed 's/^\.\///' | sort | head -40)
  else
    warn "$drv 不存在（与 B 的结论一致）"
  fi

  say "D. 候选驱动的 modinfo（有输出=这个内核认识它）"
  if command -v modinfo >/dev/null 2>&1; then
    local m
    for m in cfg80211 mac80211 \
             mt7921u mt7921e mt76 mt76_usb mt7601u \
             rtw89_8852au rtw89_8852be rtw89_8852ce rtl8852au rtw_8852au \
             rtl8xxxu rtw88_8821cu rtw88_8822bu r8188eu \
             rt2800usb ath9k_htc carl9170 brcmfmac; do
      if modinfo "$m" >/dev/null 2>&1; then
        ok "$m"
      else
        echo "  -  $m"
      fi
    done
    echo "  （对照：mt7921u 自 kernel 5.18 进主线；rtw89 的 USB 支持（8852AU/BU）自 6.4。
       本机若为 JetPack 5.x=kernel 5.10，两者都不会有。）"
  else
    warn "没有 modinfo（sudo apt install kmod）"
  fi

  say "E. 无线固件"
  if [ -d /lib/firmware ]; then
    local fw; fw="$(ls /lib/firmware 2>/dev/null | grep -iE 'mt79|mt76|mediatek|rtw89|rtlwifi|rtl8' | head -20)"
    [ -n "$fw" ] && echo "$fw" | sed 's/^/  /' || echo "  没看到相关固件（驱动认到设备但缺固件，同样起不来）"
  fi

  say "F. 当前挂着的 USB 设备（插上网卡后重跑这一节，看有没有新面孔）"
  lsusb 2>/dev/null || warn "没有 lsusb（sudo apt install usbutils）"

  say "结论"
  if [ -d "$wl" ] || [ -d "$mac" ]; then
    echo "  内核有无线子系统 → 外接网卡**有可能**：插入后先看 'lsusb' 拿到 VID:PID，
      再按芯片对驱动（MT7921AU→mt7921u 需 ≥5.18；RTL8852AU/BU→rtw89 USB 需 ≥6.4；
      RTL8188CU/8192CU→rtl8xxxu；MT7601U→mt7601u；RT5370→rt2800usb；AR9271→ath9k_htc）。
      后四者在本内核若 modinfo 有输出，才是真正即插即用。"
  else
    echo "  **本机加 USB WiFi 网卡这条路是死的**（内核没编无线子系统），别再买网卡。
      改走：机器人内网交换机插旅行路由器（NAT），或笔记本做 UDP 中继 + ssh -J。"
  fi
}

case "${1:-check}" in
  check|"")  cmd_check ;;
  scan)      cmd_scan ;;
  connect)   shift; cmd_connect "$@" ;;
  status)    cmd_status ;;
  usbprobe)  cmd_usbprobe ;;
  -h|--help) sed -n '2,34p' "$0" ;;
  *) echo "未知子命令: $1（check|scan|connect|status|usbprobe）" >&2; exit 1 ;;
esac

# ===========================================================================
# 【备选】没有 NetworkManager 时的手工路径（本脚本不自动执行，按需复制）
#
#   # 1) 确认网卡与 rfkill
#   iw dev; rfkill list; sudo rfkill unblock wifi
#
#   # 2) 生成配置（避免明文密码直接写在命令行）
#   sudo sh -c 'wpa_passphrase "SSID" "密码" > /etc/wpa_supplicant/wpa_supplicant-wlan0.conf'
#   sudo chmod 600 /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
#
#   # 3) 起 wpa_supplicant + 取地址
#   sudo wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant-wlan0.conf
#   sudo dhclient wlan0
#   ip -4 addr show wlan0
#
#   # 4) 开机自动（systemd）
#   sudo systemctl enable --now wpa_supplicant@wlan0
#
#   # 5) netplan 场景（Ubuntu Server 风格），/etc/netplan/99-wifi.yaml:
#   #   network:
#   #     version: 2
#   #     wifis:
#   #       wlan0:
#   #         access-points:
#   #           "SSID": { password: "密码" }
#   #         dhcp4: true
#   #   → sudo chmod 600 /etc/netplan/99-wifi.yaml && sudo netplan apply
# ===========================================================================
