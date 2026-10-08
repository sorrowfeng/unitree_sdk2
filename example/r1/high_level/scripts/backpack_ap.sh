#!/usr/bin/env bash
# =============================================================================
# backpack_ap.sh —— 在 R1-EDU 算力背包（PC2，Jetson Orin / Ubuntu 20.04 / NM）
# 上把背包本身变成一个 WiFi 热点（AP 模式），让 PICO 头显**不需要任何路由器**
# 就能直连背包，把 UDP 报文送进主程序。
#
# ---------------------------------------------------------------------------
# 背景：2026-09-23 在真机上只读核查得到的实测结论（别再凭猜）
# ---------------------------------------------------------------------------
#   1) 背包**没有任何无线网卡**：
#        ls /sys/class/net             -> lo dummy0 docker0 eth10
#        ls /sys/class/net/*/wireless  -> 不存在
#        lsusb                         -> 只有两个 root hub（没插任何 USB 设备）
#      ⇒ **今天这条路上跑不起来**，必须先插一块 USB WiFi 网卡。
#
#   2) 好消息：内核**编了完整无线子系统 + 有内核头文件**
#        /lib/modules/$(uname -r)/kernel/net/wireless              存在
#        /lib/modules/$(uname -r)/kernel/net/mac80211              存在
#        /lib/modules/$(uname -r)/kernel/drivers/net/wireless      存在
#        /lib/modules/$(uname -r)/build                            存在
#      ⇒ 插对芯片的 USB 网卡**即插即用**，不用重编内核。
#
#   3) **最优解：AIC8800 的厂商驱动已经装好了**
#        /lib/modules/$(uname -r)/kernel/drivers/net/wireless/aic8800/
#            aic8800_fdrv.ko + aic_load_fw.ko   （vermagic 与运行内核逐字一致）
#        /lib/firmware/aic8800DC/               （固件，只有 8800dc 这一套）
#        /etc/udev/rules.d/aic.rules            （U 盘模式自动 eject）
#        NM 策略 10-globally-managed-devices.conf 里 wifi 类型不被排除 ⇒ 插上有 wlan0 就会被接管
#      且驱动内**明确含 AP 模式能力**：
#        NL80211_FEATURE_AP_MODE_CHAN_WIDTH_CHANGE / NL80211_IFTYPE_P2P_GO /
#        `parm=ap_uapsd_on:Enable UAPSD in AP mode` / 日志串 "%s: ap mode, mac=%pM"
#      ⇒ **买 AIC8800 的卡，零编译插上即用，且能开热点**（第一推荐）。
#      ⚠️ 买之前对 VID:PID 白名单（两组都要命中，同一张卡会换 ID 二次枚举）：
#        网卡态 a69c:88dc 88dd 88de 8d41 8d81 8801 / 368b:88e5 88de 8d99 8d91
#        固件态 a69c:8800 8d40 8d80        / 368b:8d90 8d91 8d92 8d99
#
#   4) 退而求其次：内核自带的芯片 + 固件也齐：
#        htc_9271.fw ✓（AR9271 / ath9k_htc）  rt2870.bin ✓（RT5370 / rt2800usb）
#        rtlwifi/rtl8192cufw.bin ✓（RTL8192CU / rtl8192cu）
#      ⇒ 这三类卡"插上就有"，且都支持 AP。
#      ⛔ 别买 MT7601U、RTL8188EU/8192EU（驱动能认但**没有 AP 模式**）、
#         MT7921AU（需 ≥5.18）、RTL8852AU（需 ≥6.4）。
#
#   5) **不需要装 hostapd**：NetworkManager 1.22.10 自带 AP 模式
#        (802-11-wireless.mode ap + ipv4.method shared)
#      底层 wpa_supplicant 2.9，DHCP 由 NM 内置 dnsmasq 提供
#      （/usr/sbin/dnsmasq 已装）。libnl-3/route/genl-3-200、libssl1.1 也都在。
#      ⚠️ 背包 apt 是坏的（版本错位，见 R1_BACKPACK_ARCHITECTURE.md §4.7.10），
#         能少装一个包就少装一个 —— 这正是本脚本走 NM、不走 hostapd 的原因。
#
#   6) ⚠️ 硬约束：热点网段**不要**用 192.168.123.0/24。
#      NM 的 `ipv4.method shared` 默认给 AP 网卡分 10.42.0.1/24，与机器人内网
#      （eth10 = 192.168.123.164，DDS 走这张卡）天然不冲突 —— 保持默认即可。
#
# ---------------------------------------------------------------------------
# 用法（在背包上跑）
# ---------------------------------------------------------------------------
#   ./backpack_ap.sh check                   # 只读体检（默认子命令，先跑这个）
#   ./backpack_ap.sh chips                   # 按本机内核实况告诉你该买哪块卡
#   ./backpack_ap.sh start [SSID] [密码]      # 开热点（不传密码则交互式提示）
#   ./backpack_ap.sh status                  # 热点状态 + PICO 端该填什么
#   ./backpack_ap.sh stop                    # 关热点（保留配置）
#   ./backpack_ap.sh rm                      # 删除热点配置
#   ./backpack_ap.sh -h                      # 帮助
#
# 从 Mac 上直接跑（不用先 scp）：
#   ssh -t unitree@192.168.123.164 'bash -s check' \
#       < example/r1/high_level/scripts/backpack_ap.sh
#   （-t 必须加：`start` 交互式输密码需要 tty）
#
# 凭据规则：密码只在内存里流转，写盘时直接落到 NM 的 profile（root 600），
#          **不经过命令行参数**（否则会出现在 `ps` 里）、不打印、不进日志。
# =============================================================================
set -uo pipefail

CON_NAME="${AP_CON_NAME:-R1Hotspot}"
DEF_SSID="${AP_SSID:-R1Backpack}"
PORT="${PICO_PORT:-9999}"
SUDO_PASS="${BACKPACK_PASS:-123}"

say()  { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
warn() { printf '\033[33m! %s\033[0m\n' "$*"; }
ok()   { printf '\033[32m✓ %s\033[0m\n' "$*"; }
bad()  { printf '\033[31m✗ %s\033[0m\n' "$*"; }
dim()  { printf '\033[2m  %s\033[0m\n' "$*"; }

has()    { command -v "$1" >/dev/null 2>&1; }
sudo_()  { echo "$SUDO_PASS" | sudo -S "$@" 2>/dev/null; }
# 需要看见报错时用它（滤掉 sudo 自己的提示行）
sudo_v() { echo "$SUDO_PASS" | sudo -S "$@" 2>&1 | grep -v '\[sudo\]' ; }

# ---------------------------------------------------------------------------
# 探测工具
# ---------------------------------------------------------------------------
wlan_iface() {
  local d
  for d in /sys/class/net/*/wireless; do
    [ -e "$d" ] && basename "$(dirname "$d")" && return 0
  done
  return 1
}

# 输出 yes / no / unknown（这块网卡能不能当 AP）
wlan_ap_cap() {
  local ifc="$1"
  has iw || { echo unknown; return 0; }
  if echo "$SUDO_PASS" | sudo -S iw list 2>/dev/null \
     | awk '/Supported interface modes/{f=1;next} /^[[:space:]]*[A-Z]/{if(f)f=0} f' \
     | grep -qw 'AP'; then
    echo yes
  else
    echo no
  fi
}

ap_ip() {
  local ifc; ifc="$(wlan_iface 2>/dev/null || true)"
  [ -n "$ifc" ] || return 0
  nmcli -g IP4.ADDRESS dev show "$ifc" 2>/dev/null | head -1 | cut -d/ -f1
}

# ---------------------------------------------------------------------------
cmd_chips() {
  local kver; kver="$(uname -r)"
  local drv="/lib/modules/$kver/kernel/drivers/net/wireless"

  say "当前内核：$kver $(uname -m)"
  if [ -d "$drv" ]; then
    ok "内核有无线驱动目录（${drv}）"
  else
    bad "内核**没有** drivers/net/wireless ⇒ 插任何 USB 网卡都不会被识别"
    dim "这条路是死的 ⇒ 改用旅行路由器 / Mac 热点中继（文档方案 C/D）"
    return 0
  fi

  say "USB WiFi 网卡选型（按本机实况自动判定，不是照抄网上攻略）"
  printf '  %-13s %-4s %-4s %-4s %s\n' 模块 驱动 固件 AP 结论
  printf '  %-13s %-4s %-4s %-4s %s\n' ------------- ---- ---- -- --------------------------

  local mod chip fw ap note m_ok f_ok verdict
  while IFS='|' read -r mod chip fw ap note; do
    [ -z "${mod:-}" ] && continue

    if has modinfo && modinfo "$mod" >/dev/null 2>&1; then m_ok="有"; else m_ok="-"; fi
    if [ -z "$fw" ]; then f_ok="-"
    elif [ -e "/lib/firmware/$fw" ]; then f_ok="有"
    else f_ok="缺"; fi

    if   [ "$m_ok" = "-" ];    then verdict="别买：本内核无此驱动"
    elif [ "$f_ok" = "缺" ];   then verdict="驱动在但固件缺（可另补固件）"
    elif [ "$ap"   = "no" ];   then verdict="别买：驱动能认但**不支持 AP 模式**"
    elif [ "$ap"   = "yes" ];  then verdict="✅ 可选：插上即可用"
    else                            verdict="不确定，保守起见别买"
    fi

    printf '  %-13s %-4s %-4s %-4s %s\n' "$mod" "$m_ok" "$f_ok" "$ap" "$verdict"
    dim "$chip"
    dim "$note"
  done <<'ROWS'
aic8800_fdrv|AIC8800（AX1800 免驱款主流）|aic8800DC|yes|✅✅ 第一推荐：厂商 out-of-tree 驱动**本机已装**（vermagic 与运行内核逐字一致），固件/udev/NM 托管策略全就位，零编译插上即用；驱动内明确含 AP 模式能力标志。⚠️ 必须买 **DC 变体**：固件只有 aic8800DC 一套，驱动把目录名写死；D80 变体可能缺固件
ath9k_htc|AR9271（TP-Link TL-WN722N **v1**、各类 AR9271 模块）|htc_9271.fw|yes|✅ 次选：2.4GHz，AP 模式最成熟。注意 TL-WN722N 的 v2/v3 换了芯片，不是 AR9271！
rt2800usb|RT5370 / RT3070（大量廉价 nano 卡）|rt2870.bin|yes|✅ 次选：2.4GHz，便宜好买，AP 模式可用
rtl8192cu|RTL8188CUS / RTL8192CU|rtlwifi/rtl8192cufw.bin|yes|⚠️ 有 AP 模式，但同一颗芯片可能被无 AP 的 rtl8xxxu 抢走绑定 ⇒ 有运气成分，不如上面三块
carl9170|AR9170（很老的卡）|ar9170-1.fw|yes|固件本机缺失
zd1211rw|ZD1211 / ZD1211B|zd1211_u-phy1.fw|yes|固件本机缺失
mt7601u|MT7601U|mt7601u.bin|no|❌ 固件在，但主线驱动没有实现 AP 模式，只能当客户端
rtl8xxxu|RTL8188EU / RTL8192EU|rtlwifi/rtl8188eufw.bin|no|❌ 主线只有 STA 模式
mt7921u|MT7921AU / MT7922（WiFi6 新卡）||no|❌ 需 kernel >= 5.18，本机 5.10 没有
rtw89_8852au|RTL8852AU / RTL8852BU（WiFi6 新卡）||no|❌ 需 kernel >= 6.4，本机 5.10 没有
ROWS

  say "具体买哪个型号（去店里下单用）"
  dim "① AIC8800 **DC** 变体 USB 网卡            ¥15-35    确定性：中（赌变体）"
  dim "   常见名「AX286 免驱 WiFi6」迷你款：USB2.0 / 2.4GHz 286Mbps / 内置天线"
  dim "   工业模块带蓝牙的 AIC8800US9-80I 约 ¥25-30"
  dim "② AR9271 + 可拆 SMA 天线                  ¥40-250   确定性：高（最稳，推荐）"
  dim "   Alfa AWUS036NHA / TP-Link TL-WN722N **v1**（v2/v3 换了芯片）/ 第三方 AR9271+SMA 模块"
  dim "③ RT5370 / RT3070                         ¥15-30    确定性：高（但要防假货）"
  dim "   Alfa AWUS036NH / Panda PAU05 / PAU06"
  echo
  say "三条硬条件（缺一条就白买）"
  dim "1) 必须是 **USB 口**版本 —— AIC8800 还有 SDIO / M.2 / 贴片模组"
  dim "2) AIC8800 必须 **DC 变体**，下单前直接问卖家要 USB VID:PID（商品页从不写变体）"
  dim "3) 优先 **可拆 SMA 外置天线** —— 金属背包壳 + 内置天线是常见翻车点"
  echo
  say "廉价「免驱 nano 网卡」的假货替身（外观一样，芯片却无 AP）"
  dim "  148f:5370 = RT5370 ✅    148f:7601 = MT7601U ❌    0bda:8179|8178 = RTL8xxxEU ❌"
  echo
  say "买回来之后怎么验（商品页不可信，以这条为准）"
  dim "  lsusb                          # 看 VID:PID，多出来的就是它"
  dim "  iw list | sed -n '/Supported interface modes/,/^\$/p' | grep -c AP   # ≥1 才算真支持 AP"
  dim "  $0 check                     # 应出现 wlanX，且 AP 能力判成 yes"
  dim "  sudo dmesg | tail -20           # 看固件有没有加载失败"
}

# ---------------------------------------------------------------------------
cmd_check() {
  say "0. 系统"
  # shellcheck disable=SC1091
  . /etc/os-release 2>/dev/null || true
  echo "  ${PRETTY_NAME:-unknown}   kernel $(uname -r)   $(uname -m)"
  echo "  hostname: $(hostname)"

  say "1. 有没有无线网卡"
  local ifc; ifc="$(wlan_iface || true)"
  if [ -n "$ifc" ]; then
    ok "发现无线网卡：$ifc"
    ip -brief link show "$ifc" 2>/dev/null | sed 's/^/  /'
  else
    bad "没有任何无线网卡（/sys/class/net/*/wireless 为空）"
    dim "背包（PC2）是**无 WiFi** 的 Jetson Orin 配置。R1 规格里的 WiFi 6 在主机（PC1）"
    dim "那一侧，而 PC1 禁 SSH、我们不碰它。所以想让**背包自己**开热点，必须插 USB 网卡。"
    cmd_chips
    say "在买到网卡之前，先走别的路"
    dim "见 example/r1/high_level/R1_TELEOP_STARTUP.md 第 5 章："
    dim "  方案 A  机器人配套的路由器/AP（网关 192.168.123.1 现在 ARP FAILED = 不在线）"
    dim "  方案 C  Mac 做热点 + UDP 中继（零硬件，今天就能跑）"
    dim "  方案 D  头显走 USB-C 转网口，插机器人内部交换机（有线最稳）"
    return 0
  fi

  say "2. 这块网卡支持 AP 吗"
  case "$(wlan_ap_cap "$ifc")" in
    yes) ok "$ifc 支持 AP 模式 —— 可以开热点" ;;
    no)  bad "$ifc **不支持 AP 模式**，只能当客户端 ⇒ 换卡（见 '$0 chips'）" ;;
    *)   warn "没有 iw 命令，无法判定；直接试 '$0 start' 也行" ;;
  esac
  dim "完整能力表： sudo iw list | sed -n '/Supported interface modes/,/^$/p'"

  say "3. 内核无线子系统 / 头文件"
  local kver; kver="$(uname -r)" d
  for d in kernel/net/wireless kernel/net/mac80211 kernel/drivers/net/wireless; do
    printf '  %-48s ' "/lib/modules/$kver/$d"
    [ -d "/lib/modules/$kver/$d" ] && echo "存在" || echo "缺失"
  done
  printf '  %-48s ' "/lib/modules/$kver/build"
  [ -d "/lib/modules/$kver/build" ] && echo "存在（可编 out-of-tree 驱动）" || echo "缺失"

  say "4. 无线固件"
  local f
  for f in htc_9271.fw rt2870.bin rtlwifi/rtl8192cufw.bin mt7601u.bin; do
    printf '  %-32s ' "$f"
    [ -e "/lib/firmware/$f" ] && echo "有" || echo "-"
  done

  say "5. 开热点要用的组件（本脚本走 NM，**不用** hostapd）"
  printf '  %-16s ' "nmcli";          has nmcli && nmcli -v 2>&1 || echo "缺"
  printf '  %-16s ' "wpa_supplicant"; has wpa_supplicant && wpa_supplicant -v 2>&1 | head -1 || echo "缺"
  printf '  %-16s ' "dnsmasq";        has dnsmasq && echo "$(command -v dnsmasq)" || echo "缺（NM shared 模式需要）"
  printf '  %-16s ' "iptables";       has iptables && echo "$(command -v iptables)" || echo "缺"
  printf '  %-16s ' "hostapd";        has hostapd && echo "$(command -v hostapd)" || echo "-（本脚本不需要）"

  say "6. RF 开关"
  if has rfkill; then rfkill list 2>/dev/null | sed 's/^/  /'; else warn "无 rfkill 命令"; fi

  say "7. 已存的热点配置"
  if has nmcli; then
    nmcli -f NAME,TYPE,DEVICE,STATE con show 2>/dev/null | sed 's/^/  /'
    if [ "$(nmcli -g 802-11-wireless.mode con show "$CON_NAME" 2>/dev/null)" = "ap" ]; then
      ok "已有热点配置 '$CON_NAME'（mode=ap）"
    fi
  fi

  say "结论"
  if [ -z "$ifc" ]; then
    warn "没有无线网卡 ⇒ 开不了热点。① 插受支持的 USB 网卡（见 '$0 chips'）"
    warn "                            ② 先走方案 C（Mac 热点 + UDP 中继）"
  elif [ "$(wlan_ap_cap "$ifc")" = "no" ]; then
    warn "$ifc 不支持 AP ⇒ 换卡"
  else
    ok "条件齐备 ⇒ '$0 start [SSID] [密码]'"
  fi
}

# ---------------------------------------------------------------------------
cmd_start() {
  local ssid="${1:-$DEF_SSID}" pass="${2:-}"

  has nmcli || { bad "没有 nmcli；本脚本走 NM，无法继续（备选见脚本末尾）"; exit 1; }
  local ifc; ifc="$(wlan_iface || true)"
  [ -n "$ifc" ] || { bad "没有无线网卡 —— 先跑 '$0 check'，它会告诉你该买哪块卡"; exit 1; }

  case "$(wlan_ap_cap "$ifc")" in
    no) bad "$ifc 不支持 AP 模式，换卡（见 '$0 chips'）"; exit 1 ;;
  esac

  if [ -z "$pass" ]; then
    printf '给热点 "%s" 设一个密码（至少 8 位，输入不回显）: ' "$ssid"
    read -r -s pass; printf '\n'
  fi
  if [ "${#pass}" -lt 8 ]; then
    bad "WPA-PSK 密码至少 8 位，当前 ${#pass} 位"; exit 1
  fi

  say "准备网卡 $ifc"
  has rfkill && sudo_ rfkill unblock wifi
  sudo_ ip link set "$ifc" up
  ok "已 up"

  say "写 NM profile（密码直接落到 root 600 的文件，**不经命令行**）"
  local tmp; tmp="$(mktemp)"
  chmod 600 "$tmp"
  cat > "$tmp" <<EOF
[connection]
id=$CON_NAME
uuid=$(sudo_v cat /proc/sys/kernel/random/uuid | tr -d '\r\n')
type=wifi
interface-name=$ifc
autoconnect=false

[wifi]
mode=ap
ssid=$ssid
band=bg
channel=6
hidden=false

[wifi-security]
key-mgmt=wpa-psk
psk=$pass

[ipv4]
method=shared

[ipv6]
method=ignore
EOF
  unset pass

  sudo_ install -m 600 -o root -g root "$tmp" \
        "/etc/NetworkManager/system-connections/$CON_NAME.nmconnection" \
    || { bad "写 $CON_NAME.nmconnection 失败"; rm -f "$tmp"; exit 1; }
  rm -f "$tmp"
  ok "已写入 /etc/NetworkManager/system-connections/$CON_NAME.nmconnection"

  sudo_ nmcli con reload
  sudo_ nmcli con delete "$CON_NAME" >/dev/null 2>&1 || true   # 清掉同名内存态旧对象
  sudo_ nmcli con reload

  say "up —— 这一步才真正开始发信标"
  if ! sudo_v nmcli con up "$CON_NAME"; then
    bad "nmcli con up 失败。排查："
    dim "1) 驱动真的支持 AP 吗： sudo iw list | sed -n '/Supported interface modes/,/^\$/p'"
    dim "2) 详细报错：          sudo nmcli con up $CON_NAME    （上面就是原始输出）"
    dim "3) journalctl -u NetworkManager -n 50"
    dim "4) 退路：hostapd 老路（脚本末尾注释），但 apt 坏的，要本机下 deb 再 dpkg -i"
    exit 1
  fi

  sleep 2
  cmd_status
  local ip4; ip4="$(ap_ip)"
  say "PICO 头显侧"
  ok "设置 → WLAN，连 SSID：$ssid"
  [ -n "$ip4" ] && ok "PICOHandLink 目标地址填：$ip4:$PORT"
  dim "填 $ip4 最稳；填 192.168.123.164 也能到 —— 收端绑的是 INADDR_ANY，"
  dim "两个地址都落在同一个 socket 上，代码一行都不用改。"
}

cmd_status() {
  local ifc; ifc="$(wlan_iface || true)"
  [ -n "$ifc" ] || { bad "没有无线网卡"; return 0; }
  has nmcli && nmcli -f DEVICE,TYPE,STATE,CONNECTION dev status 2>/dev/null | sed 's/^/  /'
  local ip4; ip4="$(ap_ip)"
  if [ -n "$ip4" ]; then
    ok "热点地址：$ip4"
    echo "  无线 SSH：  ssh unitree@$ip4"
    echo "  PICO 目标： $ip4:$PORT"
    echo "  收工关掉：  $0 stop"
  else
    warn "$ifc 还没有 IPv4（热点没起来 / 没 up）"
  fi
}

cmd_stop() {
  has nmcli || exit 1
  sudo_ nmcli con down "$CON_NAME" && ok "已关热点（配置保留，'$0 start' 可再开）" \
                                   || warn "关失败或本来就没开"
}

cmd_rm() {
  has nmcli || exit 1
  sudo_ nmcli con delete "$CON_NAME" && ok "已删除配置 $CON_NAME" || warn "删除失败或本来就没有"
}

case "${1:-check}" in
  check|"")  cmd_check ;;
  chips)     cmd_chips ;;
  start)     shift; cmd_start "$@" ;;
  stop)      cmd_stop ;;
  rm)        cmd_rm ;;
  status)    cmd_status ;;
  -h|--help) sed -n '1,62p' "$0" ;;
  *) echo "未知子命令: $1（check|chips|start|status|stop|rm）" >&2; exit 1 ;;
esac

# =============================================================================
# 【备选】不用 NetworkManager、直接 hostapd 的老路（本脚本不自动执行）
#
#   前提：背包 apt 是坏的（版本错位），**不能** `apt install hostapd`。
#   走法：在能上网的机器上取 deb，再传进去单包装：
#     # 在 Mac 上（Ubuntu 20.04 / arm64）
#     curl -LO http://ports.ubuntu.com/ubuntu-ports/pool/main/w/wpa/hostapd_2.9-1ubuntu4_arm64.deb
#     scp hostapd_*.deb unitree@192.168.123.164:~/
#     # 在背包上
#     sudo dpkg -i ~/hostapd_*.deb        # 报依赖缺就逐个补同样的 arm64 deb
#
#   /etc/hostapd/hostapd.conf:
#     interface=wlan0
#     driver=nl80211
#     ssid=R1Backpack
#     hw_mode=g
#     channel=6
#     wpa=2
#     wpa_passphrase=<至少8位>
#     wpa_key_mgmt=WPA-PSK
#     rsn_pairwise=CCMP
#   起服务：
#     sudo systemctl unmask hostapd
#     sudo systemctl enable --now hostapd
#   DHCP + NAT 另配（dnsmasq / iptables 都已装）：
#     sudo sysctl -w net.ipv4.ip_forward=1
#     sudo iptables -t nat -A POSTROUTING -o eth10 -j MASQUERADE
#
#   ⚠️ 仍然不要让热点网段撞 192.168.123.0/24。
# =============================================================================
