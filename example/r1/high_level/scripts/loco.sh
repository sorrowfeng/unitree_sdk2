#!/usr/bin/env bash
# R1 官方运控（LocoClient）测试脚本
#
# 目标运行环境：R1-EDU 开发计算单元（算力背包，aarch64，192.168.123.164）。
# 用途：二次开发前验证机器人正常运控——查询状态、阻尼、零力矩、站立、
#       启动主运控、速度移动、停走、速度档位、任意 FSM 切换。
#
# 底层封装 unitree::robot::r1::LocoClient，调用本仓库已编译的
# build/bin/r1_loco_client（对应 example/r1/high_level/r1_loco_client_example.cpp）。
# 若该二进制不存在，脚本会自动调用同目录 build.sh 编译它。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
LOCO_BIN="${ROOT}/build/bin/r1_loco_client"

IFACE=""
ASSUME_YES=0
DRY_RUN=0

die() { echo "[loco] $*" >&2; exit 1; }

usage() {
  cat <<EOF
R1 官方运控测试脚本（建议部署/运行于算力背包）

用法: $(basename "$0") [选项] <命令> [参数]

命令:
  status                                查询 FSM ID 与 FSM Mode
  watch [period_s]                      每 period_s 秒查询 FSM ID（默认 1，Ctrl+C 退出）
  damp                                  进入阻尼模式 (fsm 1)
  stand                                 站立 (fsm 4)
  start                                 启动主运控 (fsm 811)
  stop                                  停止移动 (velocity 置零)
  zero-torque                           零力矩 (fsm 0)  [危险：机器人会瘫软]
  set-fsm <id>                          设置任意 FSM ID
  move <vx> <vy> <vyaw> [duration_s]    速度移动（duration 默认 1s，给大值即持续）
  speed <mode>                          设置速度档位 (SetSpeedMode)
  menu                                  交互式菜单

选项:
  --iface <name>   指定 DDS 网卡（默认自动探测 192.168.123.x 网卡）
  -y, --yes        跳过危险操作确认（自动化用）
  --dry-run        只打印将执行的原生命令，不真正下发
  -h, --help       显示本帮助

示例:
  $(basename "$0") status
  $(basename "$0") --iface eth0 stand
  $(basename "$0") -y move 0.2 0 0 5
  $(basename "$0") menu
EOF
}

# ---------------- 参数解析 ----------------
CMD=""
ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --iface)    IFACE="${2:-}"; shift 2 ;;
    -y|--yes)   ASSUME_YES=1; shift ;;
    --dry-run)  DRY_RUN=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    -*)         die "未知选项: $1（用 -h 查看帮助）" ;;
    *)          CMD="$1"; shift; ARGS=("$@"); break ;;
  esac
done
[ -n "$CMD" ] || { usage; exit 1; }

# ---------------- 工具函数 ----------------
detect_iface() {
  command -v ip >/dev/null 2>&1 || return 1
  ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}'
}

ensure_iface() {
  [ -n "$IFACE" ] || IFACE="$(detect_iface || true)"
  [ -n "$IFACE" ] || die "未探测到 192.168.123.x 网卡，请用 --iface <name> 指定（如 --iface eth0）"
}

ensure_bin() {
  if [ ! -x "$LOCO_BIN" ]; then
    echo "[loco] 未找到 $LOCO_BIN，正在编译 r1_loco_client ..."
    "${SCRIPT_DIR}/build.sh" r1_loco_client
  fi
  [ -x "$LOCO_BIN" ] || die "编译后仍未找到 $LOCO_BIN"
}

confirm() {
  [ "$ASSUME_YES" = 1 ] && return 0
  printf '%s\n' "$1"
  local ans
  read -r -p "确认执行？输入 YES 继续: " ans
  [ "$ans" = "YES" ] || { echo "[loco] 已取消"; exit 1; }
}

run_loco() {
  local -a cmd=( "$LOCO_BIN" "--network_interface=${IFACE}" "$@" )
  if [ "$DRY_RUN" = 1 ]; then
    printf '[dry-run]'; printf ' %q' "${cmd[@]}"; printf '\n'
    return 0
  fi
  echo "[loco] 执行: ${cmd[*]}"
  "${cmd[@]}"
}

fsm_name() {
  case "$1" in
    0)   echo "ZeroTorque(零力矩)" ;;
    1)   echo "Damp(阻尼)" ;;
    4)   echo "StandUp(站立)" ;;
    811) echo "Start(主运控)" ;;
    *)   echo "Unknown" ;;
  esac
}

# ---------------- 子命令 ----------------
cmd_status() {
  ensure_iface; ensure_bin
  echo "[loco] 网卡: ${IFACE}"
  run_loco --get_fsm_id
  run_loco --get_fsm_mode
}

cmd_watch() {
  ensure_iface; ensure_bin
  local period="${1:-1}"
  echo "[loco] 网卡: ${IFACE}，每 ${period}s 查询 FSM ID（Ctrl+C 退出）"
  while true; do
    printf '%s  ' "$(date +%H:%M:%S)"
    run_loco --get_fsm_id || true
    sleep "$period"
  done
}

cmd_set_fsm() {
  local id="${1:-}"
  [ -n "$id" ] || die "用法: set-fsm <id>"
  ensure_iface; ensure_bin
  if [ "$id" = "0" ]; then
    confirm "⚠️  即将切换到【零力矩 ZeroTorque】：机器人会失去支撑瘫软，请确保有吊具或人员保护！"
  else
    [ "$ASSUME_YES" = 1 ] || echo "[loco] 切换到 FSM ${id} ($(fsm_name "$id"))"
  fi
  run_loco "--set_fsm_id=${id}"
}

cmd_move() {
  local vx="${1:-}" vy="${2:-}" vyaw="${3:-}" dur="${4:-1}"
  { [ -n "$vx" ] && [ -n "$vy" ] && [ -n "$vyaw" ]; } || die "用法: move <vx> <vy> <vyaw> [duration_s]"
  ensure_iface; ensure_bin
  confirm "⚠️  即将以 vx=${vx} vy=${vy} vyaw=${vyaw} 移动 ${dur}s，请确保机器人周围安全！"
  run_loco "--set_velocity=${vx} ${vy} ${vyaw} ${dur}"
}

cmd_menu() {
  ensure_iface; ensure_bin
  echo "[loco] 网卡: ${IFACE}，二进制: ${LOCO_BIN}"
  while true; do
    cat <<EOF

===================== R1 官方运控 =====================
  1) 查询状态 (get_fsm_id / get_fsm_mode)
  2) 阻尼 Damp                     [fsm 1]
  3) 站立 StandUp                  [fsm 4]
  4) 启动主运控 Start              [fsm 811]
  5) 移动 (输入 vx vy vyaw duration)
  6) 停止移动
  7) 设置速度档位 (SetSpeedMode)
  8) 零力矩 ZeroTorque  [fsm 0]   ⚠️ 危险
  9) 设置任意 FSM ID
  0) 退出
=======================================================
EOF
    local c
    read -r -p "选择: " c || exit 0
    case "$c" in
      1) cmd_status ;;
      2) run_loco --damp ;;
      3) run_loco --stand_up ;;
      4) run_loco --start ;;
      5) local a b d e; read -r -p "vx vy vyaw duration: " a b d e; run_loco "--set_velocity=${a:-0} ${b:-0} ${d:-0} ${e:-1}" ;;
      6) run_loco --stop_move ;;
      7) local m; read -r -p "speed mode: " m; run_loco "--set_speed_mode=${m}" ;;
      8) cmd_set_fsm 0 ;;
      9) local f; read -r -p "fsm id: " f; cmd_set_fsm "$f" ;;
      0) exit 0 ;;
      *) echo "无效选择" ;;
    esac
  done
}

# ---------------- 分发 ----------------
case "$CMD" in
  status)      cmd_status ;;
  watch)       cmd_watch "${ARGS[0]:-1}" ;;
  damp)        ensure_iface; ensure_bin; run_loco --damp ;;
  stand)       ensure_iface; ensure_bin; run_loco --stand_up ;;
  start)       ensure_iface; ensure_bin; run_loco --start ;;
  stop)        ensure_iface; ensure_bin; run_loco --stop_move ;;
  zero-torque|zero_torque) cmd_set_fsm 0 ;;
  set-fsm|set_fsm)         cmd_set_fsm "${ARGS[0]:-}" ;;
  move)        cmd_move "${ARGS[@]}" ;;
  speed)       ensure_iface; ensure_bin; [ -n "${ARGS[0]:-}" ] || die "用法: speed <mode>"; run_loco "--set_speed_mode=${ARGS[0]}" ;;
  menu)        cmd_menu ;;
  *)           die "未知命令: $CMD（用 -h 查看帮助）" ;;
esac
