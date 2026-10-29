#!/usr/bin/env bash
# 灵巧手桥的启动封装。
#
# 要调左右映射 / 节点号 / 端口，就改下面这几个默认值（或在命令行用环境变量覆盖）：
#     LEFT_BUS=1 RIGHT_BUS=0 ./bridge.sh
# 命令行里多出来的参数会原样透传给 hand_bridge.py（例：`./bridge.sh --verbose`）。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LEFT_BUS=${LEFT_BUS:-0}     # 0 = 左手（2026-10-29 实测确认：device_index 0 驱动左手）
RIGHT_BUS=${RIGHT_BUS:-1}   # 1 = 右手（2026-10-29 实测确认：device_index 1 驱动右手）
NODE=${NODE:-1}             # 两条总线上的节点号（两只手都是 1）
PORT=${PORT:-9998}          # 遥操程序发位置过来的 UDP 端口

exec "$HERE/hand_canfd.sh" hand_bridge.py \
  --left-bus "$LEFT_BUS" \
  --right-bus "$RIGHT_BUS" \
  --node "$NODE" \
  --port "$PORT" \
  "$@"
