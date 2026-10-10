#!/usr/bin/env bash
# 灵巧手 CANFD 工具的统一入口 —— 自动处理平台差异，不用记环境变量。
#
# 用法（Mac 与背包通用）:
#   hand_canfd.sh check_comms.py --nodes 1 2              # 只读通讯验证
#   hand_canfd.sh check_comms.py --scan --scan-max 8      # 扫 node，找总线上的手
#   hand_canfd.sh test_hand_canfd.py --nodes 1 2 --position 1000
#
# 依赖目录（需含 canfd_lib.py / gsusb_canfd/ / usb/）按顺序找：
#   $R1_HAND_DIR  →  ~/r1_hand  →  脚本自身所在目录
#
# 平台差异（这就是本脚本存在的理由）：
#   macOS: pyusb 要靠 DYLD_LIBRARY_PATH 才能找到 Homebrew 的 libusb-1.0.dylib，
#          否则报 "no USB backend available"。
#   Linux: 系统自带 libusb，什么都不用设。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 找依赖目录 ----
DIR=""
# 顺序：显式指定 → **脚本自己所在目录**（本目录自包含，仓库里直接可用）→ ~/r1_hand（背包部署位置）
for cand in "${R1_HAND_DIR:-}" "$SCRIPT_DIR" "$HOME/r1_hand"; do
  [ -n "$cand" ] || continue
  if [ -f "$cand/canfd_lib.py" ] && [ -d "$cand/gsusb_canfd" ]; then
    DIR="$cand"; break
  fi
done
if [ -z "$DIR" ]; then
  echo "找不到依赖目录（需要 canfd_lib.py + gsusb_canfd/ + usb/）。" >&2
  echo "请设置 R1_HAND_DIR，或在本目录/~/r1_hand 下补齐依赖。" >&2
  exit 1
fi

# ---- 选 python ----
PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  for c in "$DIR/.venv/bin/python" "$(command -v python3)"; do
    [ -x "$c" ] && { PY="$c"; break; }
  done
fi
[ -n "$PY" ] || { echo "找不到 python3" >&2; exit 1; }

# ---- 平台相关 ----
if [ "$(uname -s)" = "Darwin" ]; then
  # Homebrew 前缀：Apple Silicon 在 /opt/homebrew，Intel 在 /usr/local
  for lib in /opt/homebrew/lib /usr/local/lib; do
    if [ -e "$lib/libusb-1.0.dylib" ] || [ -e "$lib/libusb-1.0.0.dylib" ]; then
      export DYLD_LIBRARY_PATH="$lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
    fi
  done
  if [ -z "${DYLD_LIBRARY_PATH:-}" ]; then
    echo "⚠ 没在 /opt/homebrew/lib 或 /usr/local/lib 找到 libusb —— 可能需要 'brew install libusb'" >&2
  fi
fi

export PYTHONPATH="$DIR${PYTHONPATH:+:$PYTHONPATH}"
[ $# -ge 1 ] || { echo "用法: $(basename "$0") <check_comms.py|test_hand_canfd.py> [参数...]" >&2; exit 2; }

TARGET="$1"; shift
if [ -f "$DIR/$TARGET" ]; then
  TARGET="$DIR/$TARGET"
elif [ -f "$SCRIPT_DIR/$TARGET" ]; then
  TARGET="$SCRIPT_DIR/$TARGET"
fi
[ -f "$TARGET" ] || { echo "找不到脚本: $TARGET" >&2; exit 2; }

# -u：无缓冲。长跑进程（如 hand_bridge.py）在非 TTY（管道/后台/ssh）下会被块缓冲，
#     不加 -u 就看不到任何进度输出，极易误判成"没跑起来"。
exec "$PY" -u "$TARGET" "$@"
