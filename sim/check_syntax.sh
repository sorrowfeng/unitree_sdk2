#!/usr/bin/env bash
# ============================================================================
# check_syntax.sh —— 在 macOS 本机对 main/工具源文件做全量语法校验
#
# 背景：r1_dual_arm_loco.cpp / r1_tool.cpp 依赖 unitree SDK 头文件，而 SDK 头
#       会 include Linux 专有头（sys/sysinfo.h、pthread 自旋锁、sched 策略常量），
#       在 macOS 上直接编译会先挂在 SDK 里，看不到我们自己代码的错误。
#       本脚本用 -fsyntax-only + 垫片把 SDK 部分"骗过"编译器，只校验我们的代码。
#
# 注意：这只是语法校验，不是构建。真正的编译验证仍在 aarch64 VM 上做（deploy.sh）。
#       语法校验抓得住拼错的标识符、漏掉的 include、类型/枚举写错等——本次修复中
#       主程序漏 include 策略头就是这样被抓住的。
#
# 用法：
#   ./sim/check_syntax.sh              # 校验全部目标
#   ./sim/check_syntax.sh r1_tool.cpp  # 只校验指定文件
# ============================================================================
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK="$(cd "$DIR/.." && pwd)"

EIGEN_SRC="$DIR/thirdparty/eigen-3.4.0"
if [ ! -d "$EIGEN_SRC" ]; then
  echo "[check_syntax] 请先运行 ./sim/build_probe.sh 拉取 Eigen"
  exit 2
fi
# 源码用 #include <eigen3/Eigen/Dense>，需要一个含 eigen3 的父目录
INC_GEN="$DIR/build/inc"
mkdir -p "$INC_GEN"
ln -sfn "$EIGEN_SRC" "$INC_GEN/eigen3"

TARGETS=("$@")
if [ ${#TARGETS[@]} -eq 0 ]; then
  TARGETS=(r1_dual_arm_loco.cpp r1_tool.cpp)
fi

COMMON=(
  -std=c++17 -fsyntax-only
  -Wall -Wextra -Wno-unused-parameter
  -include "$DIR/shim/mac_syntax_compat.h"   # 仅语法校验垫片，不参与构建
  -I "$INC_GEN"
  -I "$DIR/shim"
  -I "$SDK/include"
  -I "$SDK/thirdparty/include"
  -I "$SDK/thirdparty/include/ddscxx"
  -I "$SDK/thirdparty/include/ddsc"
  -I "$SDK/example/r1/high_level"
)

fail=0
for t in "${TARGETS[@]}"; do
  src="$SDK/example/r1/high_level/$t"
  if [ ! -f "$src" ]; then
    echo "[check_syntax] 找不到 $src"
    fail=1
    continue
  fi
  printf '[check_syntax] %-24s ' "$t"
  out=$(clang++ "${COMMON[@]}" "$src" 2>&1)
  n=$(printf '%s' "$out" | grep -c "error:" || true)
  if [ "$n" -eq 0 ]; then
    echo "0 error"
  else
    echo "$n error"
    printf '%s\n' "$out" | grep "error:" | head -20
    fail=1
  fi
done

exit $fail
