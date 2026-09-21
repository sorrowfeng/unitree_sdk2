#!/usr/bin/env bash
# 构建 R1 PICO 遥操作项目（unitree_sdk2 + example/r1/high_level）。
#
# 用法:
#   scripts/build.sh                 # 只编译 r1_dual_arm_loco_skeleton（默认）
#   scripts/build.sh all             # 编译 SDK 全部示例
#   scripts/build.sh <target>        # 编译指定 CMake 目标
#   BUILD_TYPE=Debug scripts/build.sh
#
# 依赖 (Ubuntu 20.04/22.04，x86_64 或 aarch64 均可):
#   sudo apt install -y cmake g++ build-essential \
#     libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"   # unitree_sdk2 仓库根目录
BUILD_DIR="${ROOT}/build"
BUILD_TYPE="${BUILD_TYPE:-Release}"
TARGET="${1:-r1_dual_arm_loco_skeleton}"
JOBS="$( (command -v nproc >/dev/null 2>&1 && nproc) || sysctl -n hw.ncpu 2>/dev/null || echo 2 )"

echo "[build] repo root : ${ROOT}"
echo "[build] build dir : ${BUILD_DIR}"
echo "[build] arch      : $(uname -m)"
echo "[build] type      : ${BUILD_TYPE}"
echo "[build] target    : ${TARGET}"

# ---- 依赖自检 ----
missing=()
for c in cmake make g++; do
  command -v "$c" >/dev/null 2>&1 || missing+=("$c")
done
for h in /usr/include/eigen3/Eigen/Dense /usr/include/yaml-cpp/yaml.h /usr/include/fmt/format.h; do
  [ -e "$h" ] || missing+=("$h")
done
if [ "${#missing[@]}" -gt 0 ]; then
  echo "[build] 缺少依赖: ${missing[*]}" >&2
  echo "        sudo apt install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev" >&2
  exit 1
fi

cmake -S "${ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
if [ "${TARGET}" = "all" ]; then
  cmake --build "${BUILD_DIR}" -j "${JOBS}"
  echo "[build] OK (all targets) -> ${BUILD_DIR}/bin/"
else
  cmake --build "${BUILD_DIR}" -j "${JOBS}" --target "${TARGET}"
  echo "[build] OK -> ${BUILD_DIR}/bin/${TARGET}"
fi
