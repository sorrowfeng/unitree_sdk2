#!/usr/bin/env bash
# 编译 C++ 运动学探针（本机可跑，仅依赖 Eigen）
# Eigen 首次会自动下载到 sim/thirdparty/（已 gitignore，不入库）
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK="$(cd "$DIR/.." && pwd)"
EIGEN_VER="3.4.0"

mkdir -p "$DIR/thirdparty" "$DIR/build"

if [ ! -d "$DIR/thirdparty/eigen-${EIGEN_VER}/Eigen" ]; then
  echo "[build_probe] 下载 Eigen ${EIGEN_VER} ..."
  curl -sL --max-time 180 -o "$DIR/thirdparty/eigen.tar.gz" \
    "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VER}/eigen-${EIGEN_VER}.tar.gz"
  tar xzf "$DIR/thirdparty/eigen.tar.gz" -C "$DIR/thirdparty"
  rm -f "$DIR/thirdparty/eigen.tar.gz"
fi
ln -sfn "eigen-${EIGEN_VER}" "$DIR/thirdparty/eigen3"

echo "[build_probe] 编译 ik_probe ..."
c++ -std=c++17 -O2 \
  -I "$DIR/thirdparty" \
  -I "$SDK/example/r1/high_level" \
  "$DIR/ik_probe.cpp" -o "$DIR/build/ik_probe"

echo "[build_probe] 完成: $DIR/build/ik_probe"
