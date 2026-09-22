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

# nlohmann/json 单头（json_shim.cpp 用；同样不入库）
# 先写 .part 再改名，避免网络中断留下不完整文件被误当成已就绪
JSON_VER="3.11.3"
JSON_MIN=500000
json_size=0
[ -f "$DIR/thirdparty/json.hpp" ] && json_size=$(wc -c < "$DIR/thirdparty/json.hpp" | tr -d ' ')
if [ "$json_size" -lt "$JSON_MIN" ]; then
  echo "[build_probe] 下载 nlohmann/json ${JSON_VER} ..."
  curl -fsSL --max-time 300 -o "$DIR/thirdparty/json.hpp.part" \
    "https://raw.githubusercontent.com/nlohmann/json/v${JSON_VER}/single_include/nlohmann/json.hpp"
  mv "$DIR/thirdparty/json.hpp.part" "$DIR/thirdparty/json.hpp"
fi

echo "[build_probe] 编译 ik_probe ..."
c++ -std=c++17 -O2 \
  -I "$DIR/thirdparty" \
  -I "$SDK/example/r1/high_level" \
  "$DIR/ik_probe.cpp" -o "$DIR/build/ik_probe"

# PICO 全链路测试：需要 SDK 头 + Linux 专有头的 macOS 占位(sim/shim)
echo "[build_probe] 编译 pico_pipeline_test ..."
c++ -std=c++17 -O2 \
  -I "$DIR/shim" \
  -I "$DIR/thirdparty" \
  -I "$SDK/include" \
  -I "$SDK/thirdparty/include" \
  -I "$SDK/example/r1/high_level" \
  "$DIR/json_shim.cpp" "$DIR/pico_pipeline_test.cpp" \
  -o "$DIR/build/pico_pipeline_test"

echo "[build_probe] 完成:"
echo "  $DIR/build/ik_probe            (IK 探针)"
echo "  $DIR/build/pico_pipeline_test  (PICO 报文全链路)"
