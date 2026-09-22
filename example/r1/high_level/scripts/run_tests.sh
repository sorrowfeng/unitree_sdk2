#!/usr/bin/env bash
# 编译并运行不接机器人的 PICO 报文解析/对齐测试。
#
# 用法:
#   scripts/run_tests.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
HIGH_LEVEL="$(cd "${SCRIPT_DIR}/.." && pwd)"
ARCH="$(uname -m)"
OUT_DIR="${ROOT}/build"
OUT="${OUT_DIR}/test_pico_parse"
LIB_DIR="${ROOT}/lib/${ARCH}"
TP_LIB_DIR="${ROOT}/thirdparty/lib/${ARCH}"

if [ ! -f "${LIB_DIR}/libunitree_sdk2.a" ]; then
  echo "[test] 找不到 ${LIB_DIR}/libunitree_sdk2.a（架构 ${ARCH} 是否被 SDK 支持？）" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"
echo "[test] arch=${ARCH} out=${OUT}"
g++ -std=c++17 -O2 \
  -I"${ROOT}/include" \
  -I"${ROOT}/thirdparty/include" \
  -I"${ROOT}/thirdparty/include/ddscxx" \
  -I"${ROOT}/thirdparty/include/ddsc" \
  -I"${HIGH_LEVEL}" \
  "${HIGH_LEVEL}/tests/test_pico_parse.cpp" -o "${OUT}" \
  -L"${TP_LIB_DIR}" -Wl,-rpath,"${TP_LIB_DIR}" \
  "${LIB_DIR}/libunitree_sdk2.a" -lddsc -lddscxx -lpthread -ldl

echo "[test] running..."
exec "${OUT}"
