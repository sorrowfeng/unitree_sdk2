#!/usr/bin/env bash
# 本地改代码 -> 同步到 aarch64 VM -> 远程编译 + 测试。
#
# 用法:
#   VM_PASS=<密码> scripts/deploy.sh            # 增量同步 + 远程 build + run_tests
#   VM_PASS=<密码> scripts/deploy.sh --full     # 首次：全量同步整个 SDK（排除 .git/build）
#   VM_PASS=<密码> scripts/deploy.sh --no-test  # 只编译，不跑测试
#
# 可用环境变量覆盖：
#   VM_HOST(默认 10.211.55.6) VM_USER(plf-virtual) VM_PORT(22)
#   VM_DIR(~/RobotProject/unitree_sdk2)
#
# 同步范围（增量）：example/r1/high_level/ 与 example/r1/CMakeLists.txt。
# 若改动其它 SDK 文件，用 --full。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
HIGH_LEVEL="${ROOT}/example/r1/high_level"

VM_HOST="${VM_HOST:-10.211.55.6}"
VM_USER="${VM_USER:-plf-virtual}"
VM_PORT="${VM_PORT:-22}"
VM_DIR="${VM_DIR:-~/RobotProject/unitree_sdk2}"
SSH_EXP="${SCRIPT_DIR}/_vm_ssh.exp"
RSYNC_EXP="${SCRIPT_DIR}/_vm_rsync.exp"

if [ -z "${VM_PASS:-}" ]; then
  echo "[deploy] 请设置 VM_PASS 环境变量（VM 登录密码），例如：" >&2
  echo "         VM_PASS=你的密码 scripts/deploy.sh" >&2
  exit 1
fi
command -v expect >/dev/null 2>&1 || { echo "[deploy] 需要 expect（macOS 自带）" >&2; exit 1; }

MODE=incremental
DO_TEST=1
for a in "$@"; do
  case "$a" in
    --full)    MODE=full ;;
    --no-test) DO_TEST=0 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "[deploy] 未知参数: $a" >&2; exit 1 ;;
  esac
done

echo "[deploy] ${VM_USER}@${VM_HOST}:${VM_PORT}  dir=${VM_DIR}  mode=${MODE}"

sync() { expect "${RSYNC_EXP}" "${VM_PASS}" "${VM_PORT}" "${VM_USER}" "${VM_HOST}" "$@"; }

if [ "${MODE}" = full ]; then
  echo "[deploy] 全量同步（排除 .git / build）..."
  sync "${ROOT}/" "${VM_DIR}/" --exclude=build
else
  echo "[deploy] 增量同步 r1/high_level ..."
  sync "${HIGH_LEVEL}/" "${VM_DIR}/example/r1/high_level/"
  sync "${ROOT}/example/r1/CMakeLists.txt" "${VM_DIR}/example/r1/CMakeLists.txt"
fi

REMOTE_CMD="cd ${VM_DIR} && chmod +x example/r1/high_level/scripts/*.sh && example/r1/high_level/scripts/build.sh"
if [ "${DO_TEST}" = 1 ]; then
  REMOTE_CMD="${REMOTE_CMD} && example/r1/high_level/scripts/run_tests.sh"
fi

echo "[deploy] 远程编译${DO_TEST:+ / 测试} ..."
expect "${SSH_EXP}" "${VM_PASS}" "${VM_PORT}" "${VM_USER}" "${VM_HOST}" "${REMOTE_CMD}"
echo "[deploy] 完成"
