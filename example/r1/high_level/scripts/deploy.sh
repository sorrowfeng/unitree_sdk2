#!/usr/bin/env bash
# 本地改代码 -> 同步到 aarch64 VM -> 远程编译 + 测试。
#
# 用法:
#   VM_PASS=<密码> scripts/deploy.sh            # 增量同步 + 远程 build + run_tests
#   VM_PASS=<密码> scripts/deploy.sh --example  # 同步整个 example/（上游动了别的示例目录时）
#   VM_PASS=<密码> scripts/deploy.sh --full     # 首次：全量同步整个 SDK（排除 .git/build）
#   VM_PASS=<密码> scripts/deploy.sh --no-test  # 只编译，不跑测试
#
# 可用环境变量覆盖：
#   VM_HOST(默认 10.211.55.6) VM_USER(plf-virtual) VM_PORT(22)
#   VM_DIR(~/RobotProject/unitree_sdk2)
#
# 同步范围（增量）：example/r1/ 整个目录。
#   ⚠️ 范围必须 ≥ CMakeLists 的可见范围：example/r1/CMakeLists.txt 里的 target 引用了
#   high_level/、low_level/、audio/ 三处源文件。早先只同步 high_level/ + CMakeLists.txt，
#   上游把 low_level/r1A_wrist_swing_example.cpp 改名为 r1_A5/A7_... 并新增
#   high_level/r1_arm_action_example.cpp 后，cmake 配置阶段即报「找不到源文件」。
#   （既不上传 submodules/ 343MB，也不动 VM 上其它示例目录，故为默认。）
# 上游改动其它示例目录（如 example/g1/）→ --example；改动 include/、lib/、thirdparty/
# 等 SDK 核心 → --full。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

VM_HOST="${VM_HOST:-10.211.55.6}"
VM_USER="${VM_USER:-plf-virtual}"
VM_PORT="${VM_PORT:-22}"
VM_DIR="${VM_DIR:-~/RobotProject/unitree_sdk2}"
SSH_EXP="${SCRIPT_DIR}/_vm_ssh.exp"
RSYNC_EXP="${SCRIPT_DIR}/_vm_rsync.exp"

command -v expect >/dev/null 2>&1 || { echo "[deploy] 需要 expect（macOS 自带）" >&2; exit 1; }

# 参数先于密码校验解析：--help 不需要密码。
MODE=incremental
DO_TEST=1
for a in "$@"; do
  case "$a" in
    --full)    MODE=full ;;
    --example) MODE=example ;;
    --no-test) DO_TEST=0 ;;
    -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "[deploy] 未知参数: $a" >&2; exit 1 ;;
  esac
done

if [ -z "${VM_PASS:-}" ]; then
  echo "[deploy] 请设置 VM_PASS 环境变量（VM 登录密码），例如：" >&2
  echo "         VM_PASS=你的密码 scripts/deploy.sh" >&2
  exit 1
fi

echo "[deploy] ${VM_USER}@${VM_HOST}:${VM_PORT}  dir=${VM_DIR}  mode=${MODE}"

sync() { expect "${RSYNC_EXP}" "${VM_PASS}" "${VM_PORT}" "${VM_USER}" "${VM_HOST}" "$@"; }

case "${MODE}" in
  full)
    echo "[deploy] 全量同步整个 SDK（排除 .git / build）..."
    sync "${ROOT}/" "${VM_DIR}/" --exclude=build
    ;;
  example)
    echo "[deploy] 同步整个 example/ ..."
    sync "${ROOT}/example/" "${VM_DIR}/example/"
    ;;
  *)
    echo "[deploy] 增量同步 example/r1 ..."
    sync "${ROOT}/example/r1/" "${VM_DIR}/example/r1/"
    ;;
esac

REMOTE_CMD="cd ${VM_DIR} && chmod +x example/r1/high_level/scripts/*.sh && example/r1/high_level/scripts/build.sh"
if [ "${DO_TEST}" = 1 ]; then
  REMOTE_CMD="${REMOTE_CMD} && example/r1/high_level/scripts/run_tests.sh"
fi

if [ "${DO_TEST}" = 1 ]; then
  echo "[deploy] 远程编译 / 测试 ..."
else
  echo "[deploy] 远程编译（跳过测试）..."
fi
expect "${SSH_EXP}" "${VM_PASS}" "${VM_PORT}" "${VM_USER}" "${VM_HOST}" "${REMOTE_CMD}"
echo "[deploy] 完成"
