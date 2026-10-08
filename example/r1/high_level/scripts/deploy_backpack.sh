#!/usr/bin/env bash
# 本地改代码 -> 同步到 R1-EDU 算力背包（PC2 / aarch64）-> 远程编译 + 解析层单测。
#
# 用法:
#   BACKPACK_PASS=<密码> scripts/deploy_backpack.sh               # 首次：同步 SDK 骨架 + example/r1，编译 + 单测
#   BACKPACK_PASS=<密码> scripts/deploy_backpack.sh --incremental # 日常：只同步 example/r1
#   BACKPACK_PASS=<密码> scripts/deploy_backpack.sh --sync-only   # 只同步，绝不编译/不运行（安全档）
#   BACKPACK_PASS=<密码> scripts/deploy_backpack.sh --no-test     # 只编译
#
# 可用环境变量覆盖:
#   BACKPACK_HOST(默认 192.168.123.164) BACKPACK_USER(unitree) BACKPACK_PORT(22)
#   VM_DIR(~/unitree_sdk2)  ← 背包上「我们自己的工作树」；与 deploy.sh 共用该变量名
#
# ⚠️ 目录角色（2026-09-23 定案，别再混）：
#   ~/unitree_sdk2                      = **我们的工作树**（SDK 骨架 + 我们的改动），唯一编译/运行来源。
#   ~/unitree_sdk2.factory.<时间戳>     = 宇树工厂原状的**只读副本**（git 仓库 main @ fa925bf，
#                                         `chmod -R a-w`）。只作回滚锚点与参考，绝不往这里同步。
#                                         恢复：rm -rf ~/unitree_sdk2 && mv ~/unitree_sdk2.factory.<TS> ~/unitree_sdk2
#                                               && chmod -R u+w ~/unitree_sdk2
#   ~/unitree_sdk2-main(+zip)           = 2026-02 官方 zip 解压本，与我们无关，别动。
#
#   为什么不能省掉那份骨架、直接复用工厂版当 SDK：工厂版 include/ 里连我们依赖的头文件都没有
#   （audio_client.hpp、r1.h、r1_pub.h 等是上游后来新增的），复用它编译会在第一步 fatal error。
#
# ⚠️ 若目标目录已存在别的东西，先改名备份再部署：
#      mv ~/unitree_sdk2 ~/unitree_sdk2.bak.$(date +%Y%m%d_%H%M%S)
#
# 与 deploy.sh 的区别（为什么单独一个脚本）:
#   1) 目标是算力背包 PC2，不是 Parallels 测试 VM（默认 HOST/USER 不同）；
#   2) 默认模式是「骨架全量同步」——背包上很可能还没有本仓库，只同步 example/r1 会缺
#      CMakeLists.txt / cmake/ / include/ / lib/ / thirdparty/，cmake 配置阶段直接失败；
#   3) 明确排除 submodules/(343MB，只有本机 sim/ 用得到) 与 sim/(macOS 侧 MuJoCo 工具)，
#      避免首次同步白传 370MB。
#
# 同步范围必须 ≥ example/r1/CMakeLists.txt 的可见范围（high_level/、low_level/、audio/），
# 故 example/r1/ 整个目录一起走，不能只挑 high_level/。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

HOST="${BACKPACK_HOST:-192.168.123.164}"
RUSER="${BACKPACK_USER:-unitree}"
PORT="${BACKPACK_PORT:-22}"
DIR="${VM_DIR:-~/unitree_sdk2}"
PASS="${BACKPACK_PASS:-${VM_PASS:-}}"
SSH_EXP="${SCRIPT_DIR}/_vm_ssh.exp"
RSYNC_EXP="${SCRIPT_DIR}/_vm_rsync.exp"

command -v expect >/dev/null 2>&1 || { echo "[deploy_backpack] 需要 expect（macOS 自带）" >&2; exit 1; }

# 参数先于密码校验解析：--help 不需要密码。
MODE=full
DO_TEST=1
DO_BUILD=1
for a in "$@"; do
  case "$a" in
    --incremental) MODE=incremental ;;
    --sync-only)   DO_BUILD=0; DO_TEST=0 ;;
    --no-test)     DO_TEST=0 ;;
    -h|--help)     sed -n '2,35p' "$0"; exit 0 ;;
    *) echo "[deploy_backpack] 未知参数: $a" >&2; exit 1 ;;
  esac
done

if [ -z "${PASS}" ]; then
  echo "[deploy_backpack] 请设置 BACKPACK_PASS 环境变量（背包登录密码），例如：" >&2
  echo "                  BACKPACK_PASS=*** scripts/deploy_backpack.sh" >&2
  exit 1
fi

echo "[deploy_backpack] ${RUSER}@${HOST}:${PORT}  dir=${DIR}  mode=${MODE}"

sync() { expect "${RSYNC_EXP}" "${PASS}" "${PORT}" "${RUSER}" "${HOST}" "$@"; }

if [ "${MODE}" = incremental ]; then
  echo "[deploy_backpack] 增量同步 example/r1 ..."
  sync "${ROOT}/example/r1/" "${DIR}/example/r1/"
else
  echo "[deploy_backpack] 同步 SDK 骨架（排除 .git/build/submodules/sim）..."
  sync "${ROOT}/" "${DIR}/" \
    --exclude=build \
    --exclude=submodules \
    --exclude=sim \
    --exclude=.workbuddy \
    --exclude=.DS_Store
fi

if [ "${DO_BUILD}" = 0 ]; then
  echo "[deploy_backpack] --sync-only：已按要求跳过远程 chmod/编译/测试，未在背包上执行任何命令（除只读核对）。"
  echo "[deploy_backpack] 同步完成。产物尚未编译 —— 需要时再单独跑一次不带 --sync-only 的命令。"
  exit 0
fi

REMOTE_CMD="cd ${DIR} && chmod +x example/r1/high_level/scripts/*.sh && example/r1/high_level/scripts/build.sh"
if [ "${DO_TEST}" = 1 ]; then
  REMOTE_CMD="${REMOTE_CMD} && example/r1/high_level/scripts/run_tests.sh"
  echo "[deploy_backpack] 远程编译 / 测试 ..."
else
  echo "[deploy_backpack] 远程编译（跳过测试）..."
fi

expect "${SSH_EXP}" "${PASS}" "${PORT}" "${RUSER}" "${HOST}" "${REMOTE_CMD}"
echo "[deploy_backpack] 完成。产物: ${DIR}/build/bin/"
