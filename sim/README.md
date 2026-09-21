# sim/ — 本机双臂遥操仿真验证

在 **macOS 本机**（不需要机器人、不需要 DDS）验证 `example/r1/high_level/` 的
IK 精度、坐标对齐、以及 PICO 报文全链路，并做可视化回放。

## 为什么能本机跑

| 模块 | 依赖 | 本机可编译 |
|---|---|---|
| `r1_arm_ik.h`（FK / IK） | 仅 Eigen | ✅ |
| `r1_xr_pose_alignment.h`（对齐） | 仅 Eigen | ✅ |
| `r1_pico_udp.h`（UDP 解析） | SDK 的 `FromJsonString()`（官方实现在 Linux 私有库里） | ✅ 由 `json_shim.cpp` 补实现 |
| `r1_arm_controller.h`（臂控） | unitree DDS 封装 | ❌ 需 SDK |

`r1_pico_udp.h` 能本机编译的关键：`Any`/`JsonMap`/`JsonArray` 都是 header-only
的标准类型，只有 `FromJsonString` 一个函数在库里。`json_shim.cpp` 用
nlohmann/json 解析后转成同构的 `Any`，于是**解析层也能在本机完整运行**。
另有 `sim/shim/sys/*.h` 两个空占位，用来绕开 `unitree/common/decl.hpp`
里 include 的 Linux 专有头（`sys/sysinfo.h`、`sys/timerfd.h`，本层并不使用其符号）。

只有 DDS 臂控层需要上机验证。

## 方法论：避免自证循环

用 **MuJoCo 加载同一份官方 URDF**（`submodules/xr_teleoperate/assets/r1/r1_a*.urdf`）
做**独立正运动学**作为真值，而不是拿 `r1_arm_ik.h` 自己的 FK 去验证它自己的 IK。

```
目标 EE 位姿 ──┬─→ C++ IK (r1_arm_ik.h)      ─→ q ─┐
              │                                    ├─→ MuJoCo FK（同一 URDF）→ 实际 EE 位姿 → 残差
              └─→ (可选) 官方 CasADi/IPOPT IK ─→ q'─┘
```

## 安装依赖

```bash
# 1) C++ 探针（自动下载 Eigen 3.4 到 sim/thirdparty/，已 gitignore）
./sim/build_probe.sh

# 2) Python 侧（建议用隔离 venv）
python3 -m venv .venv && .venv/bin/pip install mujoco numpy matplotlib imageio imageio-ffmpeg
```

## 用法

```bash
# 精度评估：FK 交叉校验 + IK 残差统计 + 轨迹跟踪 → 图表 + JSON
.venv/bin/python sim/eval_ik.py                      # a5 + a7 全跑
.venv/bin/python sim/eval_ik.py --variant a5 --n-ik 500
#   产物：sim/out/{v}_ik_error_box.png、{v}_traj_error.png、{v}_traj_xy.png、eval_summary.json

# 可视化回放：MuJoCo 窗口，每只手两个标记球
#   绿球（下方）= 目标末端（人手给的位置）
#   红球（上方）= 实际末端（IK 解算后到达的位置）
#   两球水平对齐 = 跟踪准确；红球相对绿球偏移 = 误差
.venv/bin/python sim/view_traj.py --variant a5
.venv/bin/python sim/view_traj.py --variant both --mode ik      # 含 WMA 平滑（实机口径）
.venv/bin/python sim/view_traj.py --variant both --headless --record sim/out/teleop.mp4

# 根因诊断：复刻 solveArm 的更新公式，对比"原样/加权/限位"三组
.venv/bin/python sim/diag_ik_rootcause.py
```

## PICO 报文全链路测试（无需 PICO、无需机器人）

`pico_pipeline_test` 与 `r1_dual_arm_loco.cpp` 的 `--pico` 分支**同源**：同一组头文件、
同一套默认参数（`--pico-source controllers` / `--pico-frame head_yaw`），只去掉 DDS。

```bash
# 1) 生成一批 PICO 报文（用官方 URDF 做 FK 造可达位姿，再按对齐的逆变换打包）
.venv/bin/python example/r1/high_level/scripts/pico_sim_sender.py \
    --dry-run --dump /tmp/pico.jsonl --truth-out /tmp/truth.jsonl --duration 3 --hz 30

# 2) 灌进全链路：报文 -> 解析 -> 安全判据 -> 对齐 -> IK
./sim/build/pico_pipeline_test --variant a5 < /tmp/pico.jsonl > /tmp/solved.txt
./sim/build/pico_pipeline_test --variant a5 --raw < /tmp/pico.jsonl   # 跳过 WMA 平滑

# 3) 用 MuJoCo 独立 FK 比对 /tmp/solved.txt 与 /tmp/truth.jsonl 的 EE 位姿
```

**验证闭环**：模拟器在机器人腰部系给出目标腕位姿 → 反推成 OpenXR 位姿发出 →
被测程序解析 + 正向对齐 + IK → 解出的关节角再用 MuJoCo 独立 FK 算 EE 位姿 →
与最初的目标比。全链路往返误差 1e-13 mm（已单独验证），因此残差只反映 IK 与平滑。

### 模拟发送器 `pico_sim_sender.py`

发 UDP-JSON v3 报文到机器人（`example/r1/high_level/r1_dual_arm_loco.cpp --pico` 的输入）。

```bash
# 发到机器人（R1 背包）
.venv/bin/python example/r1/high_level/scripts/pico_sim_sender.py \
    --host 192.168.123.164 --port 9999 --duration 60

# 先生成、后回放（机器人侧没有 MuJoCo 时用这个）
... --dump /tmp/pico.jsonl ;  ... --replay /tmp/pico.jsonl --host 192.168.123.164

# 安全场景：normal | estop | unsafe | lost | dropout | stop_signal | return_zero
... --scenario estop --switch-at 1.5
```

`--pose-mode`：`wave`（关节空间正弦，默认）/ `circle`（任务空间圆周，半径 3cm）。
两者都先经官方 URDF 的 FK，保证发出的位姿是 R1 可达的。

## 文件

| 文件 | 作用 |
|---|---|
| `ik_probe.cpp` | C++ 探针：把 `r1_arm_ik.h` 的 FK/IK 暴露成行式文本接口（stdin/stdout） |
| `build_probe.sh` | 编译探针（自动拉 Eigen） |
| `r1_model.py` | MuJoCo 模型封装：URDF 加载、关节映射、独立 FK、限位采样 |
| `eval_ik.py` | L1 精度评估：FK 交叉校验、IK 残差统计、轨迹跟踪、出图 |
| `view_traj.py` | L2 可视化：MuJoCo 回放 + 目标/实际末端标记 + 可选导出 mp4 |
| `diag_ik_rootcause.py` | 发散问题的根因对照实验 |
| `json_shim.cpp` | 本机补 `unitree::common::FromJsonString`（官方实现在 Linux 库里） |
| `pico_pipeline_test.cpp` | PICO 报文全链路：解析 → 安全判据 → 对齐 → IK |
| `safety_scenarios.py` | 安全场景一键回归（7 个场景，断言判据优先级） |
| `check_syntax.sh` | 本机对主程序/工具做 `-fsyntax-only` 全量语法校验 |
| `shim/sys/*.h` | Linux 专有头（`sysinfo.h`/`timerfd.h`）的 macOS 空占位 |
| `shim/mac_syntax_compat.h` | 语法校验垫片（pthread 自旋锁 / sched 常量），**不参与构建** |
| `../example/r1/high_level/r1_pico_safety_policy.h` | 判据优先级（急停 > 回零 > 保持 > 遥操） |
| `../example/r1/high_level/scripts/pico_sim_sender.py` | PICO 模拟发送器（UDP-JSON v3） |

## 探针接口（供二次开发）

```
ik_probe --variant a5|a7 --mode fk|ik|ik-raw

fk     : q[2n]                                  → L(p3 q4) R(p3 q4)
ik     : L(p3 q4) R(p3 q4) qcur[2n]             → q[2n]（solve，含 WMA 平滑）
ik-raw : 同上                                    → q[2n]（solveArm，无平滑，逐臂）

坐标：骨盆系 = URDF waist_yaw_link 系（waist_yaw 锁 0）；EE 含手安装偏移
      （A5 +0.20m x / A7 +0.05m x）；四元数顺序 x y z w。
```

## 当前基线（2026-09-21，`r1_arm_ik.h` 修复后）

| 指标 | A5 | A7 |
|---|---|---|
| FK 交叉校验（C++ vs MuJoCo） | 位置差 p95 6.3e-10 mm | 6.4e-10 mm |
| IK 冷启动 home→典型位姿（±0.5 rad） | mean 0.13 mm | 0.39 mm |
| IK 任务空间（±0.6 rad，n=300） | mean 0.09 / p95 0.40 mm，100% ≤5mm | 0.33 / 1.07 mm，98.5% |
| 轨迹跟踪 800 帧（warm start，无平滑） | mean 0.03 mm | 0.03 mm |
| 轨迹跟踪（含 WMA 平滑） | mean 4.33 mm | 3.45 mm |
| IK 全域采样（全关节范围，n=300） | mean 29.6 mm，71.8% ≤5mm | 34.8 mm，66.2% |

> 全域采样下的大误差来自两方面：① 5 DOF 手臂跟踪 6D 全姿态的固有折衷；
> ② 目标本身可能不满足关节限位。真实遥操工作空间在任务空间采样一档。

### PICO 报文端到端（A5，3s @30Hz，sway 幅度 0.2 rad）

| 口径 | EE 位置 mean / max | EE 姿态 mean / max |
|---|---|---|
| `--raw`（跳过 WMA 平滑） | **0.108 mm / 0.358 mm** | 0.013° / 0.037° |
| `ik.solve`（含 WMA，= 实机口径） | **7.69 mm / 16.74 mm** | 1.53° / 2.80° |

> 两者差值即 **WMA 平滑的滞后**：`weights=[0.4,0.3,0.2,0.1]` 相当于约 1 帧群延迟，
> 跟随误差 ≈ 手速 × 滞后时间。这一行为与官方 `WeightedMovingFilter` 一致，不是缺陷；
> 若要更跟手，可改 `r1_arm_ik.h::smooth()` 的权重（代价是抖动变大）。

### 安全判据场景验证

判据已集中到 `../example/r1/high_level/r1_pico_safety_policy.h::decideDisposition()`，
优先级 **急停 > 回零 > 保持 > 遥操**。`pico_pipeline_test` 与主程序**共用**该策略，
因此下面的结论就是 `r1_dual_arm_loco.cpp --pico` 分支的结论。

```bash
.venv/bin/python sim/safety_scenarios.py      # 一键回归全部场景
```

| 场景 | 期望 | 实测（2026-09-21 修复后） |
|---|---|---|
| `normal` | 全程执行 | 110 帧全为 `teleop` ✅ |
| `estop`（只置闩锁位，`safe_to_execute` 仍为 `true`） | 切换后停 | `seq>=42` 起恒为 `emergency_stop` ✅ |
| `unsafe`（`safe_to_execute=false`） | 全程停 | 110 帧全为 `hold` ✅ |
| `lost`（手部 `quality=lost`） | 切换后停 | `seq>=42` 起恒为 `hold` ✅ |
| `stop_signal` | 切换后停 + 阻尼 | `seq>=43` 起恒为 `hold` ✅ |
| `return_zero` | 切换后回零 | `seq>=42` 起恒为 `return_zero`，`q≡0` ✅ |
| `stale`（正常包 + `--rx-age-ms 700`） | 全程停 | 110 帧全为 `hold` ✅ |

### 修复的两个安全判据缺口（2026-09-21 已修）

1. **`emergency_stop_latched` 未纳入执行判据**。`safeToExecute()` 原先只看
   `safe_to_execute && operator_mode=="active_stream"`：PICO 端若只置急停位、
   而 `safe_to_execute` 仍为 `true`，机器人不会停。
   → 现在 `safeToExecute()` 第一位就判 `!emergency_stop_latched`，
   且急停处置为「停移动 + 阻尼」，与官方 `xr_teleoperate` 的软急停（`Damp()`）一致。
2. **`return_zero` 是死代码**。主循环里 `if (!safe) { …; continue; }` 排在
   `if (operator_mode == "return_zero")` 之前，而 `return_zero` 时 `safeToExecute()`
   恒为 false（模式不是 `active_stream`），一键回零永远不触发。
   → 判据下沉为 `PicoTeleopPacket::returnZeroRequested()` + 纯函数
   `decideDisposition()`，并用单测锁住顺序（`tests/test_pico_parse.cpp` 第 7 节）。

**反向对照**（用修复前的 `HEAD` 版本编译同一份测试程序，验证新断言确实抓得住）：

| 场景 | 修复前 | 修复后 |
|---|---|---|
| `estop` 切换后仍判定可执行的帧数 | **69 / 69**（继续执行运动） | 0 / 69 |
| `return_zero` 切换后真正输出关节目标的帧数 | **0 / 69**（分支不可达） | 69 / 69（`q≡0`） |

## 本机语法校验（`check_syntax.sh`）

`r1_dual_arm_loco.cpp` / `r1_tool.cpp` 依赖 unitree SDK 头文件，而 SDK 会 include
Linux 专有头（`sys/sysinfo.h`、`pthread` 自旋锁、sched 策略常量），在 macOS 上直接
编译会先挂在 SDK 里，看不到自己代码的错误。`check_syntax.sh` 用 `-fsyntax-only`
加一层垫片（`shim/mac_syntax_compat.h`，**只用于语法校验，不参与构建**）来绕开，
于是 AGENTS.md 里那条「clang 下 0 error」的基线可以在本机随时复现：

```bash
./sim/check_syntax.sh                 # 两个目标文件
./sim/check_syntax.sh r1_tool.cpp     # 指定文件
```

本次修复中，主程序漏 include `r1_pico_safety_policy.h` 就是被这个脚本抓住的
（sim 侧不编译主程序，否则会一路漏到 VM 构建才暴露）。
