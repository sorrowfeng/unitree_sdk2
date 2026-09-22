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

## 环境准备

**本机已装好，直接复用下面这个解释器即可**（项目内**没有** `.venv`，下面的 `$PY` 请先 export 一次）：

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
$PY -c "import mujoco,numpy,matplotlib,imageio; print('mujoco', mujoco.__version__)"   # 自检，应打印 3.13.0
```

换机器时也可以自建 venv，然后把 `PY` 指向它：

```bash
python3 -m venv .venv && .venv/bin/pip install mujoco numpy matplotlib imageio imageio-ffmpeg
```

C++ 探针（首次自动下载 Eigen 3.4 到 `sim/thirdparty/`，已 gitignore）：

```bash
./sim/build_probe.sh     # 产出 sim/build/{ik_probe,pico_pipeline_test,test_pico_parse}
```

改过 `example/r1/high_level/` 的头文件后，**必须重跑一次 `build_probe.sh`**，
否则探针还是旧逻辑（探针不在 CMake 构建里，不会自动跟着更新）。

## 快速开始（推荐顺序）

```bash
cd <repo>/unitree_sdk2
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python

./sim/check_syntax.sh        # ① 秒级：主程序/工具语法自检，看两个文件是否 0 error
$PY sim/eval_ik.py           # ② 约 3s：IK 精度 + 出图到 sim/out/
$PY sim/safety_scenarios.py  # ③ 约 27s：7 个安全场景回归，末行应打印 ALL SCENARIOS PASSED
$PY sim/view_traj.py --variant both    # ④ 开 MuJoCo 窗口，肉眼双臂跟踪（关窗即退出）
# ⑤ 接 PICO 的实时可视化：$PY sim/view_pico.py --variant a5 --port 9999（需另开终端发报文，见下文）
```

前两步不需要 MuJoCo 窗口，适合改完代码立刻回归；第 ④ 步是给人看的主观检查。

## 用法

```bash
# 精度评估：FK 交叉校验 + IK 残差统计 + 轨迹跟踪 → 图表 + JSON
$PY sim/eval_ik.py                      # a5 + a7 全跑
$PY sim/eval_ik.py --variant a5 --n-ik 500
#   产物：sim/out/{v}_ik_error_box.png、{v}_traj_error.png、{v}_traj_xy.png、eval_summary.json

# 可视化回放：MuJoCo 窗口，每只手两个标记球
#   绿球（下方）= 目标末端（人手给的位置）
#   红球（上方）= 实际末端（IK 解算后到达的位置）
#   两球水平对齐 = 跟踪准确；红球相对绿球偏移 = 误差
$PY sim/view_traj.py --variant a5
$PY sim/view_traj.py --variant both --mode ik      # 含 WMA 平滑（实机口径）
$PY sim/view_traj.py --variant both --headless --record sim/out/teleop.mp4
#   注意：--variant both 时 --record 会按变体插后缀，实际写出 teleop_a5.mp4 / teleop_a7.mp4
#   （离屏渲染可正常工作，会打印 ffmpeg 的 900x640 -> 912x640 resize 提示，属正常）

# 根因诊断：复刻 solveArm 的更新公式，对比"原样/加权/限位"三组
$PY sim/diag_ik_rootcause.py
```

## 可视化 GUI 验证（两条路径）

本机有两类**互不相同**的 GUI 验证，别混用：

| | 不接 PICO | 接 PICO |
|---|---|---|
| 脚本 | `view_traj.py` | `view_pico.py` |
| 目标位姿从哪来 | MuJoCo 自己生成的关节轨迹，再取官方 URDF 的 FK 真值 | PICO 头显（或 `pico_sim_sender.py`）经 UDP 发来的 OpenXR 位姿 |
| 实际验证的是 | IK 求解器本身的精度 | 整条遥操作链路（解析 → 安全判据 → 坐标对齐 → IK）在真实数据下的表现 |
| 需要 PICO 吗 | 不需要 | 需要，或用模拟发送器代替 |

两者窗口里的记号一致：**绿球 = 目标腕位置**，**红球 = 实际腕位置**（IK 解算后 MuJoCo FK 到达的位置）。
红绿贴合＝跟得准，红球偏移量就是误差。

### 路径 A：不接 PICO

```bash
$PY sim/view_traj.py --variant both            # A5 / A7 各开一次窗口
$PY sim/view_traj.py --variant a5 --mode ik    # 含 WMA 平滑（实机主程序口径）
```

### 路径 B：接 PICO（两个终端）

```bash
# 终端 1：开窗等报文
$PY sim/view_pico.py --variant a5 --port 9999

# 终端 2：真 PICO 直接把报文发到本机 9999；没有头显时用模拟发送器
$PY example/r1/high_level/scripts/pico_sim_sender.py \
    --host 127.0.0.1 --port 9999 --duration 30
```

窗口左上角实时显示 `disp=`（`teleop` / `hold` / `return_zero` / `emergency_stop`）与报文序号。
想用肉眼确认安全判据，直接上带场景的发送器：

```bash
$PY example/r1/high_level/scripts/pico_sim_sender.py --host 127.0.0.1 --port 9999 \
    --scenario estop --switch-at 3 --duration 8     # 3 秒后触发急停，手臂应立即停住
```

`view_pico.py` 只取最新一包（丢弃积压），避免乱序时"补播"旧动作；因此发送 300 包、
窗口处理 200 出头属于正常，不是丢帧故障。

### HUD 文字为什么容易看不清（两个坑）

`viewer.set_texts()` 的官方 docstring 说第一个参数是 `mjtFontScale`，**这是错的**。
它最终会原样传给 `mjr_overlay`，而 `mujoco.h` 的声明写得很明确：

```
// Draw text overlay; font is mjtFont; gridpos is mjtGridPos.
MJAPI void mjr_overlay(int font, int gridpos, mjrRect viewport, ...);
```

即只认 **`mjtFont`（`mjFONT_NORMAL=0` / `mjFONT_SHADOW=1` / `mjFONT_BIG=2`）**。
传 `mjFONTSCALE_150`（整数值 150）是非法值，会被静默降成最小号字：1280px 画布上
字形只有 15px 高，而 Retina 下 framebuffer 是 2x，换算到屏幕只剩约 7pt。

实测（同一段文字、同一场景，离屏差分量的字形高度）：

| 传入 font | 一行尺寸 | 说明 |
|---|---|---|
| `mjFONTSCALE_150`（=150） | 444 x 22 px | 非法值 → 最小号字 |
| `mjFONT_NORMAL`（=0） | 444 x 22 px | 与上式**逐像素相同** |
| `mjFONT_BIG`（=2） | 856 x 38 px | 真正的大字，约 2 倍 |

第二个坑：MuJoCo 的 overlay 用的是**内置点阵字体，只覆盖 ASCII**。中文没有字形，
会渲染成实心方块（小号字时）或干脆消失（大号字时）。所以 HUD 文案必须全英文 ——
`err 7.69 mm  pkts 215`，而不是 `误差 7.69 mm  包 215`。

`view_pico.py` 已默认用 `mjFONT_NORMAL`（原始字号，一行约 444x22 px）且 HUD 全 ASCII。
实测 `mjFONT_BIG` 在真实窗口里偏大，故不作默认；嫌小可 `--font big`。

### macOS 开窗须知

`mujoco.viewer.launch_passive()` 要求 Cocoa 事件循环占据真正的 macOS 主线程，
所以必须由 `mjpython` 启动，普通 python 会直接抛
``RuntimeError: `launch_passive` requires that the Python script be run under `mjpython` on macOS``。

两个 viewer 都会**自动检测并 `os.execve` 到同目录的 `mjpython`**（实现在 `gui_boot.py`，
顺带修掉 venv 符号链接导致的 `libpython` dlopen 失败），所以直接
`$PY sim/view_traj.py` / `$PY sim/view_pico.py` 即可，不用手动设任何环境变量。
`--headless` 离屏渲染不经过此路径，普通 python 就能跑。

## PICO 报文全链路测试（无需 PICO、无需机器人）

`pico_pipeline_test` 与 `r1_dual_arm_loco.cpp` 的 `--pico` 分支**同源**：同一组头文件、
同一套默认参数（`--pico-source controllers` / `--pico-frame head_yaw`），只去掉 DDS。

```bash
# 1) 生成一批 PICO 报文（用官方 URDF 做 FK 造可达位姿，再按对齐的逆变换打包）
$PY example/r1/high_level/scripts/pico_sim_sender.py \
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
$PY example/r1/high_level/scripts/pico_sim_sender.py \
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
| `view_traj.py` | L2 可视化（**不接 PICO**）：MuJoCo 回放 + 目标/实际末端标记 + 可选导出 mp4 |
| `view_pico.py` | L2 可视化（**接 PICO**）：监听 UDP，实时把 PICO 报文驱动的双臂动作显示出来 |
| `gui_boot.py` | macOS 开窗适配：自动把普通 python 切到 `mjpython`，并修 libpython 搜索路径 |
| `diag_ik_rootcause.py` | 发散问题的根因对照实验（**注意：其「as-written」模型是修复前的旧公式**，仅作历史留档） |
| `check_official_parity.py` | 机检关节链参数（origin/rpy/axis/限位/EE 偏移）vs 官方 URDF，逐项零偏差判据 |
| `check_ik_vs_official.py` | 同批目标点：官方目标函数（pinocchio + SLSQP）vs 我方 DLS，比**解**与**目标函数值 J** |
| `check_ik_stress.py` | 随机远目标 + warm-start 偏离扫描（回答"实机手速下会不会分叉"） |
| `check_pico_e2e.py` | PICO 端到端残差：合成报文 → 全链路 → 解出关节角 → MuJoCo 独立 FK 比对 |
| `bridge_ik.cpp` | 行协议桥（`FK`/`IKL`/`IKL2`/`IKEOBJ`），把 C++ 求解器暴露给 Python 驱动 |
| `dump_chain.cpp` | 导出关节链参数供与官方 URDF 机检 |
| `probe_ik_solver.cpp` | 求解器受控实验（位置追踪 / 初值扫描 / 姿态扫描 / 权重消融） |
| `probe_reach.py` | 反算目标 → 真实管线 → MuJoCo 独立 FK 校验，带顶限位标记 |
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

## 当前基线（2026-09-22，`r1_arm_ik.h` 补齐官方**四项目标**后）

| 指标 | A5 | A7 |
|---|---|---|
| FK 交叉校验（C++ vs MuJoCo） | 位置差 p95 6.5e-10 mm | 6.4e-10 mm |
| FK 交叉校验（C++ vs 官方 pinocchio） | 位置 6e-11 m / 姿态 9e-4° | 同 |
| **官方目标函数逐点对照**（`check_ik_vs_official.py`） | 我方 52.5 vs 官方 52.4 mm，逐点差 max **2.79 mm** | 109.9 vs 110.7 mm，max **4.42 mm** |
| **同输入目标函数值 J**（逐样本差） | mean 0.0018 / max 0.050 | mean ~0.000 |
| 随机目标压力（warm start σ=0.1 rad，`check_ik_stress.py`） | 4.11 vs 官方 4.09 mm，逐点差 p95 0.49 mm | 4.77 vs 4.73 mm，p95 0.34 mm |
| 轨迹跟踪 800 帧（warm start，无平滑） | mean 4.00 mm | mean 5.10 mm |
| 轨迹跟踪 800 帧（含 WMA 平滑） | mean 5.81 mm | mean 6.64 mm |
| 任务空间（±0.6 rad，n=300） | mean 6.46 mm，42.3% ≤5mm | 9.23 mm，23.8% |
| IK 全域采样（全关节范围，n=300） | mean 22.8 mm，2.7% ≤5mm | 28.8 mm，0.7% |

> ⚠️ **别误读这两档**：上面「任务空间 / 全域采样」两档的绝对位置精度，
> **官方目标函数自己也达不到更高**。同工况实测（官方 URDF + 官方权重 + SLSQP 30 iter）：
> 任务空间档官方 mean **5.75 mm** / 46% ≤5mm，我方 6.97 mm / 41.3%，J 中位 0.0515 vs 0.0521。
> 原因是 `0.1·||q−q_last||²` 在"warm start 离目标远"时**固有地**换来位置精度：
> warm start 偏 0.6 rad 时，该平滑项等效于 ~27 mm 的位置代价（√(0.1·0.6²/50)）。
>
> 也就是说这两档测的是**目标函数的折衷**，不是求解器精度。
> 实机连续遥操时 warm start ≈ 当前关节角 ≈ 目标附近（σ < 0.1 rad），
> 逐点位置差 p95 **0.49 mm** —— 这才是该看的数。
>
> 📌 历史注记：补齐前（缺 reg/smooth 两项）这两档数字更"漂亮"（任务空间 0.09 mm / 100%），
> 但那是因为**没在解官方问题** —— 它牺牲姿态硬贴位置，同目标下 J 反而更高（见下节「与官方的一致性验证」）。

### 与官方的一致性验证（两套独立方法，`sim/check_ik_vs_official.py` / `sim/check_ik_stress.py`）

```bash
$PY sim/check_official_parity.py a5      # 关节链参数 vs 官方 URDF（24 项零偏差）
$PY sim/check_ik_vs_official.py a5       # 同批目标：官方目标函数(SLSQP) vs 我方 DLS
$PY sim/check_ik_stress.py a5            # 随机目标 + warm-start 偏离扫描
```

| 项 | 结论 |
|---|---|
| 关节链 origin/rpy/axis/限位 | **24/24 项零偏差**（A5 10 项 / A7 14 项） |
| 目标权重 | 50 / 0.5(A5)·1.0(A7) / 0.02 / 0.1 —— 与官方 `opti.minimize` 逐项同 |
| `q_last` 语义 | = warm start = 上层传入的当前关节角（官方 `current_lr_arm_motor_q`） |
| FK | 与官方 pinocchio 差 6e-11 m |
| EE 手安装偏移 | A5 wrist_roll+0.20 / A7 wrist_yaw+0.05 |
| 迭代预算 | 30（= 官方 `ipopt.max_iter`） |

**warm-start 偏离扫描**（σ = 每关节随机偏移；力臂 0.5 m 折算）——我方与官方的位置差 p95：

| σ (rad) | ≈腕部偏移 | A5 | A7 |
|---|---|---|---|
| 0.02 | 10 mm | 0.47 mm | 0.49 mm |
| 0.10 | 50 mm | 0.49 mm | 0.34 mm |
| 0.30 | 150 mm | 0.54 mm | 0.45 mm |
| 0.60 | 300 mm | 0.67 mm | 0.65 mm |
| 1.20 | 600 mm | 302 mm ⚠️ | 175 mm ⚠️ |

即 warm start 偏到 300 mm（单帧不可能的量级）仍与官方一致；只有到 600 mm 才分叉。


### PICO 报文端到端（3s @30Hz，wave 幅度 0.2 rad）

一键复现（`check_pico_e2e.py` 固化 README 里原先只写了描述的第 3 步）：

```bash
$PY sim/check_pico_e2e.py --variant a5           # wave（关节空间正弦）
$PY sim/check_pico_e2e.py --variant a5 --pose-mode circle
```

| 口径 | A5 位置 mean / max | A5 姿态 mean / max | A7 位置 mean / max |
|---|---|---|---|
| `--raw`（跳过 WMA 平滑） | **0.57 mm / 1.48 mm** | 0.58° / 2.36° | 0.64 mm / 2.35 mm |
| `ik.solve`（含 WMA，= 实机口径） | **10.29 mm / 17.27 mm** | 2.09° / 3.23° | 7.37 mm / 10.04 mm |

> `--raw` 档 0.57 mm（旧版为 0.108 mm）就是**官方目标函数的最优解残差**：位置与姿态按
> 100:1 加权后位置不再精确命中，官方同样如此（见上表 J 对照）。这是"按官方走"的代价。
> 两档之差 ≈ **WMA 平滑的滞后**（`weights=[0.4,0.3,0.2,0.1]` 约 1 帧群延迟，
> 跟随误差 ≈ 手速 × 滞后时间）。与官方 `WeightedMovingFilter` 一致，不是缺陷。
>
> `--pose-mode circle`（任务空间圆周、姿态固定）在 A5 上 raw 残差 ~38 mm —— **旧版同为
> 38.6 mm**，属 5-DOF 臂跟踪固定姿态的固有折衷，与本次改动无关。


### 安全判据场景验证

判据已集中到 `../example/r1/high_level/r1_pico_safety_policy.h::decideDisposition()`，
优先级 **急停 > 回零 > 保持 > 遥操**。`pico_pipeline_test` 与主程序**共用**该策略，
因此下面的结论就是 `r1_dual_arm_loco.cpp --pico` 分支的结论。

```bash
$PY sim/safety_scenarios.py      # 一键回归全部场景
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
