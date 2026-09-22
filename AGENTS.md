# AGENTS.md — RobotProject 项目记忆

> 本文件供 AI 助手快速了解本项目。工作目录：`/Users/plf/Project/RobotProject`（**非 git 仓库**）。
> 两个子目录各自是独立的上游 git clone。

## 1. 项目目标

用 **PICO 头显（OpenXR App "PICOHandLink"，UDP-JSON）** 遥操作 **宇树 Unitree R1 上半身（双臂 + 底盘）**，
并保留数据采集/二次开发能力。核心实现是 C++，运行在 `unitree_sdk2` 之上。

机器人：**R1-EDU**（26 DOF，带"开发计算单元/算力背包"，Jetson Orin，aarch64，`192.168.123.164`）。

## 2. 目录结构

> 注：本文件原位于 `RobotProject/` 根目录，现已移入本仓库（`unitree_sdk2/`）。

```
RobotProject/
├── unitree_sdk2/             ← 主工作仓库（remote: unitreerobotics/unitree_sdk2，本地有大量未提交改动）
│   ├── AGENTS.md             ← 本文件
│   ├── submodules/           ← 统一存放 git 子模块
│   │   ├── hand_control_ws/  ← submodule: sorrowfeng/hand_control_ws（灵巧手 ROS 2 工作区）
│   │   └── xr_teleoperate/   ← submodule: unitreerobotics/xr_teleoperate（官方 Python 摇操仓库，仅作参考）
│   └── example/r1/high_level/ ← 我们的自定义代码（untracked）
└── xr_teleoperate/           ← 旧位置副本（已被 submodules/xr_teleoperate 取代，可删除）
```

**关键：实际代码在 `unitree_sdk2/example/r1/high_level/`，均为未跟踪文件（untracked）。**

### 自定义/新增文件（我们维护）

| 文件 | 作用 |
|---|---|
| `example/r1/high_level/r1_dual_arm_loco.cpp` | 主程序：LocoClient + ArmSdk + PICO UDP；CMake target `r1_dual_arm_loco_skeleton` |
| `example/r1/high_level/r1_arm_controller.h` | 双臂控制器：订阅 `rt/lowstate`，250Hz 发 `rt/arm_sdk`（或 `rt/lowcmd`），CRC、速度限幅、权重渐变 |
| `example/r1/high_level/r1_arm_ik.h` | R1 A5/A7 正运动学 + DLS 数值 IK + WMA 平滑（移植自官方 `robot_arm_ik.py`） |
| `example/r1/high_level/r1_pico_udp.h` | PICOHandLink UDP-JSON v3 接收与解析 |
| `example/r1/high_level/r1_xr_pose_alignment.h` | OpenXR → R1 腰部/骨盆系对齐 |
| `example/r1/high_level/r1_hand_interface.h` | 灵巧手驱动接口（默认 `NullHandDriver`，未接实际手） |
| `example/r1/high_level/r1_tool.cpp` | 官方全部服务 CLI 测试工具；CMake target `r1_tool` |
| `example/r1/high_level/R1_CONTROL_ALGORITHM.md` | 算法/参数与官方对照说明 |
| `example/r1/high_level/README_zh.md` | 项目说明文档 |
| `example/r1/high_level/scripts/` | 构建/部署/测试/运控脚本（见下） |
| `example/r1/high_level/tests/test_pico_parse.cpp` | 不接机器人的 PICO 报文解析/对齐单测 |

`example/r1/CMakeLists.txt` 被修改：新增 `r1_dual_arm_loco_skeleton` 和 `r1_tool` 两个 target。

### 脚本

| 脚本 | 作用 |
|---|---|
| `scripts/build.sh [target]` | 配置 + 编译（默认 `r1_dual_arm_loco_skeleton`，`all` 编译全部） |
| `scripts/run_tests.sh` | 编译并运行 PICO 解析/对齐单测 |
| `scripts/deploy.sh` | **本地改 → rsync 到 VM → 远程 build + test**（核心工作流） |
| `scripts/_vm_ssh.exp`, `_vm_rsync.exp` | deploy 内部用（expect 密码登录） |
| `scripts/r1_tool.py` | **官方全功能交互菜单**（loco/msc/state/audio/config/lowstate） |
| `scripts/loco_cli.py`, `scripts/loco.sh` | 仅运控（loco）的快捷封装 |
| `scripts/r1_tool`(C++) | 官方服务 CLI，被 r1_tool.py 调用 |

## 3. 开发工作流（务必遵守）

用户在 macOS 上编辑；**macOS 不能编译**（SDK 只带 Linux `lib/{x86_64,aarch64}/libunitree_sdk2.a`，
且代码用 Linux 专用头文件如 `raps/inet.h`、`sys/socket.h`）。所有编译/测试都在 aarch64 VM 上做。

标准循环：

1. **本地改代码**（在 `unitree_sdk2/example/r1/high_level/` 或 `CMakeLists.txt`）。
2. **同步并验证**：
   ```bash
   VM_PASS=<密码> unitree_sdk2/example/r1/high_level/scripts/deploy.sh          # 增量同步 + build + tests
   VM_PASS=<密码> .../scripts/deploy.sh --no-test                              # 只编译
   VM_PASS=<密码> .../scripts/deploy.sh --example                              # 同步整个 example/
   VM_PASS=<密码> .../scripts/deploy.sh --full                                 # 首次/改了 SDK 核心
   ```
   增量同步 **`example/r1/` 整个目录** —— 范围必须 ≥ `example/r1/CMakeLists.txt` 的可见范围
   （其 target 引用 `high_level/`、`low_level/`、`audio/` 三处；只同步其中一部分会在 cmake
   配置阶段报「找不到源文件」）。改别的示例目录（如 `example/g1/`）用 `--example`；
   改 `include/`、`lib/`、`thirdparty/` 等 SDK 核心用 `--full`。rsync 不带 `--delete`。
3. 编译特定 target：`deploy.sh` 默认只编 `r1_dual_arm_loco_skeleton`；编 `r1_tool` 需在远端手动
   `example/r1/high_level/scripts/build.sh r1_tool`（或 SSH 执行）。

### 远程测试环境（本地 Parallels 测试机）

- `VM_HOST=10.211.55.6`，`VM_USER=plf-virtual`，`VM_PORT=22`，密码通过环境变量 `VM_PASS` 传入。
- 项目路径：`~/RobotProject/unitree_sdk2`。
- ⚠️ 这是**本机 Parallels 上的临时测试 VM**。密码只经环境变量 `VM_PASS` 传入，**不写入仓库任何文件**。不要用于生产。
- 依赖已装好：`cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev`。

## 4. 官方服务与运控层次（重要认知）

R1 SDK 对外只有这几个服务（对比 upstream main 一致）：

| 服务 | 客户端 | 内容 |
|---|---|---|
| `sport` | `r1::LocoClient` | GetFsmId/Mode、SetFsmId、SetVelocity、SetSpeedMode；封装 Damp(1)/StandUp(4)/Start(811)/ZeroTorque(0)/StopMove/Move |
| `voice` | `r1::AudioClient` | TTS、Get/SetVolume、PlayStream、LedControl；ASR 订阅 `rt/audio_msg` |
| `motion_switcher` | `b2::MotionSwitcherClient` | CheckMode、SelectMode、ReleaseMode、Get/SetSilent |
| `robot_state` | `b2::RobotStateClient` | ServiceList、ServiceSwitch、SetReportFreq、LowPowerSwitch/Status、GetPkgVersion |
| `config` | `b2::ConfigClient` | Get/Set/Del/Meta |

遥控器上其它按键基本都是 `SetFsmId` 的不同取值（2/3/4/811…）与速度档位，不是独立 API。

控制层次（不要混淆）：

```
LocoClient ──▶ 机载运控 ai_sport（站立/行走/阻尼/零力矩）
遥操程序   ── rt/arm_sdk ──▶ 运控管腿，双臂被并行接管（安全，本项目主路径）
底层开发   ── rt/lowcmd ───▶ 必须先 MotionSwitcher ReleaseMode 释放运控（危险）
```

## 5. 已修复 / 已知问题

**已修复（勿回退）：**
- 退出卡死：`r1_dual_arm_loco.cpp` 的 stdin 线程由阻塞 `getline` 改为 `poll`+`read`。
- 头/腰回零跳变：`setWeight(1.0)` 必须在 `homeHeadAndWaist` **之前**（对齐官方）。
- A7：默认 `motion_mode` 不支持 `rt/arm_sdk`，`--variant a7` 未加 `--lowcmd` 会直接报错；`--variant` 有校验。
- 编译错误：`R1ArmController(Variant, const Params& = {})` 中嵌套类默认成员初始化器不可用，
  已改为委托构造 `R1ArmController(Variant)`。**不要改回 `= {}`**。
- 全量语法校验基线：`r1_dual_arm_loco.cpp` / `r1_tool.cpp` 在 clang 下 0 error。
  **本机可复现**：`./sim/check_syntax.sh`（用 `sim/shim/mac_syntax_compat.h` 垫掉
  Linux 专有类型后才轮得到校验我们的代码；垫片只用于 `-fsyntax-only`，不参与构建）。
- 安全判据顺序错误：`return_zero` 分支曾排在 `if (!safe) continue;` 之后，而
  `safeToExecute()` 要求 `active_stream`，导致该分支是**死代码**（一键回零永不触发）。
  同理 `emergency_stop_latched` 未纳入判据，PICO 端只置闩锁位时仍会继续执行运动。
  已修复：判据集中到 `r1_pico_safety_policy.h::decideDisposition()`，优先级
  **急停 > 回零 > 保持 > 遥操**，并由 `tests/test_pico_parse.cpp` 覆盖顺序。
  **不要再把判据写回主循环的 if/else 链**。

**待办（尚未实现）：**
- 头/腰遥操跟随未实现（仅启动回零）。
- 灵巧手仅 `NullHandDriver`，不驱动实际手。
- `r1_xr_pose_alignment.h` 腰部偏移 `+0.15x/+0.45z` 为常量，上机需标定。
- C++ IK 用 DLS 最小化官方的**完整四项**目标（pos/rot/正则/平滑），已与官方
  IPOPT 逐点对照（`sim/check_ik_vs_official.py`，A5 最大偏差 2.79 mm）；仍非逐位一致。

**设计选择（不是缺陷）：** 掉包（>600 ms）或急停触发阻尼后**不自动重新站立**，
需操作者显式 `s`（PICO 站立键）。与官方 `xr_teleoperate` 一致，避免链路抖动后自行起身。

## 6. 部署目标

最终部署到 R1-EDU 算力背包（aarch64），SSH+tmux 运行（无显示器），PICO UDP 发往
`192.168.123.164:9999`，`sudo ufw allow 9999/udp`。**不要在运控计算单元（PC1）上部署**。
交叉架构：x86 编译物不能上背包，必须在 aarch64 重编。

## 7. 约定

- 全程中文交流/注释（沿用现有风格）。
- **未经明确要求不要 git commit / push**；本项目文件多为 untracked。
- 改动后按第 3 节流程在 VM 上编译验证，再回报结果。
- 危险运控操作（zero-torque、move、msc release）保持二次确认与安全距离提示。
- 若新增 C++ 源文件，记得在 `example/r1/CMakeLists.txt` 加 target。
