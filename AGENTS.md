# AGENTS.md — RobotProject 项目记忆

> 本文件供 AI 助手快速了解本项目。**本文件位于 `unitree_sdk2/`（是 git 仓库）**，
> 会话工作目录 = `/Users/plf/Project/RobotProject/unitree_sdk2`。
> 父目录 `RobotProject/` 本身**不是** git 仓库，还放着无关的 `tl_vision_grasp_ws/`（别动）。
>
> **深度资料指针**（细节不在本文件）：
> - **启动遥操：最简命令流程** → `example/r1/high_level/R1_TELEOP_QUICKSTART.md`（**先看这个**，只有命令）
> - 上机指令 Runbook（多判据/排障二分表/红线） → `example/r1/high_level/R1_TELEOP_RUNBOOK.md`
> - 完整上机手册（原理、日志解读、网络四方案、故障二分） → `example/r1/high_level/R1_TELEOP_STARTUP.md`
> - 背包/游标/灵巧手普查档案 → `example/r1/high_level/R1_BACKPACK_ARCHITECTURE.md`
> - 算法/参数与官方对照 → `example/r1/high_level/R1_CONTROL_ALGORITHM.md`
> - 逐日工作记忆 → `.workbuddy/memory/`（**已 gitignore**，`MEMORY.md` 是长期硬规则）
> - AI 技能库 → `~/.workbuddy/skills/`（`r1-edu-backpack-deploy` 等，按需加载）

## 1. 项目目标

用 **PICO 头显（OpenXR App "PICOHandLink"，UDP-JSON）** 遥操作 **宇树 Unitree R1 上半身（双臂 + 底盘）**，
并保留数据采集/二次开发能力。核心实现是 C++，运行在 `unitree_sdk2` 之上。

机器人：**R1-EDU**（26 DOF，带"开发计算单元/算力背包"，Jetson Orin，aarch64，`192.168.123.164`）。
实机手臂型号 **A5**（`--variant a5`）；A7 未开放。

## 2. 三台机器（别混，最容易出事的一节）

| 角色 | 地址 | 说明 |
|---|---|---|
| **PC1** 运控计算单元 | `192.168.123.161` | **禁 SSH**，全端口不通 **≠ 离线**（禁 ICMP，别用 ping/nc 判）。**绝不在此部署**。 |
| **PC2 = 算力背包** | `192.168.123.164` | 用户 `unitree`，Ubuntu 20.04 + JetPack 5 + aarch64，内核 `5.10.104-tegra`。唯一物理网卡 **`eth10`**。日志里的 `pc4` 就是它。**唯一部署目标**。 |
| **测试 VM**（Parallels，本机） | `10.211.55.6` | 用户 `plf-virtual`，Ubuntu 22.04，cmake 3.22 / g++ 11.4。⚠️ **与背包工具链不是一套**（背包 cmake 3.16 / g++ 9.4）⇒ VM 上编过 **不代表**背包上编得过。 |

- 凭据只经环境变量传入（VM 用 `VM_PASS`，背包用 `BACKPACK_PASS`）。**禁止**写进脚本/日志/配置/提交
  —— 09-22 真实事故：本文件正文写出过 VM 密码，推送前扫描才发现。
- 测试 VM **只走密码认证，不推公钥、不配免密**（用户已定）。
- **认证间歇失败**：同脚本同密码时而全败 —— 服务端抽风，**重试 2~5 次**，别一失败就怀疑密码。

## 3. 目录结构

```
RobotProject/
├── unitree_sdk2/                  ← 主工作仓库（origin=官方只读 / fork=sorrowfeng，本地有未提交改动）
│   ├── AGENTS.md                  ← 本文件
│   ├── sim/                       ← macOS 侧仿真/校验工具（见 §4.1，不入 CMake 构建）
│   ├── .workbuddy/                ← 逐日工作记忆（gitignore）
│   ├── submodules/
│   │   ├── hand_control_ws/       ← sorrowfeng/hand_control_ws（灵巧手 ROS 2 工作区）
│   │   └── xr_teleoperate/        ← unitreerobotics/xr_teleoperate（官方 Python 摇操，**仅作参考**）
│   └── example/r1/high_level/     ← **我们的自定义代码，绝大多数是 untracked**
└── tl_vision_grasp_ws/            ← 与本项目无关，别动
```

**关键：实际代码在 `unitree_sdk2/example/r1/high_level/`。**

### 3.1 自定义文件（我们维护）

| 文件 | 作用 |
|---|---|
| `r1_dual_arm_loco.cpp` | 主程序：LocoClient + ArmSdk + PICO UDP；target `r1_dual_arm_loco_skeleton` |
| `r1_arm_manual.cpp` | **双臂关节手动调试工具**（交互式逐关节）；target `r1_arm_manual`。与主程序**不可同跑**（互抢 `rt/arm_sdk`） |
| `r1_arm_controller.h` | 双臂控制器：订阅 `rt/lowstate`，250Hz 发 `rt/arm_sdk`（或 `rt/lowcmd`），CRC、速度限幅、权重渐变 |
| `r1_arm_ik.h` | R1 A5/A7 正运动学 + DLS 数值 IK + WMA 平滑（移植自官方 `robot_arm_ik.py`） |
| `r1_pico_udp.h` | PICOHandLink UDP-JSON v3 接收与解析 |
| `r1_pico_safety_policy.h` | **处置判据单一来源**：`decideDisposition()`，优先级 急停 > 回零 > 保持 > 遥操 |
| `r1_xr_pose_alignment.h` | OpenXR → R1 腰部/骨盆系对齐 |
| `r1_hand_interface.h` | 手驱动**接口**（`HandDriver`；默认 `NullHandDriver` 只打印） |
| `r1_hand_canfd_bridge.h` | **灵巧手 CANFD 桥驱动（实际在用）**：把遥操手部位置经 UDP 转发给 `hand_bridge.py`。见 §8 |
| `r1_hand_dds.h` | 灵巧手 DDS 驱动（三家厂商预设）。**已弃用仅留参考** —— 本项目实际用的手不在那三家之列 |
| `r1_tool.cpp` | 官方全部服务 CLI 测试工具；target `r1_tool` |
| `tests/test_pico_parse.cpp` | 不接机器人的 PICO 报文解析/判据单测 |

`example/r1/CMakeLists.txt` 被修改：新增 `r1_dual_arm_loco_skeleton`、`r1_tool`、`r1_arm_manual` 三个 target。

### 3.2 脚本

| 脚本 | 作用 |
|---|---|
| `scripts/build.sh [target]` | 配置 + 编译（默认 `r1_dual_arm_loco_skeleton`，`all` 编译全部）。⚠️ 只做依赖自检，**不会调 apt** |
| `scripts/run_tests.sh` | 编译并运行 PICO 解析/判据单测 |
| `scripts/deploy.sh` | 本地改 → rsync 到 **测试 VM** → 远程 build + test |
| `scripts/deploy_backpack.sh` | 本地改 → rsync 到 **算力背包 PC2** → 远程 build + test。`--incremental` / `--sync-only`（**要求"部署后不执行任何操作"就用它**）/ `--no-test` |
| `scripts/_vm_ssh.exp`, `_vm_rsync.exp` | 上述两个 deploy 内部用（expect 密码登录） |
| `scripts/r1_tool.py` | 官方全功能交互菜单（loco/msc/state/audio/config/lowstate） |
| `scripts/loco_cli.py`, `scripts/loco.sh` | 仅运控（loco）的快捷封装；`r1_tool`(C++) 被 r1_tool.py 调用 |
| `scripts/backpack_ap.sh` | **背包自己当 AP**（`check`/`chips`/`start`/`status`/`stop`，走 NM，**不装 hostapd**） |
| `scripts/backpack_wifi.sh` | 背包当客户端连外部 AP（与上一个依赖同一块 USB 网卡，别同时用） |
| `scripts/hand_modbus_probe.py` | 灵巧手 Modbus 探针（适配新手用） |
| `scripts/pico_sim_sender.py` | **假装头显**的模拟发送器，真机排练用 |
| `scripts/pico_udp_relay.py` | **方案 C2 单向 UDP 中继**（PICO → Mac → 有线 → 背包）。零 root / 零 NAT；启动时自动打印 PICO 该填哪个地址 |
| `scripts/r1_hand/` | **灵巧手整套（自包含运行目录）**：封装 / 桥 / 测试 / 统一入口 + vendor 依赖（`canfd_lib.py`、`gsusb_canfd/`、pyusb 1.2.1、udev 规则）+ `README.md`。**与背包 `~/r1_hand/` 一一对应**，rsync 即部署。详见 §8 |
| `scripts/hand_rs485_bringup.py` | 灵巧手 **RS485** 打通脚本（**备用路线**，现走 CANFD） |

## 4. 开发工作流（务必遵守）

用户在 macOS 上编辑；**macOS 不能编译**（SDK 只带 Linux `lib/{x86_64,aarch64}/libunitree_sdk2.a`，
且代码用 Linux 专用头文件如 `raps/inet.h`、`sys/socket.h`）。所有编译都在 aarch64 上做。

### 4.1 本机（先跑，最快）
```bash
./sim/check_syntax.sh          # 3 目标应全 0 error（用 sim/shim 垫掉 Linux 专有类型，仅 -fsyntax-only）
./sim/build_probe.sh           # 产出 sim/build/{ik_probe,pico_pipeline_test,test_pico_parse}
./sim/build/test_pico_parse    # 解析/判据单测
$PY sim/safety_scenarios.py    # 7 个安全场景
$PY sim/check_pico_e2e.py      # 合成报文 → 全链路 → 独立 FK 比对
```
`$PY` = 带 mujoco/numpy 的 python：`/Users/plf/.workbuddy/binaries/python/envs/default/bin/python`（已实测可用）。
**改过 `high_level/` 的头文件后必须重跑 `build_probe.sh`** —— 探针不在 CMake 构建里，不会自动更新。

### 4.2 上机（背包 / VM）
```bash
BACKPACK_PASS=<密码> .../scripts/deploy_backpack.sh --incremental   # 日常
BACKPACK_PASS=<密码> .../scripts/deploy_backpack.sh --sync-only     # 只同步，绝不编译/运行
VM_PASS=<密码>       .../scripts/deploy.sh                          # 测试 VM
```
- 增量同步 **`example/r1/` 整个目录** —— 范围必须 ≥ `example/r1/CMakeLists.txt` 的可见范围
  （其 target 引用 `high_level/`、`low_level/`、`audio/` 三处；只同步一部分会在 cmake 配置阶段报
  「找不到源文件」）。改别的示例目录用 `--example`；改 `include/`/`lib/`/`thirdparty/` 用 `--full`。
  rsync **不带 `--delete`**。
- 默认只编 `r1_dual_arm_loco_skeleton`；编别的 target 需在远端手动 `scripts/build.sh <target>`。

## 5. 官方服务与运控层次（重要认知）

R1 SDK 对外只有这几个服务（对比 upstream main 一致）：

| 服务 | 客户端 | 内容 |
|---|---|---|
| `sport` | `r1::LocoClient` | GetFsmId/Mode、SetFsmId、SetVelocity、SetSpeedMode；封装 Damp(1)/StandUp(4)/Start(811)/ZeroTorque(0)/StopMove/Move |
| `voice` | `r1::AudioClient` | TTS、Get/SetVolume、PlayStream、LedControl；ASR 订阅 `rt/audio_msg` |
| `motion_switcher` | `b2::MotionSwitcherClient` | CheckMode、SelectMode、ReleaseMode、Get/SetSilent |
| `robot_state` | `b2::RobotStateClient` | ServiceList、ServiceSwitch、SetReportFreq、LowPowerSwitch/Status、GetPkgVersion |
| `config` | `b2::ConfigClient` | Get/Set/Del/Meta |

遥控器上其它按键基本都是 `SetFsmId` 的不同取值（2/3/4/811…）与速度档位，不是独立 API。
FSM：`0` 零力矩（完全失力/瘫软）· `1` 阻尼（安全态）· `4` **锁定站立** · `811` **走跑（主运控）**。
⛔ **切 811 必须按顺序 `damp → stand → start`，不能直接 `start`**（用户实测确认）。
**每次切完跟一条 `status` 复验** —— `ret=0` 只说明 service 调用成功，不代表状态真切过去了。

控制层次（不要混淆）：

```
LocoClient ──▶ 机载运控 ai_sport（站立/行走/阻尼/零力矩）
遥操程序   ── rt/arm_sdk ──▶ 运控管腿，双臂被并行接管（安全，本项目主路径）
底层开发   ── rt/lowcmd ───▶ 必须先 MotionSwitcher ReleaseMode 释放运控（危险）
```

## 6. 已修复（勿回退）

- 退出卡死：主程序 stdin 线程由阻塞 `getline` 改为 `poll`+`read`。
- 头/腰回零跳变：`setWeight(1.0)` 必须在 `homeHeadAndWaist` **之前**（对齐官方）。
- A7：默认 `motion_mode` 不支持 `rt/arm_sdk`，`--variant a7` 未加 `--lowcmd` 会直接报错；`--variant` 有校验。
- 编译错误：`R1ArmController(Variant, const Params& = {})` 中嵌套类默认成员初始化器不可用，
  已改为委托构造 `R1ArmController(Variant)`。**不要改回 `= {}`**。
- 安全判据顺序错误：`return_zero` 分支曾排在 `if (!safe) continue;` 之后（死代码，一键回零永不触发）；
  `emergency_stop_latched` 未纳入判据。已集中到 `r1_pico_safety_policy.h::decideDisposition()`，
  优先级 **急停 > 回零 > 保持 > 遥操**，由 `tests/test_pico_parse.cpp` 覆盖。
  **不要再把判据写回主循环的 if/else 链**。
- **SIGHUP/SIGTERM 未接**（09-23）：SSH 掉线/关窗口时进程被立即杀死，来不及回零交还 ⇒ 双臂冻结。
  现 `SIGINT`/`SIGTERM`/`SIGHUP` 走同一条清理路径。
- ⚡ **速度限幅必须作用在「指令」上**（指令积分 `cmd += clamp(目标-cmd, ±vel*dt)`），
  **不能**写成「实测 + 每帧一小步」—— 后者在"偏差力矩 < 关节静摩擦"时**永久死锁**
  （09-23 真机实测：`--vel 2` 纹丝不动、`--vel 30` 能动）。两处同源，**别只改一处**：
  `r1_arm_manual.cpp::publishOnce()` 与 `r1_arm_controller.h::clipTargets()`
  （后者另有 `arm_cmd_lag_max` 兜底，防卡滞解除瞬间暴冲）。
- **首包不再切 FSM**（09-29）：原先首个可执行包自动 `StandUp()`（811→4）是**偏离官方**的一步
  —— 官方 `xr_teleoperate` 从不切 FSM（要求事先在运控态、只支持 `Regular mode`）。
  现只解锁底盘速度闸门，旧行为降级为 `--auto-stand`（**默认关**）。
  ⇒ **起程序前机器人必须已在 811**。
- **`operator_mode` 缺失可诊断**（09-29）：新增 `PicoTeleopPacket::operator_mode_present`，
  区分「键缺失」（= 装错 App）与「操作者真发 stop_signal」—— 两者现象都是每包 `Damp()`、腿一直软。
  同时 `[pico] first packet` 增打 `sdk` 与 `operator_mode`。
- `disp == kTeleop` 但 `pose` 缺失时**静默不动**（与历史"权重 100 却什么都不动"无法区分）
  ⇒ 已加 **1 Hz 限速告警**。

## 7. 待办（尚未实现）

- 头/腰遥操跟随未实现（仅启动回零）。⚠️ App 默认**不发** `robot_control.head`/`body.waist`，
  除了改代码还要在 App「发送内容」页勾选；且 `WaistRoll(12)` 写不生效（运控独占）。
- 灵巧手：**已接入主程序**（CANFD 桥，见 §8）。`r1_hand_dds.h`（DDS 厂商桥那条路）
  **已弃用仅留参考** —— 那三家（BrainCo/Linker/Inspire）不是本项目实际用的手。
- `r1_xr_pose_alignment.h` 腰部偏移 `+0.15x/+0.45z` 为常量，上机需标定。
- C++ IK 用 DLS 最小化官方的**完整四项**目标（pos/rot/正则/平滑），已与官方 IPOPT 逐点对照
  （`sim/check_ik_vs_official.py`，A5 最大偏差 2.79 mm）；仍非逐位一致。
  实机 ~22 mm 残差是 A5 五自由度的结构固有值，**别当回归**。
- **PICO → 背包 网络通道 = 方案 C2**（`scripts/pico_udp_relay.py`：PICO → Mac → 有线 → 背包），
  **2026-10-08 已端到端验证**（背包 tcpdump 抓到 10/10 包）。背包内置无 WiFi；
  外接 **AR9271 USB 卡已实测否决**（一关联就**内核静默挂死 + 120 s 看门狗重启**，连续 4 次）。
  四条路线全表见上机手册 §5。
- ✅ **09-29 那批改动已于 2026-10-08 同步上背包并编译通过**（单测全过；特征串 `保持当前 FSM`
  在产物中 = 1，`bin` mtime 晚于 `src`）。**这笔欠账已消**。
- ⚠️ **已知无害但扰民的现象**：**UDP 一停就按 30 Hz 刷 `[loco] Damp.`**。
  根因：`tryGet()` 收到过至少一个包后就**永远返回最后一个包**，`rx_age_ms` 单调增长
  ⇒ 每帧都判「掉包 → kHold → 触发阻尼」⇒ `damp_cmd` 每帧置位。Damp 幂等，**重复发无害**，
  只是**没做边沿检测**（急停那条有 `estop_active` 去重，掉包这条没有）。
  **退出不受影响**：stdin 线程独立于刷屏，直接打 `q` + 回车即可。恢复发送端即停止刷屏。

**设计选择（不是缺陷）：** 掉包（>600 ms）或急停触发阻尼后**不自动重新站立**，
需操作者显式 `s`（PICO 站立键）。与官方 `xr_teleoperate` 一致，避免链路抖动后自行起身。

## 8. 灵巧手（CANFD 直控，2026-10-09 落地）

**实际硬件**：LHandPro **`DH116S-L000-A1`(左) / `DH116S-R000-A1`(右)**，各 6 自由度。
USB-CANFD 适配器 **`a8fa:8598`**（Com Equipment "CANFD Analyser"，gs_usb 用户态协议），
两只手挂**同一条总线**，靠 **node id** 区分（默认 1 / 2）。

### 8.1 为什么弃用厂商 SDK（硬性 ABI 不兼容）
`libLHandProLib.so` 是在 **Ubuntu 22.04 / GCC 11** 上编的，要求
`GLIBCXX_3.4.30` + **`GLIBC_2.34`**（还带 `GLIBC_2.32`）；而背包是 **Ubuntu 20.04 / glibc 2.31**、
`libstdc++` 只到 `GLIBCXX_3.4.28` ⇒ **`ctypes.CDLL` 直接失败**
（`version 'GLIBCXX_3.4.29' not found`）。**换 libstdc++ 也救不了** —— glibc 是整个 C 运行时。
⇒ 手的 CAN 帧协议极简单，**直接发原始帧**，完全绕开 SDK。

### 8.2 协议（由厂商示例帧逐字节解出，脚本内有自检）
```
CAN ID    = 0x500 + node_id
14B 数据  = <cmd:1> <轴数:1=0x06> <6×2 字节小端数值>
  cmd 0x01 模式：0x0120 使能 · 0x0425 回零 · 0x0020 切回位置模式
  cmd 0x02 位置（0..10000，总行程系数 10000 ⇒ 0x03E8=1000=10% 行程）
  cmd 0x03 速度（0x07D0=2000 ≈ 0.5 s 走完全行程）
  cmd 0x04 电流上限（0x03E8=1000 = 100% 额定，千分比）
反馈  = 0x480+node（控制）/ 0x580+node（SDO）
```
⚠️ **`0x500+node` 是适配器把自己发出去的帧"回环"回来，不是手的反馈** ——
两者必须分开统计，否则会得出"手在应答"的假象（我第一版就踩了）。
⚠️ **`00 02 50 01` 是「开启异步反馈上报」**，对每个节点各发一条。
**运动本身不需要它**（真机 ID 1 未开反馈也能动）；只有想读状态/确认"手在线"时才发。
发了之后设备约 **1000 帧/秒**持续上报。

### 8.3 架构：C++ ↔ Python 桥
C++ 侧要用 gs_usb 就得在背包上编译 `gsusb-canfd` 的 C++ 库；而 Python 那套
（`canfd_lib` + `gsusb_canfd` + pyusb 1.2.1）已真机验证且**离线部署完成**。故拆成桥：

```
r1_dual_arm_loco.cpp ──UDP 127.0.0.1:9998──> hand_bridge.py ──CANFD──> 两只手
   （--hand canfd）        "POS l0..l5 r0..r5 mask"      独占适配器
```

- **C++**：`r1_hand_canfd_bridge.h` 的 `CanfdBridgeHandDriver` 实现现有 `HandDriver` 接口，
  只把 6+6 个关节目标（0..10000）组一行文本发 UDP。
- **Python**：`scripts/r1_hand/hand_bridge.py` **启动时**走完初始化
  （使能 → 回零(等 `--home-wait`) → 位置模式 → 速度 → 电流），之后**位置直接映射**，零换算。
- ⚠️ **安全语义**：主循环只在**遥操分支**传 `mode=kPose`；急停/掉包/保持/回零都传 `kIdle`。
  驱动**只在 kPose 时发送** ⇒ 手保持在最后位置不动（与双臂"保持"一致）。
  桥侧另有 `--timeout`（默认 1 s）没收到新位置就停止下发。

### 8.4 用法
```bash
# ① 先起桥（会初始化两只手，回零约 4~5 s/只）
~/r1_hand/hand_canfd.sh hand_bridge.py --left 1 --right 2
#   看到 "[bridge] ✅ 就绪：..." 再往下走
# ② 另一个 tmux window 起遥操
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --hand canfd
```
不加 `--hand` 时行为**完全不变**（默认 `null`，只打印 6 路 raw）。

### 8.5 坑（都是实测踩到的）
1. **回环 ≠ 反馈**（见 8.2）——按 CAN ID 分开统计。
2. **逐帧日志刷屏**：开反馈后 ~1000 帧/秒。`LHandCanfd(log_frames=False)` 关掉；
   生命周期日志（扫描/连接/初始化/关闭）不受影响。
3. **stdout 块缓冲**：非 TTY（后台/管道/ssh）下 Python 输出被缓冲，**看不到任何进度**，
   极易误判成"没跑起来"。统一走 `scripts/r1_hand/hand_canfd.sh`（内部 `python3 -u`）。
4. **适配器 USB 不稳**：`a8fa:8598` 会反复 disconnect→重枚举（先 `a8fa:0008` 后 `8598`，
   是 Artery MCU 的 bootloader→firmware 切换）。掉了要重插。
5. **非 root 访问**：需 udev 规则 `/etc/udev/rules.d/hcanbus.rules`
   （`a8fa`/`8598` → `MODE=0666`），否则打不开 USB 设备。
6. **离线依赖**：背包无网、apt 坏。`~/r1_hand/` 里已放好
   `canfd_lib.py` + `gsusb_canfd/` + `usb/`(pyusb **1.2.1**，**1.3.x 要求 Python≥3.9 用不了**)；
   **pyusb 是纯 Python，直接拷 `usb/` 目录即可，不需要 pip**。
7. **macOS 本地也能测**：需 `DYLD_LIBRARY_PATH` 指向 Homebrew 的 libusb
   （否则 `no USB backend available`），`hand_canfd.sh` 已自动处理。
   `~/r1_hand/` 是本地运行目录，布局与背包一致。

### 8.6 相关文件

**整套在 `scripts/r1_hand/`**（自包含，与背包 `~/r1_hand/` 一一对应，rsync 即部署）：

| 文件 | 作用 |
|---|---|
| `r1_hand_canfd_bridge.h` | C++ 桥驱动（UDP 转发，实现 `HandDriver`）；在主程序源码目录 |
| `r1_hand/hand_canfd.py` | **封装**：`LHandCanfd` + `build_payload()`（协议常量、回环/反馈分离、`start_feedback()`） |
| `r1_hand/hand_bridge.py` | **桥进程**：独占适配器 + 启动初始化 + 位置直接映射 |
| `r1_hand/check_comms.py` | 只读通讯验证；`--scan` 按节点找总线上的手 |
| `r1_hand/test_hand_canfd.py` | 编码器逐字节自检 → 双手初始化 → 运动 → 统计 |
| `r1_hand/hand_canfd.sh` | 跨平台入口（自动处理 macOS libusb 路径、`-u` 无缓冲；依赖目录按 `$R1_HAND_DIR` → 脚本目录 → `~/r1_hand` 查找） |
| `r1_hand/{canfd_lib.py,gsusb_canfd/,usb/,hcanbus.rules}` | **vendor**（勿改）：厂商封装 + gs_usb 纯 Python 实现 + pyusb 1.2.1 + udev 规则 |
| `r1_hand/README.md` | 该目录的说明：硬件、文件分工、来源/许可、平台差异、三个坑 |

## 9. 红线清单（背下来）

1. ⛔ **绝不 `kill -9`**：权重停在 100 且不再发布 ⇒ 双臂**冻结在最后一帧姿态**。真发生了就重跑程序再正常退出。
2. ⛔ **不与 `r1_arm_manual` 同时跑**：两个程序互抢 `rt/arm_sdk`。
3. ⛔ **`CYCLONEDDS_URI` 必须为空**：非空指向写死 `eth0` 的 xml ⇒ DDS 静默超时（现象同"脚本坏了"）。
   tmux 里见到 `ros:foxy(1) noetic(2) ?` **按回车，永不按 1**。
4. ⛔ **起程序前机器人必须已在 811**；每次切 FSM 后必须 `status` 复验。
5. ⚠️ **启动/退出前清空臂展**：启动时头/腰真实回零 3 秒；退出时双臂主动摆回零位（先阻尼后回零，约 2~7 s）。
6. ⛔ **背包 apt 是坏的、无外网 DNS**：装包只能「本机下 `.deb` → `scp` → `sudo dpkg -i`」。
   **别跑 `apt --fix-broken install`**（会真去升 systemd/python3.8）。
7. ⏰ **背包时钟会漂移**（记录里 09-23 慢 ~90 s，10-08 实测已慢到 **~200 s**）⇒ `make: Clock skew detected`
   会**静默跳过重编**（产物还是旧的）。**改代码后先校时**（`$(date)` 必须在 **Mac** 展开）：
   `ssh unitree@192.168.123.164 "sudo date -s '$(date '+%Y-%m-%d %H:%M:%S')'"`
   验证"新代码进没进产物"用 `grep -ac "中文特征串" build/bin/<target>`，
   **别用 `strings | grep 中文`**（永远 0，会误判）。
8. ⚠️ **危险运控操作**（zero-torque、move、msc release）保持二次确认与安全距离提示；
   命令式切 FSM 用 `loco.sh`（默认带 YES 确认），`r1_tool.py` 的脚本化形式**无确认**。

## 10. 约定

- 全程中文交流/注释（沿用现有风格）。
- **未经明确要求不要 git commit / push**；本项目文件多为 untracked。
- 改动后按 §4 流程验证（本机 `sim/` 先行，再上机），再回报结果。
- 若新增 C++ 源文件，记得在 `example/r1/CMakeLists.txt` 加 target。
- 同一文件多次 `Edit` **必须串行**（并行调用会静默丢改动）。
- macOS 上 BSD grep **不支持** `\|` 与 `\b`（当字面量）⇒ 用 `grep -E` 或 ripgrep。已因此误报多次。
- PICO 端 App 源码在本机 `~/Project/PICOProject/PICOHandLink`。**契约问题直接读源码，别猜**。
  实机主 App = `:openxr-app`（`sdk="pico-openxr-single"`）；
  旧 `:app`（`sdk="pico-openxr-bridge"`）**不发 `operator_mode`**，误装后果见 §6。
