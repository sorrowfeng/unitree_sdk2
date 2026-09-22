# R1 上半身 PICO 遥操作（C++ / unitree_sdk2）

把官方 `xr_teleoperate`（Python）的 R1 双臂遥操链路移植成 C++，并接入自研
PICOHandLink（OpenXR App）通过 UDP-JSON 下发的位姿/手柄/手部数据。运行在
`unitree_sdk2` 上，无需相机、无需 teleimager/PC2 图像服务。

## 文件说明

| 文件 | 作用 |
|---|---|
| `r1_dual_arm_loco.cpp` | 主程序：LocoClient + ArmSdk 双线程 + PICO UDP 接入 |
| `r1_arm_controller.h` | 双臂控制器：订阅 `rt/lowstate`，250 Hz 发布 `rt/arm_sdk`（或 `rt/lowcmd`），CRC、速度限幅、权重渐变 |
| `r1_arm_ik.h` | R1 A5/A7 正运动学 + 阻尼最小二乘（DLS）IK + WMA 平滑，移植自 `robot_arm_ik.py` |
| `r1_pico_udp.h` | PICOHandLink UDP-JSON v3 接收与解析（POSIX UDP） |
| `r1_xr_pose_alignment.h` | OpenXR → R1 腰部/骨盆系对齐（移植 `tv_wrapper.py`） |
| `r1_hand_interface.h` | 灵巧手驱动接口（预留，默认 `NullHandDriver`） |
| `R1_CONTROL_ALGORITHM.md` | 控制算法/参数与官方对照说明 |
| `scripts/build.sh` | 一键配置 + 编译 |
| `scripts/run_tests.sh` | 编译并运行不接机器人的解析/对齐测试 |
| `tests/test_pico_parse.cpp` | 上述测试源码 |

## 依赖

Ubuntu 20.04 / 22.04，x86_64 或 aarch64：

```bash
sudo apt install -y cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev
```

## 编译

```bash
# 只编译遥操作主程序
example/r1/high_level/scripts/build.sh
# 或显式：
cd unitree_sdk2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) r1_dual_arm_loco_skeleton
```

产物：`build/bin/r1_dual_arm_loco_skeleton`。

> 交叉架构提示：x86 上编的二进制不能拿到 aarch64（算力背包）上跑。部署到背包时，
> 请在背包本机（或对等的 aarch64 环境）重新编译。

## 开发流程：本地改 → 同步到 VM 验证

推荐流程：在本地（如 macOS）编辑，然后一键同步到 aarch64 验证环境并远程编译+测试，
保证两边代码一致：

```bash
# 首次：全量同步整个 SDK（排除 .git / build）
VM_PASS=<VM密码> example/r1/high_level/scripts/deploy.sh --full

# 日常：增量同步 example/r1 整个目录，远程 build + run_tests
VM_PASS=<VM密码> example/r1/high_level/scripts/deploy.sh

# 上游改动了其它示例目录（如 example/g1/）
VM_PASS=<VM密码> example/r1/high_level/scripts/deploy.sh --example

# 只编译不测试
VM_PASS=<VM密码> example/r1/high_level/scripts/deploy.sh --no-test
```

可用环境变量覆盖目标机：`VM_HOST`（默认 `10.211.55.6`）、`VM_USER`（`plf-virtual`）、
`VM_PORT`（`22`）、`VM_DIR`（`~/RobotProject/unitree_sdk2`）。

> **同步范围必须 ≥ CMakeLists 的可见范围。** 增量同步覆盖 `example/r1/` 整个目录：
> 该目录的 `CMakeLists.txt` 里 target 引用了 `high_level/`、`low_level/`、`audio/` 三处源文件，
> 只同步其中一部分会让 cmake 在配置阶段报「找不到源文件」。改 SDK 核心（`include/`、
> `lib/`、`thirdparty/`）用 `--full`。密码仅从 `VM_PASS` 读取，不写入脚本。
> 注意 rsync 不带 `--delete`，VM 上被上游改名后的旧文件会残留（不被 CMakeLists 引用，无害）。

## 测试（不接机器人）

```bash
example/r1/high_level/scripts/run_tests.sh
```

覆盖：完整 `active_stream` 包解析、四元数/欧拉两条姿态重建路径、OpenXR→机器人对齐、
`safe_to_execute=false` / `stop_signal` 不可执行、`emergency_stop_latched` 不可执行、
`return_zero` 可执行（但不等于遥操许可）、畸形 JSON 与 `sequence=0` 拒绝，
以及 `decideDisposition()` 的判据优先级（急停 > 回零 > 保持 > 遥操）。

## 运行

程序需要 DDS 能到达机器人（同一 `192.168.123.x` 网段），PICO 需要能把 UDP 发到本机。

```bash
# 参数：<网卡名>，网卡名用 `ip link` / `ifconfig` 查看
./build/bin/r1_dual_arm_loco_skeleton <网卡> --pico 9999

# 常用参数
#   --variant a5|a7        手臂型号（默认 a5）
#   --pico [port]          PICOHandLink UDP 端口（默认 9999）
#   --pico-frame head_yaw|head_trans|basis   双臂参考系（默认 head_yaw）
#   --pico-source controllers|teleop         位姿来源（默认 controllers）
#   --lowcmd               A7 必需；A5 可选，需先释放机载运控服务
```

非 PICO 演示/调试（stdin）：`v vx vy vyaw` | `s`（站立）| `d`（阻尼）| `t`（停走）| `q`（退出）。

## 官方功能全量测试（二次开发前）

本 SDK 为 R1 提供的官方服务全部封装在 `r1_tool` 里：

| group | 服务 | 覆盖内容 |
|---|---|---|
| `loco` | sport | `GetFsmId/Mode`、`SetFsmId`、`Damp`、`StandUp`、`Start`、`ZeroTorque`、`StopMove`、`SetVelocity/Move`、`SetSpeedMode` |
| `msc` | motion_switcher | `CheckMode`、`SelectMode`、`ReleaseMode`、`GetSilent`、`SetSilent` |
| `state` | robot_state | `ServiceList`、`GetPkgVersion`、`LowPowerStatus/Switch`、`SetReportFreq`、`ServiceSwitch` |
| `audio` | voice | `TtsMaker`、`GetVolume/SetVolume`、`LedControl`、ASR(`rt/audio_msg`) |
| `config` | config | `Get/Set/Del/Meta` |
| `lowstate` | rt/lowstate | 只读打印 IMU + 35 电机 q/dq/tau |

> 说明：遥控器上其余按键大多只是 `SetFsmId` 的不同取值（2/3/4/811…）与速度档位，
> 走的是同一个 `sport` 服务，并非独立 API；本工具的 `loco set-fsm <id>` 可覆盖。

### 交互终端（推荐，无需记命令）

```bash
# 菜单列出全部服务与指令，按序号选择，参数逐项提示
python3 example/r1/high_level/scripts/r1_tool.py
python3 example/r1/high_level/scripts/r1_tool.py --iface eth0

# 也支持直接执行
python3 example/r1/high_level/scripts/r1_tool.py loco get-fsm
python3 example/r1/high_level/scripts/r1_tool.py -y state services
python3 example/r1/high_level/scripts/r1_tool.py audio tts "你好"
```

底层 C++ 工具也可直接用：

```bash
./build/bin/r1_tool <网卡> <group> <action> [args...]
./build/bin/r1_tool help
./build/bin/r1_tool eth0 lowstate dump 3
```

### 仅运控快捷封装（loco）

`scripts/loco_cli.py` 与 `scripts/loco.sh` 只包 loco 一组，轻量快捷：

```bash
python3 example/r1/high_level/scripts/loco_cli.py            # 交互菜单
python3 example/r1/high_level/scripts/loco_cli.py status
example/r1/high_level/scripts/loco.sh status
example/r1/high_level/scripts/loco.sh damp             # 阻尼      (fsm 1)
example/r1/high_level/scripts/loco.sh stand            # 站立      (fsm 4)
example/r1/high_level/scripts/loco.sh start            # 启动主运控 (fsm 811)
example/r1/high_level/scripts/loco.sh zero-torque      # 零力矩    (fsm 0)  ⚠️ 危险
example/r1/high_level/scripts/loco.sh move 0.2 0 0 5
example/r1/high_level/scripts/loco.sh stop
example/r1/high_level/scripts/loco.sh speed 2
```

选项：`--iface <name>` 指定网卡，`-y` 跳过确认，`--dry-run` 只打印原生命令不下发。
危险操作（`zero-torque`、`move`、`msc release`）在非 `-y` 时要求输入 `YES` 确认。

## 部署（R1-EDU 算力背包）

R1-EDU 的“开发计算单元”（算力背包，Jetson Orin，`192.168.123.164`，aarch64）是最终部署目标。
程序在背包本机运行时 DDS 走 localhost，延迟最低。建议：

1. 在背包上装依赖、拉取本仓库、`scripts/build.sh` 编译。
2. `ssh` + `tmux` 运行（无需显示器），PICO 端 UDP 发往 `192.168.123.164:9999`。
3. 放行端口：`sudo ufw allow 9999/udp`；修改 `unitree/123` 默认密码。
4. **不要部署到运控计算单元（PC1）**——那是宇树运控专用，不对外开放。

开发阶段可在外部 Ubuntu 机器上先联调（编译快、看日志方便），稳定后再上背包。

## 首次上机安全顺序

1. `./bin/r1_loco_client <网卡>`：确认 DDS 通路（可站立/行走）。
2. `./bin/r1_arm_sdk_dds_example <网卡>`：确认 `rt/arm_sdk` 能接管双臂。
3. 再运行本程序。所有人员保持安全距离。

## 已知限制 / 待办

- **安全闸门**：判据集中在 `r1_pico_safety_policy.h`，优先级为
  **急停闩锁 > 一键回零 > 保持 > 正常遥操**。
  - 急停（`emergency_stop_latched=true`）= 停移动 + 阻尼，双臂保持；**本端不自动复位**，
    需操作者显式重新站立（PICO 站立键或 stdin `s`）——与官方 `xr_teleoperate`
    「双摇杆按下 = Damp()」行为一致。注意本端只看闩锁位，不依赖 PICO 端同时把
    `safe_to_execute` 置 false（两者是独立字段）。
  - 单次掉包超时（>600 ms）触发阻尼后同样**不自动站立**，恢复正常后需按 `s` 重新站立。
    这是刻意的设计选择（避免链路抖动后机器人自行起身），不是缺陷。
- **头/腰**：启动时回零使能，遥操过程中不跟随（`robot_control.head` 未消费）。
- **灵巧手**：`NullHandDriver` 只打印 6 路原始值（0..10000），实际驱动需继承 `HandDriver`。
- **A7**：官方固件未开放 `rt/arm_sdk` 覆盖模式，必须用 `--lowcmd`，且使用前需释放机载运控服务。
- **标定**：`r1_xr_pose_alignment.h` 中腰部偏移 `+0.15 x / +0.45 z` 为常量，上机后需按实际几何标定。
- **IK**：用 DLS 逼近官方 CasADi+IPOPT 目标，行为一致但非逐位相同；追求最高保真可用 Python 侧求解器。
