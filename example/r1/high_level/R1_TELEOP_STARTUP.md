# R1 上半身 PICO 遥操 —— 启动流程手册

> **适用**：R1-EDU 算力背包 PC2 = `192.168.123.164`（Jetson Orin NX / JetPack 5 / Ubuntu 20.04 / aarch64 / 单网卡 `eth10`）
> **程序**：`build/bin/r1_dual_arm_loco_skeleton`
> **整理日期**：2026-09-23 ｜ 所有日志文案、参数名、超时值均取自本仓库源码实测，非记忆
> **配套**：`R1_BACKPACK_ARCHITECTURE.md`（架构与全部排障细节）、`R1_CONTROL_ALGORITHM.md`（控制算法）
>
> 🔎 **只想"照着敲命令把遥操跑起来"→ 用 `R1_TELEOP_RUNBOOK.md`**（单页速查，2026-10-08 实测状态）。
> 本文件是**完整手册**：原理、日志逐行解读、网络四方案、故障二分表。

---

## 0. 先记住这 6 句（少踩一半坑）

1. **「启动程序」= 「接管双臂」**，不是"空跑等头显"。按下回车那一刻权重就是 100，头/腰会在 3 秒内**真实回零**。
2. **启动前机器人必须已在 811 主运控** —— 程序默认**不切 FSM**（官方遥操也不切）。先 `loco.sh status` 确认，不是 811 就 `loco.sh start`。
3. **「准备模式」不是一个开关**，而是"程序在跑、但还没收到可执行报文"这段状态。
4. **退出不是瞬停**：先腿变软（阻尼），再双臂回零，全程约 2~7 秒。退出前清空臂展。
5. **每次切完 FSM 必须跟一条 `loco.sh status` 复验** —— `ret=0` 只说明调用成功，不代表状态真切过去了。
6. ⛔ **绝不能 `kill -9`**。权重会停在 100 且不再发布 ⇒ 双臂冻结在最后一帧姿态，不会回落。

---

## 1. 启动前的三条硬前置

### 1.1 机械安全（物理前提，软件救不了）

- 机器人**吊挂**或有可靠支撑；若要试走跑，地面清空。
- **臂展范围内无人无物** —— 启动瞬间头 pitch/yaw + `WaistYaw` 会回零（3 秒），接管后双臂也会按指令动作。
- 灵巧手若已接入，同样清空。

### 1.2 服务与进程（每次上机都走，别省）

```bash
ssh unitree@192.168.123.164
tmux new -s teleop
#   弹出 `ros:foxy(1) noetic(2) ?` → **按回车跳过，永远不要按 1**（详见架构文档 §4.7.11）
cd ~/unitree_sdk2

echo "[$CYCLONEDDS_URI]"                        # 必须是 []
sudo systemctl stop r1-custom-head-remote       # 出厂服务会抢头/腰槽位
systemctl is-active r1-custom-head-remote       # 必须输出 inactive
```

| 要检查的 | 期望 | 为什么 |
|---|---|---|
| `$CYCLONEDDS_URI` | 空 `[]` | 非空 ⇒ 指向写死 `eth0` 的 xml ⇒ **DDS 静默超时**（现象和"脚本坏了"一模一样） |
| `r1-custom-head-remote` | `inactive` | 它 `enabled` + `Restart=always`，会抢头/腰关节。⚠️ **别 `disable`**，收工要 `start` 恢复 |
| 有没有别的 `r1_`/`teleop` 进程 | 无 | ⛔ **不能和 `r1_arm_manual` 同时跑** —— 两个程序互抢 `rt/arm_sdk`。要跑遥操就先在里面按 `q` |

### 1.3 网络通道（当前是**最大的一道坎**）

程序收端绑的是 `0.0.0.0:9999`（`r1_pico_udp.h` 的 `INADDR_ANY`），**从哪张网卡进来都收得到，代码一行都不用改**。
但背包**自己没有无线网卡**，所以 PICO 的报文怎么进来，必须先解决 —— 见 **第 5 章**（含"背包能不能自己开 AP"的完整结论）。

---

## 2. 启动流程：进入准备模式

### 2.1 上机检查清单（顺序别换）

```bash
cd ~/unitree_sdk2

hostname -I                                       # 应含 192.168.123.164
example/r1/high_level/scripts/loco.sh status      # **必须是 811**；不是就 loco.sh start
ss -lun | grep 9999 || echo "(9999 空闲)"          # 端口没被占
stat -c "%y %n" build/bin/r1_dual_arm_loco_skeleton   # 产物 mtime 应新于源码
```

> **产物时间戳判据**：`bin` 的 mtime 必须**晚于**源码。`rsync -a` 保留时间戳，两边源码时间一致**不代表**产物是新的。
> ⚠️ 若编译时出现 `make: Clock skew detected`，说明背包时钟偏了，`make` 会**静默跳过重编** —— 先 `sudo date -s` 校时再编（架构文档 §4.7.13）。

### 2.2 启动命令

```bash
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5
```

| 参数 | 说明 |
|---|---|
| `<网卡名>`（第 1 个位置参数，**必填**） | 与机器人内网（192.168.123.x）相连的有线网卡 = **`eth10`**。官方示例写的 `eth0` 是别的机型，别照抄 |
| `--variant a5\|a7` | 手臂型号，本机是 **a5** |
| `--pico [port]` | 启用 PICOHandLink UDP 接入，默认 `9999` |
| `--motion` / `--lowcmd` | 发布到 `rt/arm_sdk`（默认，**行走时并行接管双臂**）/ 发布到 `rt/lowcmd`（全关节，需先释放机载运动服务） |
| `--pico-frame head_yaw\|head_trans\|basis` | 双臂参考系，默认 `head_yaw` |
| `--pico-source controllers\|teleop` | 位姿来源，默认 `controllers`（原始数据，对齐宇树摇操）。**只能选 `controllers`**，理由见 §4.5.4 |
| `--auto-stand` | 首个可执行包时自动 `StandUp()`（→ FSM 4）。**默认关**，加了会把 §4.2 那个未验证状态重新引进来 |

### 2.3 日志逐行解读（下面这段是**真实上机输出**）

```
[main] R1 A5 motion_mode=1 iface=eth10 pico_mode port=9999
[arm] lowstate connected.
[arm] publishing to rt/arm_sdk @ 250 Hz
[arm] head/waist going home (3 s)...          ← ⚠️ 从这里开始真的会动，持续 3 秒
[arm] head/waist home done.
[arm] controller ready.                        ← 头/腰已回零，权重已 100
[hand] NullHandDriver initialized (no hardware).
[pico] UDP receiver listening on 0.0.0.0:9999
[main] ready. 等待 PICO HandLink 数据... (stdin: 'v vx vy vyaw' / 's' / 'd' / 't' / 'q', Ctrl+C 退出)
[pico] frame mode: head_yaw | target loop 30 Hz (官方 teleop 默认 30 Hz) | WMA 关节平滑 4 帧 [0.4,0.3,0.2,0.1] + 250Hz 发布速度限幅 30 rad/s
[pico] 首包不切 FSM（官方遥操前置：机器人已在 811 主运控）。腿不动就先执行 example/r1/high_level/scripts/loco.sh start；要旧的自动站立行为请加 --auto-stand
[loco] LocoClient ready.
```

**看到 `[main] ready.` 就算进入准备模式。**（`[loco] LocoClient ready.` 排在最后是**线程交错**，不是异常。）

### 2.4 「准备模式」到底是什么

`r1_arm_controller.h` 的 `R1ArmController::start()`：

```cpp
setWeight(params_.motion_mode ? 1.0 : 0.0);   // --motion(默认) ⇒ mode_pr = 100，一上来就满权重
homeHeadAndWaist(3.0);                        // 头/腰 3 秒内真实回零
```

所以此刻的真实状态是：

| 部位 | 谁在管 | 状态 |
|---|---|---|
| 双臂 10 关节 | **我们的程序**（`rt/arm_sdk`，权重 100） | 已脱离 `ai_sport`，静止在**启动瞬间的姿态**（目标 = 启动时实测 ⇒ 零跳变） |
| 头 pitch/yaw + `WaistYaw` | 我们的程序 | 已回零 |
| `WaistRoll` | 运控（写不生效） | 不受影响 |
| 双腿 | 运控（FSM 811） | 照常，还能走 |

⚠️ **别用手去掰手臂** —— 现在它是有刚度的。
**判据**：双臂静止保持、不抖不漂；腿正常。

---

## 3. 链路自检：UDP 到底到没到

程序"在等"不等于"包到了"。用**程序之外**的工具独立确认（另开一个 tmux window：`Ctrl+B` 再 `c`）：

```bash
cd ~/unitree_sdk2
ss -lun | grep 9999                                       # 应看到 0.0.0.0:9999
echo 123 | sudo -S timeout 10 tcpdump -n -i any udp port 9999 -c 5
```

### 3.2 二分表：链路问题 vs 解析问题

| 现象 | 结论 | 下一步 |
|---|---|---|
| 有包 **且**日志出现 `[pico] first packet seq=...` | 链路 + 解析都通 | 正常遥操 |
| 有包 **但**日志没有 `first packet` | **链路没问题，是字段/解析问题** | 查 `operator_mode` 是否 `active_stream`、`quality` 是否 `live`、`safe_to_execute` 是否 `true` |
| **完全没包** | **链路问题** | 查 PICO 端目标 IP/端口、网段、发送端到底有没有在发（见第 5 章） |

程序里的对应标志：

```
[pico] first packet seq=... model=... sdk="..." operator_mode="..."   ← 报文进来了
[loco] PICO 首个可执行包：保持当前 FSM，不自动切档。                     ← 只解锁底盘速度，不碰 FSM
```

> `sdk="..."` 那一段是**辨认 App 的关键**：应为 `pico-openxr-single`（实机主 App）。
> 若是 `pico-openxr-bridge`，说明装的是旧的 `:app` —— 它**不发 `operator_mode`**，
> 会被兜底成 `stop_signal` ⇒ 每包都 `Damp()`、腿一直软。程序会额外打一条 ⚠ 告警。

### 3.3 用模拟发送器排练（强烈建议，真机第一次动别急着戴头显）

在 **Mac** 上（已在机器人内网 `192.168.123.200`）：

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
$PY example/r1/high_level/scripts/pico_sim_sender.py \
    --host 192.168.123.164 --port 9999 --duration 30 --hz 30
```

- ⛔ 双臂会**真的跟着正弦摆动**，清空臂展。
- 另开终端 `loco.sh watch 1` 记录 FSM 变化（原因见 §4.2）。
- ⚠️ **发送端一停，>600 ms 判掉包 ⇒ `Damp()`（FSM 1）⇒ 腿变软** ⇒ 排练必须在**吊挂/有支撑**下做。
  恢复走 `loco.sh start`。

---

## 4. 实际遥操

### 4.1 让机械臂真正跟随你的三个条件

`r1_pico_safety_policy.h::decideDisposition()` 要求**同时**满足：

1. `emergency_stop_latched == false` —— 急停优先级最高，压过一切；
2. `safety.safe_to_execute == true` —— PICO 端把 `calibration_ready` **折进**这个字段了，本端**零消费**标定字段；
3. `operator_mode == "active_stream"` **且** 包新鲜 `< 600 ms`（`kDefaultStaleMs = 600.0`）**且**来源有效
   （默认 `--pico-source controllers`，看 `ctrl.output_valid`，即左右手 `quality == "live"`）。

任一不满足 ⇒ `kHold`：**只停移动，双臂保持**。

### 4.2 首包**不再切 FSM**（2026-09-29 已改）

**旧行为（已去掉，默认）**：首个可执行包自动调一次 `StandUp()`，把 FSM 从 811 拉到 4。

这是**偏离官方**的一步：官方 `xr_teleoperate` **从不切 FSM**，明确要求事先把机器人置于运控态
（只支持 `Regular mode` R1+X）。而且它正好把我们送进「**FSM 4 下低层认不认 `rt/arm_sdk`**」
这个未验证的状态，加上起身与开始跟随几乎同帧，一旦不跟随根本分不清是谁的问题。

**新行为（默认）**：首个可执行包**只解锁底盘速度闸门，不碰 FSM**。

```
[loco] PICO 首个可执行包：保持当前 FSM，不自动切档。底盘速度已解锁（腿要动仍需 811 主运控）。
```

⇒ **前置要求变成了硬性的：起程序前机器人必须已在 811。** 开跑前先确认：

```bash
example/r1/high_level/scripts/loco.sh status    # 必须是 811；不是就 loco.sh start
```

想恢复旧行为（自动站立）加 `--auto-stand`，但**不建议**——它会把悬案重新引进来。
程序本身**仍然从不调 `Start()`**，只会在你按 `s` 或 `--move` 时调 `StandUp()`。
若腿不听摇杆，手动补一条 `loco.sh start`。

> 本节的旧结论「首包会掉到 FSM 4，用 `loco.sh watch 1` 观察」**已作废**：
> 现在默认不会掉档，`watch` 只会看到一条平线。若你显式加了 `--auto-stand`，才需要照旧盯 `watch`。

### 4.3 操作中的按键（stdin，**只能在 tmux / `ssh -t` 里按**）

| 键 | 作用 |
|---|---|
| `s` | 站立（**急停/掉包后必须靠这个恢复，本端不自动复位**） |
| `d` | 阻尼（FSM 1，腿变软） |
| `t` | 停走（速度置零，不动 FSM） |
| `v vx vy vyaw` | 直接给速度 |
| `q` | 交还并退出（= Ctrl+C，完全等价） |

### 4.4 急停与掉包的行为（都不会自动复位）

| 触发 | 行为 | 恢复 |
|---|---|---|
| 头显侧软件急停 | 停移动 + 阻尼 + 双臂保持（日志：`[pico] 急停闩锁：停止移动 + 阻尼，双臂保持。`） | 解除后须**操作者显式**重新站立（PICO 站立键 或 stdin `s`） |
| 掉包 > 600 ms | `Damp()`（**腿变软**） | 同上，按 `s` |
| `operator_mode == "stop_signal"` | 同上追加阻尼 | 同上 |
| 报文里**没有** `operator_mode`（装了旧 `:app`） | 兜底成 `stop_signal` ⇒ **每包都阻尼、腿一直软**；程序打一次性 ⚠ 告警 | 换装 `:openxr-app`（§4.5.1） |

> 与官方 `xr_teleoperate` 行为一致：**本端不自动复位**，避免闩锁解除瞬间机器人自己站起来。

### 4.5 代码就绪性核查：接收端 vs 真 App 的契约（2026-09-29 核对源码）

**为什么能核对**：PICO 端 App 源码就在本机
`/Users/plf/Project/PICOProject/PICOHandLink`（分支 `main`，`9c7f3ca`）。
下面每条都是读 App 源码得出的，不是推测。

#### 4.5.1 先确认装的是哪个 App（**这条搞错会出事故**）

| 模块 | 角色 | 发什么 |
|---|---|---|
| **`:openxr-app`** | **实机主 App**（README_zh「分支说明」明写：追踪/界面/安全层/UDP 都在它里面） | `PacketEncoder` 不用；走 `HandLinkProtocol.cpp`，**含** `operator_mode` / `teleop` / `controllers` / `robot_control` |
| `:app` | 旧 Spatial SDK 参考实现，**不要装** | `network/PacketEncoder.kt`，**无** `operator_mode`/`teleop`/`robot_control`，`safety` 里也**无** `emergency_stop_latched` |

⛔ **误装 `:app` 的后果**：我们的解析器把缺失的 `operator_mode` 兜底成 `"stop_signal"`
（`r1_pico_udp.h` 的 `parsePicoPacket()`）⇒ `holdTriggersDamp()` 每包都成立 ⇒ **持续 `Damp()`，腿一直是软的**，
而且日志上看起来"包收到了"。

**判据（2026-09-29 起程序会主动告诉你）**：

```
[pico] first packet seq=... model=... sdk="pico-openxr-single" operator_mode="active_stream"
[pico] ⚠ 报文缺少 operator_mode 字段，已按 stop_signal 兜底 ⇒ 不会遥操。
       请确认头显里装的是 :openxr-app（sdk="pico-openxr-single"），不是旧的 :app（"pico-openxr-bridge"）。本包 sdk="..."
```

`sdk` 是 `pico-openxr-single` = 对；`pico-openxr-bridge` = 装错了。
解析端新增 `operator_mode_present` 位专门区分「**键缺失**」与「操作者真的发了 stop」——
两者现象一样，但处置完全不同。

#### 4.5.2 契约逐项核对（以 `HandLinkProtocol.cpp` 为准）

| 接收端消费的字段 | App 是否发送 | 结论 |
|---|---|---|
| `sequence` / `timestamp` | ✓ | 一致 |
| `operator_mode` | ✓ `active_stream`/`return_zero`/`stop_signal`/`diagnostic_capture` | 一致；正常遥操时是 `active_stream` |
| `safety.safe_to_execute` | ✓ | 一致。App 文档明确「**机器人端只以 `safe_to_execute` 作为执行闸门**」 |
| `safety.emergency_stop_latched` | ✓ | 一致 |
| `safety.calibration_ready` / `teleop_armed` / `session_*` | ✓（我们不消费） | 合理 —— 它们已被折进 `safe_to_execute` |
| `controllers.left/right.{quality,pose,input}` | ✓ | 一致 |
| `hmd.quality` / `hmd.pose` | ✓ | 一致 |
| `pose.orientation_quat` | ✓ | 一致（优先四元数，避开万向锁） |
| `robot_control.hands.left/right`（6 路 raw） | ✓ **默认发** | 一致 |
| `robot_control.base` | ✓ **默认发** | 一致，直接被 `client.Move()` 消费 |
| `robot_control.head` | ⚠️ **默认不发**（"发送内容"页需手动勾） | 我们本来也没消费 |
| `robot_control.body.waist` | ⚠️ **默认不发** | 同上（现在只解析成 `has_waist` 标志） |
| `robot_control.arms` | ❌ **R1 profile 不发**：`arm_ik_enabled=false`、`joints_per_arm=0`、`backend="no_ik"` | 我们用自己的 A5 IK，无影响 |

#### 4.5.3 App 侧「可执行」的 7 个条件（任一不满足 ⇒ 我们只 `kHold`）

来自 `docs/CONTROLLER_ONLY_PROFILE_zh.md` §安全层 + `BuildSafetyState()`：

1. UDP 连续发送中（≈ 我们的 600 ms 判据）
2. 已按「使能执行」（左 X 短按，`teleop_armed`）
3. 未急停
4. OpenXR session running **且** focused
5. **左右手柄都完成本地校准**（按右手柄 A）
6. 左右手柄追踪均 `live`
7. 样本新鲜（≤250 ms，**头显自己的时钟**；与我们的 `rx_age_ms` 是两个独立判据）

⇒ 所以「头显里不做校准」就永远不进遥操。这是**设计如此**，不是故障。

#### 4.5.4 ⚠️ 位姿语义差别：`--pico-source` **只能**选 `controllers`

| 字段 | 语义 |
|---|---|
| `controllers.left/right.pose` | **原始绝对位姿**（与 `hmd.pose` 同一参考空间） |
| `teleop.left/right.pose` | `controller_pose − calibration_pose`，**相对校准中立位的增量** |

我们的对齐公式（`r1_xr_pose_alignment.h`，移植自官方 televuer）
`P_robot = Ryawᵀ·(P_wrist − P_head) + (0.15, 0, 0.45)` 吃的是**绝对位姿**
—— televuer 原版也是拿绝对的 `left_arm_pose` 减绝对的 `head_pose`。

- ✅ 默认 `--pico-source controllers` **正确且自洽**（`src_valid` 也正好由 App 的 `controllers.*.quality` 决定）
- ⛔ 改成 `--pico-source teleop` 会变成「用**增量**减**绝对头部位姿**」，
  得到的量几乎只跟头部位置有关 ⇒ 手臂锁死在某个错位姿态。
  **要换来源必须先换公式**（`P_robot = P_neutral + Ryawᵀ·Δp`，不再减头）。
  注意仿真里 `pico_sim_sender.py` 把 `teleop` 和 `controllers` 填成**同一份数据**，
  所以这个差别在仿真里**永远不会暴露**。

#### 4.5.5 仿真验证的边界（"仿真过了"到底证明了什么）

`sim/pico_pipeline_test` 与主程序 `--pico` 分支**同源**，但**剥离了 DDS**；
且 `pico_sim_sender.py` 的位姿是用 MuJoCo FK **反算**出我们对齐公式的输入（再用独立 FK 比对）。

**证明了**：JSON 解析 + 对齐 + A5 IK 这条数学链自洽。
**没证明**：① DDS 发布 / `arm_sdk` 权重 / `ai_sport` 是否采纳
（这部分是 **`r1_arm_manual` 在真机上单独证明的**）；
② ~~首包自动 `StandUp()` 把 FSM 811→4 之后还能不能跟随~~ —— **该项已消解**，
首包默认不再切 FSM（§4.2）；
③ 真手柄的位姿范围是否落在机器人可达空间内；
④ 真 App 的字段（它发的是我们钦定 schema 的**超集**，且 `head`/`waist` 默认不发）。

#### 4.5.6 代码层面待改清单

**2026-09-29：第 1/2/3 项已修复**（改动落在 `r1_dual_arm_loco.cpp`、`r1_pico_udp.h`、
`tests/test_pico_parse.cpp`，本机语法校验 3 目标 0 error、单测 + 7 安全场景 + 全链路回归全过）。

| # | 问题 | 状态 |
|---|---|---|
| 1 | 首个 `kTeleop` 包触发 `StandUp()` → FSM 4 | ✅ **已修**：默认**不再切 FSM**，只解锁底盘速度；旧行为降级为 `--auto-stand`（默认关）。⇒ 前置要求变为「起程序前必须已在 811」 |
| 2 | `kTeleop` 但 `pose` 缺失时静默不动 | ✅ **已修**：1 Hz 限速告警，明确指出 `source` / 左右 `pose` 是否缺失 / 去 App「发送内容」页勾选 |
| 3 | `operator_mode` 缺失兜底成 `stop_signal` | ✅ **已修**：新增 `operator_mode_present` 位，缺失时**打一次性告警**并提示「装的是不是旧 `:app`」；`first packet` 日志现在也打印 `sdk` 与 `operator_mode` |
| 4 | 头/腰跟随未实现（启动回零后锁在 0） | ⏳ 未做。App 已在发 `robot_control.head`/`body.waist`（**需在「发送内容」页勾选**），消费即可；注意 `WaistRoll(12)` 写不生效 |
| 5 | 灵巧手 `NullHandDriver` | ⏳ 未做。按已定 `HandProfile` 接厂商桥，且**需先确认灵巧手型号** |
| 6 | 腰部偏移 `0.15/0.45` 为常量 | ⏳ 须上机实测。A5 IK 残差 ~22 mm 是结构固有；整体工作空间可能偏 |

> 第 4/5/6 项都**不阻塞**首次真机遥操联调：4 是功能增强（不动也不会错），
> 5 需要硬件到位，6 需要真机数据。可以先联调双臂跟随，再回头做这三项。

---

## 5. PICO → 背包 的网络通道（含「无法连 WiFi 时怎么开 AP」）

### 5.1 现状（2026-09-23 实测）

| 项 | 实测结果 | 含义 |
|---|---|---|
| `ls /sys/class/net` | `lo dummy0 docker0 eth10` | **没有 `wlan*`** |
| `/sys/class/net/*/wireless` | 不存在 | 背包**没有任何无线网卡** |
| `lsusb` | 只有 2 个 root hub | 没插任何 USB 设备 |
| NM 里的连接 | `netplan-eth10`(已激活) / `docker0` / **`Unitree`(wifi，无设备)** | 见 §5.3 |
| 内核无线子系统 | `net/wireless`、`net/mac80211`、`drivers/net/wireless` **都在** | 插对 USB 网卡**即插即用** |
| `/lib/modules/$(uname -r)/build` | **存在** | 还能编 out-of-tree 驱动 |
| 固件 | `htc_9271.fw` ✓ `rt2870.bin` ✓ `rtlwifi/rtl8192cufw.bin` ✓ | 三类常见卡"插上就有" |
| 内网网关 `192.168.123.1` | **ARP FAILED** | 网关那台设备**不在线** |

> ### ⚠️ 2026-10-08 更新：方案 B（USB 网卡）**已实测否决**
>
> 插上了一块 USB 无线网卡：**ATHEROS UB91 = AR9271**（`0cf3:9271`，驱动 `ath9k_htc`，
> 固件 `htc_9271-1.4.0.fw` 加载成功，USB 2.0 480M，`bMaxPower=500mA`，挂在 `usb 1-2`）。
>
> **症状**：扫描完全正常（能看到 `leadshine-5G` 等），但**一旦 `nmcli dev wifi connect` 关联，
> 内核就静默挂死** —— 那次启动的内核日志在挂死前**没有任何 `Call trace` / `BUG` / `Oops`**，
> 随后 `tegra_wdt_t18x`（**超时 120 s**）复位，复位源 `PMC reset source: BCCPLEXWDT`。
> 实测**连续触发 4 次**（10:01 / 10:07 / 10:10 / 10:13 各重启一次），
> 每次表现为 SSH「卡住约两分钟」后失联 —— 那正好是看门狗在倒计时。
>
> **这是驱动层死锁，不是供电不足**：掉电会给出不同的复位源，也不会走 120 s 看门狗。
> 因此换 USB 口 / 加供电 Hub **大概率无效**；`nohwcrypt=1`、`ps_enable=0` 未试。
>
> ⇒ **别在这台背包上再用这张卡**。要无线通道请走 §5.5（方案 C）。

### 5.2 四条路线总表

| 方案 | 需要什么 | 能否今天就用 | 稳定性 |
|---|---|---|---|
| **A** 机器人配套的路由器/AP | 确认实物并开机 | 待确认 | 最好（原厂配套） |
| **B** 背包插 USB 网卡自建热点 | 买一块受支持的卡（约 ¥20~50） | ❌ **已实测否决** | 见上：AR9271 一关联就内核挂死 |
| **C** Mac 做热点 + UDP 中继 | **零新增硬件** | ✅ **可以（2026-10-08 已端到端验证）** | 中（依赖 Mac 在场） |
| **D** 头显走 USB-C 转网口，插机器人内部交换机 | 一个 USB-C 转网口 | 取决于头显是否支持 | 最好（有线无干扰） |

### 5.3 方案 A：机器人配套的路由器 / AP（先确认）

**证据（已澄清一个先前的误判）**：背包 NM 里那条 `Unitree` profile 的
`802-11-wireless.mode = infrastructure` —— 它是**客户端**配置，意思是"**去连**一个叫 Unitree 的 AP"，
**不是**"背包能广播这个热点"。该文件创建于出厂装机日（2026-05-22），
说明出厂时就有人用这块背包连过那个 SSID。

**推断**：`Unitree` 很可能是**机器人配套的那台路由器/AP**（内网网关 `192.168.123.1` 就是它）。
两个旁证：
- R1 规格里写的是"**WiFi 6 + 蓝牙 5.2**"，而背包侧一片空白 ⇒ 无线在**别的模块**上；
- 默认路由 `default via 192.168.123.1` 是**静态写死**的，说明设计上那里本该有一台设备。

**现在的问题**：`192.168.123.1` 的 ARP 是 **FAILED** ⇒ 那台设备**不在线**（没开机 / 没接进内网 / 不在手边）。

**请先确认两件事**：
1. 机器人到货时是否配了一台**路由器 / 无线 AP 盒子**？如果有，找到它、给它供电、接进机器人内网的交换机；
2. 开机后回来看：`ip neigh | grep 192.168.123.1` 应该从 `FAILED` 变成 `REACHABLE`。

一旦它在位，PICO 直接连它的 SSID，目标地址填 `192.168.123.164:9999` 即可 —— **这是最省事的一条路**。

### 5.4 方案 B：背包插 USB 网卡，把背包自己变成热点

> ⛔ **本节结论已于 2026-10-08 被实测推翻，仅作背景保留。**
> 已插的 AR9271 卡「能扫描、一关联就内核挂死 + 看门狗重启」（详见 §5.1 后的更新框）。
> **不要再按本节采购/操作。**

**原可行性结论：现在不行（无硬件），插对卡之后可以。**
理由是内核**编了完整无线子系统**（§5.1），所以不需要重编内核。

**跑体检（只读）**：

```bash
cd ~/unitree_sdk2
bash example/r1/high_level/scripts/backpack_ap.sh check    # 体检
bash example/r1/high_level/scripts/backpack_ap.sh chips     # 该买哪块卡（按本机实况判定）
```

**买哪块卡 —— 关键是「AP 模式」这一列**（能上网 ≠ 能开热点，很多驱动只支持客户端）：

| 芯片 | 驱动 | 本机状态 | AP | 结论 |
|---|---|---|---|---|
| **AIC8800**（AX1800 免驱款主流） | `aic8800_fdrv` + `aic_load_fw`（厂商 out-of-tree） | **已装**：两个 `.ko` 在 `/lib/modules/…/wireless/aic8800/`，`vermagic` 与运行内核逐字一致；固件 `/lib/firmware/aic8800DC/` | ✓ | ✅✅ **第一推荐**。驱动/固件/udev/NM 托管策略全都就位，**零编译插上即用**；驱动内明确含 `NL80211_FEATURE_AP_MODE_CHAN_WIDTH_CHANGE`、`NL80211_IFTYPE_P2P_GO`、`ap_uapsd_on` 参数 ⇒ 支持 AP |
| **AR9271**（TL-WN722N **v1**） | `ath9k_htc`（内核自带） | 固件 `htc_9271.fw` ✓ | ✓ | ✅ 次选。AP 模式最成熟，2.4GHz |
| **RT5370 / RT3070** | `rt2800usb`（内核自带） | 固件 `rt2870.bin` ✓ | ✓ | ✅ 便宜好买，2.4GHz |
| **RTL8188CUS / RTL8192CU** | `rtl8192cu`（rtlwifi，内核自带） | 固件 `rtlwifi/rtl8192cufw.bin` ✓ | ✓ | ⚠️ 同一颗芯片可能被 `rtl8xxxu`（无 AP）抢走绑定 ⇒ **有运气成分**，不如上面三块 |
| MT7601U | `mt7601u` | 固件 ✓ | ✗ | ❌ 主线**没实现 AP 模式**，只能当客户端 |
| RTL8188EU / 8192EU | `rtl8xxxu` | 固件 ✓ | ✗ | ❌ 主线只有 STA 模式 |
| MT7921AU / MT7922 | ✗ | — | — | ❌ 需 kernel ≥ 5.18，本机 5.10 |
| RTL8852AU / BU | ✗ | — | — | ❌ 需 kernel ≥ 6.4，本机 5.10 |

⚠️ **AIC8800 买卡注意**：驱动白名单里的 USB `VID:PID` 必须命中——
`a69c:88dc/88dd/88de/8d41/8d81/8801`、`368b:88e5/88de/8d99/8d91`（网卡态），
以及 `a69c:8800/8d40/8d80`、`368b:8d90/8d91/8d92/8d99`（下载固件态，同一张卡会换 ID 二次枚举，**两组都命中才算支持**）。
另：固件只装了 `aic8800DC` 一套，若买到 **D80 变体**可能缺固件 ⇒ 插上后**必须看 `sudo dmesg` 确认固件下载成功**。
⚠️ **TL-WN722N 的 v2/v3 换了芯片**（不是 AR9271），买之前看清版本号。

**卡到了之后开热点**：

```bash
bash example/r1/high_level/scripts/backpack_ap.sh start R1Backpack '你的密码'   # ≥8 位
bash example/r1/high_level/scripts/backpack_ap.sh status
```

脚本做的事（**不用装 hostapd** —— 背包 apt 是坏的，能少装就少装）：
写一份 NM profile（`802-11-wireless.mode=ap` + `ipv4.method shared`），
NM 1.22 自带 AP 模式，DHCP 由它内置的 dnsmasq 提供（已装）。
密码直接落到 root 600 的 profile 文件，**不经过命令行**（否则会出现在 `ps` 里）。

**PICO 端填什么**：`10.42.0.1:9999`（AP 网卡地址，NM 默认给 `ipv4.method shared` 分 `10.42.0.1/24`）。
填 `192.168.123.164:9999` **也能到** —— 收端绑的是 `INADDR_ANY`，两个地址都落在同一个 socket 上。

**三条约束**：
- ⚠️ 热点网段**不要**用 `192.168.123.0/24`（会和 DDS 抢路由）。NM 默认的 `10.42.0.0/24` 天然不冲突，保持默认。
- `autoconnect=false`：开机不自动开热点，免得和有线/DDS 抢路由。
- 收工记得 `backpack_ap.sh stop`。

> 区分两个脚本：**`backpack_wifi.sh`** 是让背包**当客户端**去连某个 WiFi（要连外部 AP 时用）；
> **`backpack_ap.sh`** 是让背包**自己当 AP**。两者都依赖同一块 USB 网卡，别同时用。

### 5.4.1 采购清单：具体买哪个型号（2026-09-24 补）

上表是**芯片级**结论。落到「去哪个店买哪个东西」，按下面的顺序挑。

**① 最省事（零编译）：AIC8800 芯片的 USB 网卡 —— 但有**一个**赌注**

驱动 `aic8800_fdrv` + `aic_load_fw`、固件、udev 规则、NM 托管策略**本机全就位** ⇒ 插上直接出 `wlan0`，
不需要编任何东西。**赌注在于必须买到 `DC` 变体**：

- 固件只装了 `/lib/firmware/aic8800DC/`，而驱动源码里目录名是**写死**的
  （`rwnx_platform.c:1589` → `sprintf(..., "aic8800DC")`，见 §4.7/架构文档）。若买到 **D80 变体**
  （`a69c:8d80/8d81`）**可能缺固件** —— 补救办法是把 D80 固件补进 `/lib/firmware/aic8800DC/`
  （名字得改成驱动认的那个），比换卡麻烦。
- **必须是 USB 口版本**。AIC8800 还有 SDIO / M.2 / 板上贴片模组，接口不对就白买。
- 下单前**直接问卖家要 `USB VID:PID`**，对照 §5.4 的白名单（网卡态 + 固件下载态**两组都要命中**）。
  拿不到就等货到 `lsusb` 现验，别指望商品页 —— 商品页普遍只写「AX1800 / 免驱」，不写变体和 ID。
- 市场形态：常见名 **「AX286 免驱 WiFi6」**迷你款（USB2.0、2.4GHz、286Mbps、内置天线），约 ¥15–35；
  工业模块（如带蓝牙的 `AIC8800US9-80I`）约 ¥25–30。
- ⚠️ **厂商页面自相矛盾**：有的写「工作模式 STA / AP / Wi-Fi Direct」，有的只写
  「Infrastructure / Ad-Hoc」。**别信商品页**，以驱动白名单 + 到手 `iw list` 实测为准。
- ✅ 一个隐藏加分项：这类「免驱」卡出厂常以 **U 盘形态**出现，而背包
  `/etc/udev/rules.d/aic.rules`（`a69c:5721/5722/572a` → `eject`）正是为这种形态准备的，会自动把它踢回 WiFi 模式。
- ❌ 减分项：**内置天线**。背包是金属结构、又背在身上，nano 款信号会明显打折。

**② 最稳（确定性最高、但要认准版本）：AR9271 + 外置天线**

`ath9k_htc` 内核自带、固件 `htc_9271.fw` 已就位、AP 模式最成熟、社区验证最多，**没有任何变体陷阱**。

| 型号 | 要点 | 参考价 |
|---|---|---|
| **Alfa AWUS036NHA** | AR9271 + **可拆 SMA 高增益天线** | ¥150–250 |
| **TP-Link TL-WN722N** | **必须 v1**；v2/v3 换了芯片，不是 AR9271 | ¥50–100（多为二手） |
| 淘宝第三方「AR9271 模块 + SMA 天线」 | 认准芯片即可，多为无障碍改装款 | ¥40–80 |

**带外置天线这一点在实际部署里比参数重要** —— 背包金属壳 + nano 内置天线是常见翻车点。
缺点是 802.11n / 2.4GHz / 150Mbps 且已停产；但对遥操（30 Hz 小包）**绰绰有余，不要为速率买单**。

**③ 便宜可用：RT5370 / RT3070**

`rt2800usb` 内核自带、固件 `rt2870.bin` 已就位、AP 可用；nano 款约 ¥15–30。
型号参考 Alfa AWUS036NH（RT3070）、Panda PAU05 / PAU06（RT5372）。

⚠️ **本档最大的坑**：淘宝廉价「免驱 nano 网卡」**大量实际是 MT7601U 或 RTL8188EU**（两者主线都**没有 AP 模式**），
外观完全一样。到手用 `lsusb` 验 PID 最直接：

| `lsusb` 里的 PID | 芯片 | 能开热点吗 |
|---|---|---|
| `148f:5370` / `148f:3070` | RT5370 / RT3070 | ✅ |
| `148f:7601` | MT7601U | ❌ 只能当客户端 |
| `0bda:8179` / `0bda:8178` | RTL8188EU / RTL8192EU | ❌ 只能当客户端 |

**④ 不要买（会白等或卡住）**

| 类型 | 为什么不买 |
|---|---|
| MT7601U / RTL8188EU / RTL8192EU | 主线驱动**没实现 AP 模式** |
| RTL8811CU / 8821CU / 8812AU / 8814AU / 8822BU / 8852AU | 要自编驱动；背包**无外网 DNS**，只能 Mac 下源码再 scp 过去编，多一道风险 |
| 新款 AX1800 / AX3000 / WiFi6E 卡（MT7921U / MT7922U / RTL8852BU …） | 需 kernel ≥5.18 / ≥6.4，本机 **5.10** |
| RTL8188CUS / 8192CU | 驱动支持 AP，但同一颗芯片可能被无 AP 的 `rtl8xxxu` 抢绑 ⇒ 有运气成分 |

**⑤ 两条工程约束（比选型更容易翻车）**

- **天线**：优先**可拆 SMA 外置天线**款。理由同上：金属背包壳。
- **供电**：AR9271 约 350 mA、RTL8812AU 约 500 mA。Jetson 的 USB 口够用，
  但**不要接非供电 USB Hub**（供电不足的表现是「能识别、连一会儿就掉」）。

**一句话结论**：
要「今天下单、插上就能开热点」→ 买 **AIC8800DC 变体的 USB 网卡**（先问卖家 VID:PID）；
要「一次买对、信号最稳」→ 买 **AR9271 带可拆天线款（Alfa AWUS036NHA / TL-WN722N v1）**。
预算允许就**两张都买**（合计 ¥100 出头），到货按下面 30 秒验，哪张命中用哪张。

**⑥ 到手 30 秒验证（比任何商品页都可信）**

```bash
lsusb                                    # ① 对 VID:PID 白名单
ip -br link                              # ② 是否出现 wlan0
iw list | sed -n '/Supported interface modes/,/^$/p' | grep -c AP
                                         # ③ 输出 ≥1 才算「驱动真的支持 AP」
bash example/r1/high_level/scripts/backpack_ap.sh check   # ④ 一体化体检
```

第 ③ 条是**唯一的决定性判据**：`iw list` 里没有 `AP` 就没有，多花的钱和型号宣传都改变不了这一点。

### 5.5 方案 C：Mac 做热点 + UDP 中继（**零硬件，今天就能跑**）

现状：Mac 有两张网 —— **`en5 = 192.168.123.200`（有线，接在机器人内网交换机上，ping 背包 0.4 ms）**、
`en0`（无线，连办公网 `172.16.23.x`）。

> ⚠️ **别照抄 IP**：`en0` 的地址是 DHCP 分的、**会变**（2026-09-23 记录的是 `172.16.23.95`，
> 2026-10-08 已经是 `172.16.23.123`）。每次上机前现取：
> ```bash
> ipconfig getifaddr en0     # ← PICO 端要填的这个
> ipconfig getifaddr en5     # 应恒为 192.168.123.200
> ```

**C1 · 让 Mac 当 AP**：系统设置 → 通用 → 共享 → **互联网共享**，来源选 `en5`（机器人内网），
共享给 **Wi-Fi**。PICO 连上 Mac 的热点后，发往 `192.168.123.164:9999` 的包经 NAT 直达背包。
⚠️ 这会把 `en0` 从办公网切走。

**C2 · 不切网卡，只加一条中继（推荐，已实测可用）**：如果 PICO 能连上 Mac 所在的同一个 WiFi，
就让 PICO 把包发给 **Mac 的 WiFi 地址:9999**，Mac 上跑一条单向 UDP 中继转发到背包。
遥操数据是**单向**的（PICO → 背包；接收端绑 `INADDR_ANY` 且从不回包），
所以一条用户态中继就够，**不需要 root、不需要 NAT、不需要开转发**。

脚本（**已落地，不再是片段**）：`example/r1/high_level/scripts/pico_udp_relay.py`

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
$PY example/r1/high_level/scripts/pico_udp_relay.py            # 默认转发到 192.168.123.164:9999
$PY example/r1/high_level/scripts/pico_udp_relay.py --dump     # 额外打印每个包
caffeinate -i $PY example/r1/high_level/scripts/pico_udp_relay.py   # 顺带防休眠
```
启动时它会**自己打印出「PICO 端目标」该填哪个地址**（把本机所有非回环 IPv4 都列出来，挑与头显同网段的那个）。
首次运行 macOS 弹防火墙授权 → **选「允许」**。

**✅ 2026-10-08 端到端实证**（Mac 发 10 个包 → 中继 → 背包抓包）：
```
# 背包上：sudo tcpdump -n -i any udp port 9999 -c 10
10:20:35.305419 IP 192.168.123.200.57880 > 192.168.123.164.9999: UDP, length 74
…（10 包全到，0 dropped）
```
`192.168.123.200` 就是 Mac 的 `en5`，证明「中继 → 有线 → 背包」这一段是通的。

⚠️ **C2 的前提是办公/实验 WiFi 没开客户端隔离**（很多企业 AP 会开，导致 PICO 与 Mac 互相看不见）。
判据很直接：**看中继有没有打印计数**。
- 计数在涨 ⇒ PICO → Mac 通了；
- **一直为 0** ⇒ 大概率是客户端隔离 ⇒ 改用 C1（Mac 开热点）或方案 D（头显走有线）。

**C 方案的共同缺点**：Mac 必须在场且不能休眠（`caffeinate -i` 防睡）。

### 5.6 方案 D：头显转有线（最稳）

PICO 走 **USB-C 转网口**，插进机器人内网的交换机（和 Mac 的 `en5`、背包的 `eth10` 同一张二层网）。
有线无干扰、无掉包、不需要任何热点。目标地址 `192.168.123.164:9999`。
前提是头显系统支持 USB 网卡（多数 PICO 4 系列支持，需实测）。

### 5.7 ⚠️ 三条不能踩的线

1. **别去改 `cyclonedds.xml` 的 `eth0`→`eth10` 来"顺便解决网络"** —— 那是另一件事（ROS 菜单陷阱），
   改完会不会影响别的进程要单独评估。临时规避用 `env -u CYCLONEDDS_URI` 就够。
2. **别为了开 AP 去跑 `apt install hostapd`** —— 背包 apt 是坏的（版本错位），
   `apt --fix-broken install` 会真去升 systemd / python3.8。要走 hostapd 老路，只能本机下 deb → `scp` → `dpkg -i`。
3. **热点/NAT 不要动 `192.168.123.0/24`** —— DDS 走 `eth10`，网段冲突会让运控通信出问题。

---

## 6. 退出与收工

### 6.1 退出语义：**先阻尼，后回零，不是瞬停**

四条路径 —— `q`+回车 / `Ctrl+C`(SIGINT) / `kill`(SIGTERM) / SSH 掉线(SIGHUP) —— **完全等价**，
都进同一个 `goHomeAndRelease()`。真实动作顺序：

```
① loco 线程收尾：StopMove() + Damp()        ← 腿在这一刻开始变软（FSM 1）
② arm.goHomeAndRelease()：
     双臂目标置 0，等 |q| < 0.05 rad（最多 100 × 50 ms = 5 s，双臂真的摆回零位）
     → 再 2 s 线性把权重降到 0（交还运控）
③ hand.stop() → arm.stop()
```

**全程短则约 2 秒、长则约 7 秒**，最后打印 `[main] done.` 才算走完。

⚠️ 两条由此而来的实操警告：

1. **退出 = 底盘阻尼（腿变软）**，这是既定的收尾行为，不是故障。没有吊挂/支撑时，
   机器人会在"双臂还没回完零"的过程中就开始塌。
2. **退出期间双臂会主动摆回零位**（不是原地保持）⇒ 退出前**先清空臂展**，别让人扶着胳膊。

### 6.2 收工序列

```bash
# ① 先在程序里输 q + 回车（或 Ctrl+C），等它打印 [main] done.
example/r1/high_level/scripts/loco.sh status        # ② 复验：应为 FSM 1（阻尼）
sudo systemctl start r1-custom-head-remote          # ③ 恢复出厂服务
tmux kill-session -t teleop                         # ④ 或 Ctrl+B 再 D 脱离
```

> `loco.sh damp` 不用再补 —— 程序退出时已经 Damp 过了。

---

## 7. 红线清单（背下来）

| # | 红线 | 后果 |
|---|---|---|
| 1 | ⛔ **绝不 `kill -9`** | `SIGKILL` 无法捕获 ⇒ 权重停在 100 且不再发布 ⇒ **双臂冻结在最后一帧姿态**，不会回落给运控。真发生了就重跑程序再让它正常退出 |
| 2 | ⛔ **不与 `r1_arm_manual` 同时跑** | 两个程序互抢 `rt/arm_sdk` |
| 3 | ⛔ **`CYCLONEDDS_URI` 必须为空** | 非空 ⇒ DDS 静默超时，现象和"脚本坏了"一模一样 |
| 4 | ⛔ **每次切 FSM 后必须 `status` 复验** | `ret=0` 只说明 service 调用成功 |
| 5 | ⚠️ **启动/退出前清空臂展** | 启动时头/腰真实回零 3 秒；退出时双臂主动摆回零位 |
| 6 | ⚠️ **未停 `r1-custom-head-remote` 就启动** | 出厂服务抢头/腰关节，行为不可预期 |
| 7 | ⚠️ **tmux 里见到 `ros:foxy(1) noetic(2) ?` 按了 1** | 导出写死 `eth0` 的 `CYCLONEDDS_URI` ⇒ DDS 超时。**按回车** |

---

## 附录 A. 命令速查卡

```bash
# ——— 背包上 ———
cd ~/unitree_sdk2

# 起遥操（= 进入准备模式，按下回车即接管）
loco.sh status                    # ← 先确认是 811（程序默认不切 FSM）
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --auto-stand   # 仅需旧行为时加

# FSM（11 个子命令 + 4 个选项）
loco.sh status                    # 查 FSM ID / Mode（只读）
loco.sh watch 1                   # 每秒轮询 FSM（只读，Ctrl+C 退）
loco.sh stand                     # → FSM 4
loco.sh start                     # → FSM 811 主运控
loco.sh stop                      # 速度置零（不动 FSM）
loco.sh damp                      # → FSM 1 阻尼（安全态）
loco.sh move 0.1 0 0 3            # 速度移动（需输 YES）
loco.sh zero-torque               # → FSM 0（会瘫软，需输 YES）
loco.sh --dry-run stand           # 只看它将执行的原生命令，不真下发
# 另：speed <mode> / set-fsm <id> / menu；选项 --iface / -y / --dry-run

# 网络（PICO 通道）
bash example/r1/high_level/scripts/backpack_ap.sh check      # AP 体检（只读）
bash example/r1/high_level/scripts/backpack_ap.sh chips      # 该买哪块 USB 网卡
bash example/r1/high_level/scripts/backpack_ap.sh start R1Backpack '密码'   # 开热点
bash example/r1/high_level/scripts/backpack_ap.sh stop
bash example/r1/high_level/scripts/backpack_wifi.sh check    # （若背包插了网卡）连 WiFi 客户端

# 链路自检
ss -lun | grep 9999
echo 123 | sudo -S timeout 10 tcpdump -n -i any udp port 9999 -c 5

# ——— Mac 上（本机回归，不动机器人）———
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
./sim/check_syntax.sh                            # 语法校验（3 个目标应全 0 error）
./sim/build_probe.sh && ./sim/build/test_pico_parse   # 解析/安全判据单测
$PY sim/safety_scenarios.py                      # 7 个安全场景
$PY sim/check_pico_e2e.py                        # 合成报文 -> 全链路 -> 独立 FK 比对

# ——— Mac 上（假装头显，排练真机）———
$PY example/r1/high_level/scripts/pico_sim_sender.py --host 192.168.123.164 --port 9999 --duration 30 --hz 30
```

---

## 附录 B. 常见故障二分表

| 症状 | 最可能的三个原因（按概率） | 怎么确认 |
|---|---|---|
| 启动后双臂**纹丝不动**，目标在变但实测不动 | ① `--vel` 太小（指令推进被静摩擦吃掉）② FSM 不在 811 ③ 有别的进程抢 `arm_sdk` | 看 `p` 的「指令领先实测」：贴住 20° 上限 = 电机没跟上 |
| 程序 ready 了，但**一直没 `first packet`** | ① 网络通道不通 ② `operator_mode` 不是 `active_stream` ③ `quality != live` | `tcpdump` 有包 = 解析问题；无包 = 链路问题 |
| 包收得很正常，但**每包都阻尼、腿一直软** | 装了旧的 `:app`（`pico-openxr-bridge`，不发 `operator_mode`） | 看一次性 ⚠ 告警与 `first packet` 行的 `sdk=`；应为 `pico-openxr-single` |
| 已放行遥操，双臂却**不动且几乎无日志** | JSON 里没有 `pose` 对象（App「发送内容」页没勾） | 现在会打 1 Hz ⚠「已放行遥操但位姿缺失」 |
| 首包来了但**腿不跟着摇杆走** | 机器人不在 811（程序默认不再自动站立） | `loco.sh status`，不是 811 就 `loco.sh start` |
| 手臂跟着动，但**一停就变软** | 掉包 > 600 ms 触发了 `Damp()` | 检查发送端是否在持续发 |
| 急停后**怎么都不动** | 本端**不自动复位**（设计如此） | 按 stdin `s` 或 PICO 站立键 |
| 切 FSM 报成功但状态没变 | `ret=0` ≠ 状态已变 | 再跑一条 `loco.sh status` |
| 程序起不来 / DDS 超时 | `CYCLONEDDS_URI` 非空 | `echo "[$CYCLONEDDS_URI]"`，应为 `[]` |
| 编译"成功"但行为还是旧的 | `Clock skew` 让 `make` 静默跳过 | `stat -c "%y %n" build/bin/...`，并与源码 mtime 比 |
| 手动手臂时**某个关节不动** | 该关节阻力偏大 / 顶软限位 | 架构文档 §4.9.9 |
