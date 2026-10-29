# R1 遥操 —— 最简启动流程

> **只有命令，没有解释。** 判据细节、排障二分表、红线全文 → `R1_TELEOP_RUNBOOK.md`
> 改过代码后的**同步 + 编译**不在本流程内（见 RUNBOOK §3/§4）。

**顺序**：① Mac 中继 → ② 背包切 FSM → ③ 背包起桥 → ④ 背包起遥操 → ⑤ 头显 → ⑥ 收工

**⛔ 前置（软件救不了）**
- 机器人**有吊挂或可靠支撑**，臂展内无人无物。
- 原因：遥操一启动，**头/腰会真实运动 3 秒回零**；**灵巧手桥启动时两只手各回零一次**；
  随后双臂被**满权重接管**（掰不动）。掉包/急停会让**腿变软**。

---

## ① Mac：起中继（**这个终端别关**）

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
cd /Users/plf/Project/RobotProject/unitree_sdk2
ipconfig getifaddr en0                 # ← 记下这个地址，等下填进头显
caffeinate -i "$PY" -u example/r1/high_level/scripts/pico_udp_relay.py
```

**看到**：`[relay] 单向 UDP 中继已启动` + 一行「PICO 端目标请填下列之一」——**照抄那行**，别手算 IP。

## ② 背包：切 FSM（**必须按这个顺序，不能直接 `start`**）

**阻尼 → 锁定站立 → 走跑**

```bash
ssh unitree@192.168.123.164
cd ~/unitree_sdk2

sudo systemctl stop r1-custom-head-remote      # 出厂服务会抢头/腰
example/r1/high_level/scripts/loco.sh damp     # ① → FSM 1    阻尼
example/r1/high_level/scripts/loco.sh stand    # ② → FSM 4    锁定站立（机器人会真的起身）
example/r1/high_level/scripts/loco.sh start    # ③ → FSM 811  走跑（主运控）
example/r1/high_level/scripts/loco.sh status   # 复验：必须 811
```

**看到**：`current fsm_id: 811`。`ret=0` ≠ 切成功，**只认 status**。

## ③ 背包：起灵巧手桥（**必须早于遥操**）

```bash
tmux new -d -s hand "cd ~/r1_hand && ./bridge.sh > /tmp/hand_bridge.log 2>&1"
sleep 15; grep "就绪" /tmp/hand_bridge.log
```

**看到**：`[bridge] ✅ 就绪：['left', 'right'] 已进入位置模式`

两条总线各挂一只手（`LEFT_BUS=0` 左手 / `RIGHT_BUS=1` 右手，写在 `~/r1_hand/bridge.sh` 顶部）。

> **顺手验一下左右**：日志里先打 `[left] 使能 … 回零`，**那一刻动的应该是左手**。
> 如果动的是右手 ⇒ 左右反了，用 `LEFT_BUS=1 RIGHT_BUS=0 ~/r1_hand/bridge.sh` 起。

## ④ 背包：起遥操

```bash
tmux new -d -s teleop "cd ~/unitree_sdk2 && build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --hand canfd 2>&1 | tee /tmp/teleop.log"
sleep 12; grep -E "ready\.|桥驱动就绪" /tmp/teleop.log
```

**看到**：`[hand] CANFD 桥驱动就绪 → udp://127.0.0.1:9998` + `[main] ready.`

## ⑤ 头显

1. 装 **`:openxr-app`**（**不是**旧的 `:app`）
2. 目标地址 = **① 里记下的地址 + `:9999`**
3. 右手柄 **A** 校准 → 左 **X** 使能 → 左 **Y** 发送

**看到**：`[pico] first packet … operator_mode="active_stream"`，接着双臂**慢慢**抬到遥操姿态
（`[arm] 缓起步…` → `[arm] 缓起步结束`，约 1~3 s —— 这是刻意设计的，不是卡顿）。

## ⑥ 收工（**顺序别换**）

```bash
# 背包
tmux send-keys -t teleop q Enter        # 等它打印 [main] done.（约 2~7 s）
tmux send-keys -t hand C-c
sudo systemctl start r1-custom-head-remote
tmux kill-server

# Mac：终端 ① 按 Ctrl+C 停中继
```

**看到**：`[arm] arm_sdk released (weight -> 0).` + `[hand] CANFD 桥驱动已停止` + `[main] done.`
（之后可能还有一句 CycloneDDS 断言 —— 发生在这三行**之后**，**忽略**）

---

## 判据速查（一行一个）

| 看到这个 | 说明 |
|---|---|
| 中继计数在涨 | PICO → Mac 通了 |
| `[bridge] ✅ 就绪：['left', 'right']` | 两条总线都连上、两只手都进了位置模式 |
| `[pico] first packet … operator_mode="active_stream"` | PICO → 背包通了 |
| `[hand] CANFD 桥驱动就绪` + `[main] ready.` | 可以开始遥操了 |
| `[arm] 缓起步：限速 1 rad/s` → `缓起步结束` | 首次接管**慢慢**抬到遥操姿态，正常 |

## FSM 对照（`loco.sh status` 看 `current fsm_id`）

| FSM | `loco.sh` 命令 | 称呼 |
|---|---|---|
| `0` | （ZeroTorque） | 零力矩 / 失力瘫软 |
| `1` | `damp` | 阻尼（安全态） |
| `4` | `stand` | 锁定站立 |
| `811` | `start` | **走跑（主运控）** ← 遥操必须停在这里 |

## 出问题先看这三条

| 现象 | 先查 |
|---|---|
| 中继计数一直 0 | 办公 WiFi 开了**客户端隔离** → 改 Mac 开热点 |
| 双臂动、手不动 | 头显里**没做校准（右手柄 A）** → 不进遥操；或 App「发送内容」没勾 `robot_control.hands` |
| 手左右颠倒 | `LEFT_BUS=1 RIGHT_BUS=0 ~/r1_hand/bridge.sh`（或改 `bridge.sh` 顶部两行） |

## ⛔ 三件绝对不能做

1. **停遥操只能用 `q` / Ctrl+C，绝不 `kill -9`** —— 权重会停在 100 且不再发布，**双臂冻结在最后一帧姿态**。
2. **`CYCLONEDDS_URI` 必须为空**；tmux 里见到 `ros:foxy(1) noetic(2) ?` **按回车，永不按 1**。
3. **不与 `r1_arm_manual` 同时跑** —— 两个程序互抢 `rt/arm_sdk`。

## 操作中的按键（`tmux attach -t teleop`）

`s` 站立 · `d` 阻尼 · `t` 停走 · `v vx vy vyaw` 给速度 · `q` 退出
