# R1 遥操 —— 最简启动流程

> **只有命令，没有解释。** 判据、排障、红线 → `R1_TELEOP_RUNBOOK.md`
> **前置**：机器人已吊挂/有支撑，臂展清空。
> 改过代码后的**同步 + 编译**不在本流程内（见 `R1_TELEOP_RUNBOOK.md` §3/§4）。

---

## ① Mac：起中继（**这个终端别关**）

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
cd /Users/plf/Project/RobotProject/unitree_sdk2
ipconfig getifaddr en0                 # ← 记下这个地址，等下填进头显
caffeinate -i "$PY" -u example/r1/high_level/scripts/pico_udp_relay.py
```

## ② 背包：切模式（**必须按顺序，不能直接 `start`**）

**阻尼 → 锁定站立 → 走跑**

```bash
ssh unitree@192.168.123.164
cd ~/unitree_sdk2

sudo systemctl stop r1-custom-head-remote      # 出厂服务会抢头/腰

example/r1/high_level/scripts/loco.sh damp     # ① → FSM 1    阻尼
example/r1/high_level/scripts/loco.sh stand    # ② → FSM 4    锁定站立
example/r1/high_level/scripts/loco.sh start    # ③ → FSM 811  走跑（主运控）
example/r1/high_level/scripts/loco.sh status   # 复验：必须 811
```

## ③ 背包：起桥 + 起遥操

```bash
# 灵巧手桥（必须早于遥操；启动时两只手会各回零一次，约 12 s）
tmux new -d -s hand "cd ~/r1_hand && ./hand_canfd.sh hand_bridge.py --left 1 --right 2 > /tmp/hand_bridge.log 2>&1"
sleep 15; grep "就绪" /tmp/hand_bridge.log       # 要看到：✅ 就绪：[1, 2] 已进入位置模式

# 遥操
tmux new -d -s teleop "cd ~/unitree_sdk2 && build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --hand canfd 2>&1 | tee /tmp/teleop.log"
sleep 12; grep -E "ready\.|桥驱动就绪" /tmp/teleop.log   # 要看到：[main] ready.
```

## ④ 头显

1. 装 **`:openxr-app`**（不是旧的 `:app`）
2. 目标地址 = **① 里记下的地址 + `:9999`**
3. 右手柄 **A** 校准 → 左 **X** 使能 → 左 **Y** 发送

## ⑤ 收工（**顺序别换**）

```bash
# 背包
tmux send-keys -t teleop q Enter        # 等它打印 [main] done.（约 2~7 s）
tmux send-keys -t hand C-c
sudo systemctl start r1-custom-head-remote
tmux kill-server

# Mac：终端 ① 按 Ctrl+C 停中继
```

---

## 三条判据（一行一个）

| 看到这个 | 说明 |
|---|---|
| 中继计数在涨 | PICO → Mac 通了 |
| `[pico] first packet … operator_mode="active_stream"` | PICO → 背包通了 |
| `[hand] CANFD 桥驱动就绪` + `[main] ready.` | 可以开始遥操了 |

## FSM 对照（`loco.sh status` 看 `current fsm_id`）

| FSM | `loco.sh` 命令 | 称呼 |
|---|---|---|
| `1` | `damp` | 阻尼（安全态） |
| `4` | `stand` | 锁定站立 |
| `811` | `start` | **走跑（主运控）** ← 遥操必须停在这里 |

## ⛔ 三件绝对不能做

1. **停遥操只能用 `q` / Ctrl+C，绝不 `kill -9`**（双臂会冻结在最后一帧姿态）。
2. **`CYCLONEDDS_URI` 必须为空**；tmux 里见 `ros:foxy(1) noetic(2) ?` **按回车**。
3. **不与 `r1_arm_manual` 同时跑**（互抢 `rt/arm_sdk`）。

## 操作中的按键（在 `tmux attach -t teleop` 里按）

`s` 站立 · `d` 阻尼 · `t` 停走 · `v vx vy vyaw` 给速度 · `q` 退出
