# R1 上半身 PICO 遥操 —— 启动指令 Runbook（单页速查）

> 🚀 **只想"照着敲一遍把遥操跑起来"→ 用 `R1_TELEOP_QUICKSTART.md`**（更短，只有命令）。
> 本文件在它之上多了**判据、常见现象二分表、红线**，适合出问题时对照。
>
> **本文件是"操作台前照着敲"的指令清单**，是 `R1_TELEOP_STARTUP.md`（完整手册）的浓缩版。
> 两者冲突时，**以本文件为准**（它记录了 2026-10-09 的实测状态：双臂 + **灵巧手**）。
> 原理与排查细节 → `R1_TELEOP_STARTUP.md`；灵巧手协议/架构 → `AGENTS.md` §8。
>
> ⚠️ **凭据不写进任何仓库文件**：背包登录/`sudo` 密码见项目记忆，本文一律用 `<密码>` 占位。

---

## 0. 链路总览

```
PICO 头显 (:openxr-app)                    ┌─ 双臂：rt/arm_sdk（运控管腿）
   │  UDP → Mac 无线口 :9999               │
   ▼                                       │
Mac  en0 ──► pico_udp_relay.py ──► en5     │
   │  UDP → 192.168.123.164:9999           │
   ▼                                       │
背包 eth10 :9999 ──► r1_dual_arm_loco_skeleton ──┤
                                          └─ 灵巧手：UDP 127.0.0.1:9998
                                                    └─► hand_bridge.py ──CANFD──► 两只手
```

| 角色 | 地址 |
|---|---|
| 背包（PC2） | `192.168.123.164`，用户 `unitree`，网卡 **`eth10`** |
| Mac 有线 `en5` | `192.168.123.200`（挂在机器人内网交换机上） |
| Mac 无线 `en0` | **DHCP，会变** ⇒ 每次现取（**PICO 要填这个**） |
| PICO 目标 | **`<Mac 的 en0 地址>:9999`** |
| 灵巧手 | LHandPro `DH116S` 两只，USB-CANFD `a8fa:8598`，node id **1(左) / 2(右)** |

---

## ⛔ 开工前（物理安全，软件救不了）

- 机器人**必须有吊挂或可靠支撑**；臂展内**无人无物**。
- 原因：遥操程序一启动，**头/腰会在 3 秒内真实回零**，随后双臂被**满权重接管**（掰不动）；
  **灵巧手桥启动时两只手会各回零一次**。
- 掉包/急停会让**腿变软**（FSM 1）。

---

## 1️⃣ Mac：网络自检

```bash
ipconfig getifaddr en5        # 必须是 192.168.123.200
ping -c 3 192.168.123.164     # 必须通（约 0.4 ms）
ipconfig getifaddr en0        # ← 记下来，待会儿填进头显
```

## 2️⃣ Mac：起 UDP 中继（**保持这个终端不关**）

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python
cd /Users/plf/Project/RobotProject/unitree_sdk2
caffeinate -i "$PY" -u example/r1/high_level/scripts/pico_udp_relay.py
```

它会自己打印 **「PICO 端目标请填下列之一」** —— **照抄那一行**，不要手算 IP。
**判据**：头显开始发送后，这里会持续打印计数（约 30~80 包/秒）。**一直为 0 ⇒ WiFi 客户端隔离**。

## 3️⃣ 背包：校时（**改过代码后必做**，否则 `make` 静默跳过重编）

```bash
# ⚠️ 在 Mac 上执行：$(date) 必须在 Mac 展开，不是在背包上
ssh unitree@192.168.123.164 "sudo date -s '$(date '+%Y-%m-%d %H:%M:%S')'"
ssh unitree@192.168.123.164 'date'      # 与 Mac 对比，应一致
```

## 4️⃣ Mac：同步代码 + 编译（**仅当改过 C++ 时**）

```bash
cd /Users/plf/Project/RobotProject/unitree_sdk2
BACKPACK_PASS=<密码> example/r1/high_level/scripts/deploy_backpack.sh --incremental
```

验证新代码**真的进了产物**（别省）：

```bash
ssh unitree@192.168.123.164 'cd ~/unitree_sdk2 && \
  grep -ac "保持当前 FSM" build/bin/r1_dual_arm_loco_skeleton && \
  grep -ac "hand_bridge.py"  build/bin/r1_dual_arm_loco_skeleton && \
  stat -c "%y %n" build/bin/r1_dual_arm_loco_skeleton example/r1/high_level/r1_dual_arm_loco.cpp'
```
判据：两个 `grep` 都 ≥ 1，且 `bin` 的 mtime **晚于**源码。
（✅ 用 `grep -ac`；❌ **不要**用 `strings | grep 中文` —— UTF-8 中文会被截断，永远返回 0。）

**灵巧手脚本也要同步**（它们不在 CMake 里，deploy 不会带）：

```bash
# _vm_rsync.exp 的签名：<密码> <port> <user> <host> <src> <dst>
example/r1/high_level/scripts/_vm_rsync.exp <密码> 22 unitree 192.168.123.164 \
  ~/r1_hand/ /home/unitree/r1_hand/
```
> 依赖（`canfd_lib.py` + `gsusb_canfd/` + `usb/`）在 `~/r1_hand/` 里，**纯 Python 无需 pip**。
> ⚠️ pyusb 必须是 **1.2.1**（1.3.x 要求 Python ≥ 3.9，背包是 3.8）。

## 5️⃣ 背包：运行前检查

```bash
ssh unitree@192.168.123.164
cd ~/unitree_sdk2

echo "[$CYCLONEDDS_URI]"                       # 必须 []
sudo systemctl stop r1-custom-head-remote      # 出厂服务会抢头/腰关节
systemctl is-active r1-custom-head-remote      # 必须 inactive
ss -lun | grep -E "9998|9999" || echo "(端口空闲)"
lsusb | grep a8fa                              # 灵巧手 CANFD 适配器在不在
```

## 6️⃣ 背包：把机器人弄到 811 主运控

**当前可能是 FSM 0（完全失力/瘫软）**——那就先让它站起来，别直接 `start`：

```bash
example/r1/high_level/scripts/loco.sh status    # 先看当前 FSM
# 若是 0（失力）或 1（阻尼）：先站起来（⚠️ 确认有支撑）
example/r1/high_level/scripts/loco.sh damp      # → FSM 1 安全态
example/r1/high_level/scripts/loco.sh stand     # → FSM 4 站立（机器人会真的起身）

# 再到 811
example/r1/high_level/scripts/loco.sh start     # → FSM 811 主运控
example/r1/high_level/scripts/loco.sh status    # 复验：必须 811
```

> 程序**不再自动切 FSM**（2026-09-29 起，对齐官方）。`ret=0` ≠ 切成功，**必须看 status**。

## 7️⃣ 背包：起灵巧手桥（**必须在遥操之前**）

```bash
tmux new-session -d -s hand \
  "cd ~/r1_hand && ./hand_canfd.sh hand_bridge.py --left 1 --right 2 > /tmp/hand_bridge.log 2>&1"
sleep 15 && cat /tmp/hand_bridge.log
```

看到这一行才算就绪（两只手回零 + 进位置模式，约 12 s）：
```
[bridge] ✅ 就绪：[1, 2] 已进入位置模式，开始接收遥操位置 （下发 ≤50 Hz，1s 无新位置即保持）
```

> 之后遥操程序只把位置发到 UDP 9998，由桥落到 CAN。**桥先起、遥操后起**。
> 想调试可加 `--verbose`（逐帧打印；⚠️ 开了反馈后约 1000 帧/秒会刷屏）。

## 8️⃣ 背包：起遥操程序（**必须在 tmux 里**）

```bash
tmux new-session -d -s teleop \
  "cd ~/unitree_sdk2 && build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5 --hand canfd 2>&1 | tee /tmp/teleop.log"
sleep 12 && cat /tmp/teleop.log
```

就绪的标志（注意 `[hand] CANFD 桥驱动就绪`）：
```
[arm] controller ready.                ← 头/腰已回零，权重已 100
[hand] CANFD 桥驱动就绪 → udp://127.0.0.1:9998（位置由 hand_bridge.py 落到 CAN；kIdle 时不发送）
[pico] UDP receiver listening on 0.0.0.0:9999
[main] ready. 等待 PICO HandLink 数据... (stdin: 'v vx vy vyaw' / 's' / 'd' / 't' / 'q', Ctrl+C 退出)
```

> **不加 `--hand` 时行为完全不变**（默认 `null`，只打印 6 路 raw 验证链路）。

## 9️⃣ 头显：开始发送

1. 确认装的是 **`:openxr-app`**（不是旧的 `:app`）
2. 目标地址 = **中继打印的那个**（Mac 的 `en0` 地址，形如 `172.16.23.x:9999`）
3. 右手柄 **A** 完成左右手校准（不做校准永远不进遥操，这是设计）
4. 左 **X** 使能 → 左 **Y** 开始发送

## 🔟 判据：成功看这几行

```
[pico] first packet seq=… sdk="pico-openxr-single" operator_mode="active_stream"
[loco] PICO 首个可执行包：保持当前 FSM，不自动切档。底盘速度已解锁…
（双臂开始跟随；两只手跟随 PICO 手部位置）
```

**灵巧手是否真的在收位置**（可选，不影响运行）：
```bash
# 看桥收了多少帧；遥操退出时也会打印桥发/收帧数
pgrep -af hand_bridge && tail -3 /tmp/hand_bridge.log
```
正常退出时两边帧数应基本相等，例如：
```
[hand] CANFD 桥驱动已停止（发送 720 帧, 失败 0 次）      ← C++ 侧
[bridge] 关闭。RX 帧数=719，设备反馈={1: 207957, 2: 207957}  ← 桥侧
```

可选，确认包真的落到背包（背包另开 window：`Ctrl+B` 再 `c`）：
```bash
echo <密码> | sudo -S timeout 10 tcpdump -n -i any udp port 9999 -c 5
```

## 1️⃣1️⃣ 收工（**顺序别换**）

```bash
# ① 先停遥操：给它发 q（走清理路径：阻尼 → 双臂回零 → 降权）
tmux send-keys -t teleop q Enter
#    等它打印 [main] done. 才算走完（约 2~7 s）
#    ⚠️ 绝不能 kill -9（见红线）

# ② 停灵巧手桥
tmux send-keys -t hand C-c

# ③ 复验 + 恢复出厂服务
example/r1/high_level/scripts/loco.sh status        # 应回 FSM 1（阻尼）
sudo systemctl start r1-custom-head-remote
tmux kill-server 2>/dev/null                        # 清掉所有 tmux 会话

# ④ Mac 上 Ctrl+C 停中继
```

> 灵巧手**不会**自动松手或归零：桥停了以后手**保持在最后位置**（与双臂"保持"一致）。

---

## 操作中能按的键（stdin，**只在 tmux / `ssh -t` 里有效**）

| 键 | 作用 |
|---|---|
| `s` | 站立（→ FSM 4）。**急停/掉包后必须靠它恢复** —— 本端不自动复位 |
| `d` | 阻尼（→ FSM 1，腿变软） |
| `t` | 停走（速度归零，不动 FSM） |
| `v vx vy vyaw` | 直接给底盘速度 |
| `q` | 交还权重并退出（≡ Ctrl+C） |

---

## 常见现象

### 双臂动、手不动（或手不动）
**首选查 `safe_to_execute`**：PICO 端 App 的 8 项 AND 闸门（连续发送 + 已使能 + 未急停 +
session running/focused + **左右手都已校准** + 追踪 live + 样本新鲜 ≤250 ms）。
**头显里没做校准（右手柄 A）就永远不进遥操**，这是设计。
另外确认 App「发送内容」页勾了 `robot_control.hands`（否则报文本里没有 `hands`，手不动）。

### 「UDP 一停就疯狂打印 `[loco] Damp.`」—— 正常，不是故障
`tryGet()` 只要收到过**至少一个**包，就会永远返回**最后一个包**，而 `rx_age_ms` 单调增长
⇒ 每帧（30 Hz）都判「掉包 → 保持 → 触发阻尼」。Damp 幂等，**重复发无害**，只是**没做边沿检测**。
**退出不受影响**：stdin 线程独立于刷屏，**直接打 `q` + 回车**即可。

### 其它

| 现象 | 原因 | 处理 |
|---|---|---|
| 中继计数一直为 **0** | 办公 WiFi 开了**客户端隔离** | 改 Mac 开热点，或头显走有线 |
| `[bridge] 未找到 CANFD 适配器` | 适配器 USB 掉了（`a8fa:8598` 会反复 disconnect） | **重插**；确认 udev 规则 `hcanbus.rules` 在 |
| 桥起来了但 **node 2 不响应** | 第二只手供电 / 总线瞬断 | `check_comms.py --scan` 复验；必要时重插 |
| 桥一直没输出 | stdout 被块缓冲 | 统一走 `hand_canfd.sh`（内含 `python3 -u`） |
| 双臂纹丝不动 | ① FSM 不在 811 ② 权重没接管 ③ 位姿缺失（有 1 Hz ⚠） | 看日志 + `loco.sh status` |
| 每包都 Damp、腿一直软 | 装了旧 `:app`（`sdk=pico-openxr-bridge`） | 换装 `:openxr-app`；看 `first packet` 的 `sdk=` |
| 退出末尾有 CycloneDDS 断言 | DDSCXX 静态析构期报错，**发生在 `[main] done.` 之后** | **忽略**：`[main] done.` 才是收尾完成的判据 |
| DDS 超时 / 程序起不来 | `CYCLONEDDS_URI` 非空 | `echo "[$CYCLONEDDS_URI]"` 应为 `[]` |
| 编译"成功"但行为没变 | 时钟偏差让 `make` 静默跳过 | 先校时（步骤 3），再核 `bin`/`src` mtime |

---

## ⛔ 红线

1. **绝不 `kill -9`** —— 权重停在 100 且不再发布 ⇒ 双臂**冻结在最后一帧姿态**。
   停遥操**只用 `q` / Ctrl+C / SIGTERM**。
2. **不与 `r1_arm_manual` 同时跑** —— 互抢 `rt/arm_sdk`。
3. **`CYCLONEDDS_URI` 必须为空**；tmux 里见 `ros:foxy(1) noetic(2) ?` **按回车，永不按 1**。
4. **起程序前必须已在 811**；每次切 FSM 后**必须 status 复验**。
5. **启动/退出前清空臂展** —— 启动头/腰回零 3 s + 灵巧手回零；退出双臂主动摆回零位。
6. **背包 apt 是坏的**、无外网 DNS —— 别跑 `apt`，装包只能 `.deb` + `dpkg -i`。
7. **别再用那块 AR9271 USB 无线网卡** —— 一关联就内核挂死 + 120 s 看门狗重启（实测 4 次）。
