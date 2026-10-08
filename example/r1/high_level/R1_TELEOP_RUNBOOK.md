# R1 上半身 PICO 遥操 —— 启动指令 Runbook（单页速查）

> **本文件是"操作台前照着敲"的指令清单**，是 `R1_TELEOP_STARTUP.md`（41 KB 完整手册）的浓缩版。
> 两者冲突时，**以本文件为准**（它记录了 2026-10-08 的实测状态）。
> 原理与排查细节 → `R1_TELEOP_STARTUP.md`；背包/网络普查 → `R1_BACKPACK_ARCHITECTURE.md`。
>
> ⚠️ **凭据不写进任何仓库文件**：背包登录/`sudo` 密码见项目记忆，本文一律用 `<密码>` 占位。

---

## 0. 链路总览

```
PICO 头显 (:openxr-app)
   │  UDP → Mac 无线口 :9999
   ▼
Mac  en0 ──► pico_udp_relay.py ──► en5 (192.168.123.200)
   │  UDP → 192.168.123.164:9999      （有线直连，0.4 ms）
   ▼
背包 eth10 :9999 ──► r1_dual_arm_loco_skeleton ──► IK ──► rt/arm_sdk
```

| 角色 | 地址 |
|---|---|
| 背包（PC2） | `192.168.123.164`，用户 `unitree`，网卡 **`eth10`** |
| Mac 有线 `en5` | `192.168.123.200`（挂在机器人内网交换机上） |
| Mac 无线 `en0` | **DHCP，会变** ⇒ 每次现取（**PICO 要填这个**） |

---

## ⛔ 开工前（物理安全，软件救不了）

- 机器人**必须有吊挂或可靠支撑**；臂展内**无人无物**。
- 原因：程序一启动，**头/腰会在 3 秒内真实回零**，随后双臂被**满权重接管**（掰不动）。
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
caffeinate -i "$PY" example/r1/high_level/scripts/pico_udp_relay.py
```

它会自己打印 **「PICO 端目标请填下列之一」** —— **照抄那一行**，不要手算 IP。

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
  stat -c "%y %n" build/bin/r1_dual_arm_loco_skeleton example/r1/high_level/r1_dual_arm_loco.cpp'
```
判据：`grep` ≥ 1，且 `bin` 的 mtime **晚于**源码。
（✅ 用 `grep -ac`；❌ **不要**用 `strings | grep 中文` —— UTF-8 中文会被截断，永远返回 0。）

## 5️⃣ 背包：运行前检查

```bash
ssh unitree@192.168.123.164
cd ~/unitree_sdk2

echo "[$CYCLONEDDS_URI]"                       # 必须 []
sudo systemctl stop r1-custom-head-remote      # 出厂服务会抢头/腰关节
systemctl is-active r1-custom-head-remote      # 必须 inactive
ss -lun | grep 9999 || echo "(9999 空闲)"
```

## 6️⃣ 背包：切 811 主运控（**当前多半是 4，必须切**）

```bash
example/r1/high_level/scripts/loco.sh start
example/r1/high_level/scripts/loco.sh status     # 复验：必须 811
```

> 程序**不再自动切 FSM**（2026-09-29 起，对齐官方）。`ret=0` ≠ 切成功，**必须看 status**。

## 7️⃣ 背包：起遥操程序（**必须在 tmux 里**）

```bash
tmux new -s teleop
cd ~/unitree_sdk2
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5
```

就绪的标志：

```
[arm] controller ready.                ← 头/腰已回零，权重已 100
[pico] UDP receiver listening on 0.0.0.0:9999
[main] ready. 等待 PICO HandLink 数据... (stdin: 'v vx vy vyaw' / 's' / 'd' / 't' / 'q', Ctrl+C 退出)
```

## 8️⃣ 头显：开始发送

1. 确认装的是 **`:openxr-app`**（不是旧的 `:app`）
2. 目标地址 = **中继打印的那个**（形如 `172.16.23.x:9999`）
3. 右手柄 **A** 完成左右手校准（不做校准永远不进遥操，这是设计）
4. 左 **X** 使能 → 左 **Y** 开始发送

## 9️⃣ 判据：成功看这三行

```
[pico] first packet seq=… sdk="pico-openxr-single" operator_mode="active_stream"
[loco] PICO 首个可执行包：保持当前 FSM，不自动切档。底盘速度已解锁…
（双臂开始跟随）
```

可选，确认包真的落到背包（背包另开 window：`Ctrl+B` 再 `c`）：

```bash
echo <密码> | sudo -S timeout 10 tcpdump -n -i any udp port 9999 -c 5
```

## 🔟 收工（顺序别换）

```bash
# ① 在程序里输 q + 回车（或 Ctrl+C），等它打印 [main] done.（约 2~7 s）
example/r1/high_level/scripts/loco.sh status        # ② 应回 FSM 1（阻尼）
sudo systemctl start r1-custom-head-remote          # ③ 恢复出厂服务
tmux kill-session -t teleop                         # ④
# ⑤ Mac 上 Ctrl+C 停中继
```

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

### 「UDP 一停就疯狂打印 `[loco] Damp.`」—— 正常，不是故障

`tryGet()` 只要收到过**至少一个包**，就会永远返回**最后一个包**，而 `rx_age_ms` 单调增长。
于是主循环每帧（30 Hz）都判定「掉包 → 保持 → 触发阻尼」，`damp_cmd` 每帧置位一次：

```
rx_age_ms ≥ 600 ms → decideDisposition = kHold → holdTriggersDamp = true
                   → damp_cmd.store(true) → loco 线程 client.Damp() + 打印一行
```

- Damp 幂等，**重复发无害**；只是**没做边沿检测**（急停那条路有 `estop_active` 去重）。
- 会**一直打印**到 UDP 恢复或程序退出（`rx_age_ms` 只会变大）。
- **此时腿是软的（FSM 1），必须有支撑。**
- **怎么退出**：stdin 线程独立于刷屏，**直接打 `q` + 回车**即可（或 Ctrl+C / 关 SSH）。
- 只想止住刷屏而不退出：**恢复发送端**，`rx_age_ms` 落回 600 以下即恢复正常。

### 其它

| 现象 | 原因 | 处理 |
|---|---|---|
| 中继计数一直为 **0** | 办公 WiFi 开了**客户端隔离** | 改 Mac 开热点，或头显走有线 |
| 双臂纹丝不动 | ① FSM 不在 811 ② 权重没接管 ③ 位姿缺失（有 1 Hz ⚠ 告警） | 看日志 + `loco.sh status` |
| 每包都 Damp、腿一直软 | 装了旧 `:app`（`sdk=pico-openxr-bridge`） | 换装 `:openxr-app`；看 `first packet` 行的 `sdk=` |
| DDS 超时 / 程序起不来 | `CYCLONEDDS_URI` 非空 | `echo "[$CYCLONEDDS_URI]"` 应为 `[]` |
| 编译"成功"但行为没变 | 时钟偏差让 `make` 静默跳过 | 先校时（步骤 3），再核 `bin`/`src` mtime |

---

## ⛔ 红线

1. **绝不 `kill -9`** —— 权重停在 100 且不再发布 ⇒ 双臂**冻结在最后一帧姿态**。
2. **不与 `r1_arm_manual` 同时跑** —— 互抢 `rt/arm_sdk`。
3. **`CYCLONEDDS_URI` 必须为空**；tmux 里见 `ros:foxy(1) noetic(2) ?` **按回车，永不按 1**。
4. **起程序前必须已在 811**；每次切 FSM 后**必须 status 复验**。
5. **启动/退出前清空臂展** —— 启动头/腰回零 3 s；退出双臂主动摆回零位。
6. **背包 apt 是坏的**、无外网 DNS —— 别跑 `apt`，装包只能 `.deb` + `dpkg -i`。
7. **别再用那块 AR9271 USB 无线网卡** —— 一关联就内核挂死 + 120 s 看门狗重启（实测 4 次）。
