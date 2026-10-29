# r1_hand —— 灵巧手 CANFD 直控（自包含运行目录）

> 这是**灵巧手控制那一整套**，一个目录装齐、可直接搬到机器人上跑，**不需要联网、不需要 pip**。
> 协议细节与架构 → 仓库根目录 `AGENTS.md` **§8**。
>
> **部署**：本目录在 `example/r1/` 里 ⇒ `deploy_backpack.sh --incremental` 会**一并同步**到背包的
> `~/unitree_sdk2/example/r1/high_level/scripts/r1_hand/`；背包上的 `~/r1_hand` 是指向它的**软链接**，
> 所以命令不用改，也不会出现"两份不一致"。（以前要单独 rsync 一次，**现在不用了**。）

## 硬件

LHandPro **`DH116S-L000-A1`(左) / `DH116S-R000-A1`(右)**，各 6 自由度，
USB-CANFD 适配器 **`a8fa:8598`**（Com Equipment "CANFD Analyser"，gs_usb 用户态协议）。

**两条独立的总线，各挂一只手，两只手的 node id 都是 `1`**：

```
USB-CANFD #0 ── 左手（node 1）
USB-CANFD #1 ── 右手（node 1）
```

> 2026-10-29 改的拓扑。以前是**一条总线挂两个节点（1/2）**，靠 node id 区分左右；
> 现在两条总线上的 CAN ID 完全相同（都 `0x501`/`0x481`），靠**适配器**区分。

| device_index | serial（实测） | 手 | 备注 |
|---|---|---|---|
| **0** | `F080207F308A3034` | **左手** | 2026-10-29 实测确认 |
| **1** | `F0802068387F4D4D` | **右手** | 2026-10-29 实测确认 |

> `device_index` 是**扫描顺序**，插拔/换口后**可能变**。若左右反了：
> ① 直接对调 `--left-bus/--right-bus`；② 或改 `bridge.sh` 顶部的 `LEFT_BUS/RIGHT_BUS`。
> 判据：起桥时日志先打 `[left] 使能/回零` —— 此时**左手**动就对，右手动就是反了。

## 文件

### 我们自己写的（改这些）

| 文件 | 作用 |
|---|---|
| `hand_canfd.py` | **核心封装** `LHandCanfd` + `build_payload()`：协议常量、回环/反馈分离、语义化 API、`start_feedback()` |
| `hand_bridge.py` | **桥进程**：独占 CANFD 适配器；启动时做初始化（使能→回零→位置模式→速度→电流），之后把遥操位置**直接映射**到手上 |
| `test_hand_canfd.py` | 编码器逐字节自检（对照厂商示例帧）→ 双手初始化 → 运动 → 总线统计 |
| `check_comms.py` | 只读通讯验证；`--scan` 按节点找出总线上有哪几只手上报 |
| `hand_canfd.sh` | **统一入口**：自动处理 macOS 的 libusb 路径、`python3 -u`（无缓冲）、依赖目录查找 |
| `bridge.sh` | **起桥的封装**：左右映射/节点号/端口作为默认值写在文件顶部，要调就改这里（也可用环境变量覆盖） |

### vendor 进来的（**不要改**）

| 文件 | 来源 | 许可 |
|---|---|---|
| `canfd_lib.py` | 厂商 CANFD 封装（随 LHandPro SDK 示例发布） | 厂商提供 |
| `gsusb_canfd/` | 上游 `gsusb-canfd` 的纯 Python 实现，**v0.1.3**（随 SDK 示例附带的源码副本） | 见上游仓库 |
| `usb/` | **pyusb 1.2.1** —— ⚠️ **必须是 1.2.1**：1.3.x 要求 Python ≥ 3.9，而背包是 3.8 | BSD |
| `hcanbus.rules` | udev 规则：`a8fa`/`8598` → `MODE=0666`（非 root 才能打开适配器） | 厂商提供 |
| `requirements.txt` | 上游列出的依赖（**仅作参考**：背包无网、apt 坏，实际靠上面的 vendor 副本） | — |

## 用法

```bash
# 0) 只读自检（不碰硬件）
./hand_canfd.sh test_hand_canfd.py --selfcheck

# 1) 看有几个适配器、各自的 serial（用来确认哪条总线是哪只手）
./hand_canfd.sh hand_bridge.py --list

# 2) 只读通讯验证（两条总线各验一次，不发运动指令）
./hand_canfd.sh check_comms.py --quiet
./hand_canfd.sh check_comms.py --scan --scan-max 4 --quiet    # 顺便确认节点号

# 3) 双手运动测试（会让手真的动）
./hand_canfd.sh test_hand_canfd.py --position 1000 --sweep

# 4) 起桥（给遥操程序用；必须早于遥操）——推荐用封装，默认值写在文件顶部
./bridge.sh

#    等价于（要临时调映射就用这个形式）：
#    ./hand_canfd.sh hand_bridge.py --left-bus 0 --right-bus 1
#    左右反了：./bridge.sh 前加 LEFT_BUS=1 RIGHT_BUS=0
```

> `--left-bus/--right-bus` 默认就是 `0/1`；`--node` 默认 `1`（两只手相同）。
> 只接一只手时把另一条设成 `-1`，例如 `--left-bus 0 --right-bus -1`。

配合遥操：**先起桥**，再起遥操程序并加 `--hand canfd`（见 `../R1_TELEOP_QUICKSTART.md`）。

## 平台差异（`hand_canfd.sh` 已自动处理）

- **macOS**：pyusb 需要 `DYLD_LIBRARY_PATH` 指向 Homebrew 的 libusb，
  否则报 `no USB backend available; is libusb-1.0 installed?`（本脚本会自动加上）。
- **Linux/背包**：系统自带 libusb，不用设；但**要装 udev 规则**否则非 root 打不开设备：
  ```bash
  sudo cp hcanbus.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger --action=add --subsystem-match=usb
  ```

## 依赖目录查找顺序

`hand_canfd.sh` 按这个顺序找 `canfd_lib.py` + `gsusb_canfd/`：

1. `$R1_HAND_DIR`（显式指定）
2. **脚本自己所在的目录**（= 本目录，仓库里直接可用）
3. `~/r1_hand`（背包上的部署位置）

## 四个最容易踩的坑

1. **哪条总线是哪只手**（双总线新拓扑）—— 两条总线上 node 都是 1、CAN ID 完全相同，
   只能靠 `device_index` 区分。`device_index` 是扫到适配器的**顺序**，插拔后可能变。
   ⇒ 起桥后**只动一只手**确认；搞反了把 `--left-bus/--right-bus` 对调即可。
2. **适配器把自己发的帧"回环"回来**（CAN ID 就是 `0x500+node`）—— 那不是手的反馈，
   必须与 `0x480+node` 的真反馈分开统计，否则会得出"手在应答"的假象。
3. **`00 02 50 01` 不是运动的前置条件** —— 它只是"开启异步反馈上报"，
   开了之后设备约 **1000 帧/秒**持续上报（逐帧打印会淹没一切，长跑记得 `log_frames=False`）。
4. **适配器 USB 不稳**：`a8fa:8598` 会反复 disconnect→重枚举（Artery MCU 的
   bootloader→firmware 切换）。掉了就重插。
