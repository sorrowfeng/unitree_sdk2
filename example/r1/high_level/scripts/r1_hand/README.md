# r1_hand —— 灵巧手 CANFD 直控（自包含运行目录）

> 这是**灵巧手控制那一整套**，一个目录装齐、可直接搬到机器人上跑，**不需要联网、不需要 pip**。
> 协议细节与架构 → 仓库根目录 `AGENTS.md` **§8**。
>
> **部署映射**：本目录 ⟷ 背包上的 `~/r1_hand/`
> ```bash
> # 从仓库同步到背包（在仓库根目录执行）
> example/r1/high_level/scripts/_vm_rsync.exp <密码> 22 unitree 192.168.123.164 \
>   example/r1/high_level/scripts/r1_hand/ /home/unitree/r1_hand/
> ```

## 硬件

LHandPro **`DH116S-L000-A1`(左) / `DH116S-R000-A1`(右)**，各 6 自由度，
USB-CANFD 适配器 **`a8fa:8598`**（Com Equipment "CANFD Analyser"，gs_usb 用户态协议）。
两只手挂**同一条总线**，靠 **node id** 区分（默认 **1=左 / 2=右**）。

## 文件

### 我们自己写的（改这些）

| 文件 | 作用 |
|---|---|
| `hand_canfd.py` | **核心封装** `LHandCanfd` + `build_payload()`：协议常量、回环/反馈分离、语义化 API、`start_feedback()` |
| `hand_bridge.py` | **桥进程**：独占 CANFD 适配器；启动时做初始化（使能→回零→位置模式→速度→电流），之后把遥操位置**直接映射**到手上 |
| `test_hand_canfd.py` | 编码器逐字节自检（对照厂商示例帧）→ 双手初始化 → 运动 → 总线统计 |
| `check_comms.py` | 只读通讯验证；`--scan` 按节点找出总线上有哪几只手上报 |
| `hand_canfd.sh` | **统一入口**：自动处理 macOS 的 libusb 路径、`python3 -u`（无缓冲）、依赖目录查找 |

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

# 1) 找总线上有哪几只手
./hand_canfd.sh check_comms.py --scan --scan-max 4 --quiet

# 2) 双手运动测试（会让手真的动）
./hand_canfd.sh test_hand_canfd.py --nodes 1 2 --position 1000 --sweep

# 3) 起桥（给遥操程序用；必须早于遥操）
./hand_canfd.sh hand_bridge.py --left 1 --right 2
```

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

## 三个最容易踩的坑

1. **适配器把自己发的帧"回环"回来**（CAN ID 就是 `0x500+node`）—— 那不是手的反馈，
   必须与 `0x480+node` 的真反馈分开统计，否则会得出"手在应答"的假象。
2. **`00 02 50 01` 不是运动的前置条件** —— 它只是"开启异步反馈上报"，
   开了之后设备约 **1000 帧/秒**持续上报（逐帧打印会淹没一切，长跑记得 `log_frames=False`）。
3. **适配器 USB 不稳**：`a8fa:8598` 会反复 disconnect→重枚举（Artery MCU 的
   bootloader→firmware 切换）。掉了就重插。
