# R1-EDU 计算单元架构与算力背包（PC2）文件系统档案

> 实测日期：**2026-09-23**，对象：R1-EDU 算力背包，`192.168.123.164`，hostname `ubuntu`。
> 全部结论来自只读普查（`ls` / `cat` / `ss` / `ip neigh` / `systemctl`），**未在背包上编译或运行任何自研程序**。
> 涉及「别改什么」的判断，请与 `~/r1_custom_head/DEPLOYMENT_INFO.txt` 的既有安全策略对照阅读。

---

## 1. 先厘清命名：PC1 / PC2 / PC4 是三个不同层级的叫法

机器人内部是**一台交换机构成的 `192.168.123.0/24` 内网**，网关 `192.168.123.1`。
这跟官方文档一致：R1-EDU「运控计算单元 = `192.168.123.161`，不对外开放；开发计算单元 = `192.168.123.164`，`unitree/123`」。

| 叫法 | 地址 | 实际是什么 | 实测证据 |
|---|---|---|---|
| **PC1** | `192.168.123.161` | **运控计算单元**，Unitree 运动控制程序专用，**不允许 SSH 登录** | ARP 表里 **REACHABLE**（有设备应答），但 22/80/443/1026/4000 等 **全部端口不通** —— 正是「只跑 DDS、不开放登录」的特征 |
| **PC2** | `192.168.123.164` | **开发计算单元**（= 本机 / 算力背包），用户二次开发的唯一入口 | `eth10` 静态 `192.168.123.164/24`（netplan 管理），SSH 可登录 |
| **`pc4`** | 同上 | **宇树 OTA 体系里对这台机器的内部编号** | `/unitree/ota/pipe/ota_pipe_service.json` → `"HostAlias": "pc4"`；模块名 `master_service_pc4` / `unitree_patch_pc4` |

> ⚠️ 别被 `pc4` 绕晕：宇树把「运控单元」固定叫 **PC1**，而把后续的开发单元按 **PC2 / PC3 / PC4** 编号，
> 其 IP 分别是 `.162 / .163 / .164`。本机在 `.164`，所以 OTA 里就是 `pc4`；但从角色上它就是**开发计算单元（即你说的 PC2）**。
> 看到 `pc4` 的日志/模块名，指的就是本机，**不是第四台机器**。

### 通讯关系

- **我们（笔记本 `192.168.123.200`，走 `en5` 有线）→ PC2**：SSH + rsync 部署。
- **PC2 → PC1**：**CycloneDDS**（UDP 组播，不是 TCP）。SDK 里 `LocoClient` / `AudioClient` 的 `sport`/`voice` 服务、
  `rt/lowstate`、`rt/arm_sdk` 全在 PC1 侧终结。
- **本机没有任何到 PC1 的 TCP 通路** —— 这是设计如此，不是故障。诊断「机器人为什么不响应」时，
  不要用 `ping`/`nc` 去试 PC1（该机也禁 ICMP），要看 **DDS 有没有通**（`loco.sh status` 之类）。
- 当前本机唯一的 DDS 参与者是 `r1_remote_actions`（占 `udp 7400/7401`），即下面第 3 节的头/腰服务。

---

## 2. PC2 文件系统：三层结构

这台机器的文件布局在「谁可以改」这件事上切得很干净：**`/unitree` + 出厂 systemd 属 root 出厂层，碰不得；
`/home/unitree` 才是开发者的地盘。**

### ① 出厂系统层（root，禁改）

| 路径 / 服务 | 作用 |
|---|---|
| `/version.txt` | `unitree_r1_nx_v2603` —— 出厂镜像版本（`nx` = Orin **NX**，与实测板型相符） |
| `/unitree/` | 宇树出厂框架根目录，**9 个子目录**，见下 |
| `/unitree/bin` `json_checker`、`/unitree/sbin/{mscli, ota_pipe_cli, start-stop-daemon}` | 出厂 CLI。`mscli` = 主服务控制台，`ota_pipe_cli` = OTA 通道客户端 |
| `/unitree/module/master_service/{master_service, *.json}` | **主服务守护进程**（`master_service.json` 是**加密/二进制**，`cat` 出来是乱码，属正常） |
| `/unitree/etc/master_service/` | 主服务的策略目录。实测内容：`init`/`once` 均为 `["ota-pipe","ota-update","pd-init"]`（开机跑的三件事）；`protect` = `{"slam_nav": 0}`；`forbid`/`conflict`/`manual`/`prio`/`path` 均空 |
| `/unitree/etc/master_service/cmd/{am-init,core-init,ota-pipe,ota-update,pd-init}` | 开机动作定义。例如 `am-init` = `amixer set Speaker 75%`（**扬声器音量**）；`ota-pipe` = 从备份恢复 OTA 通道二进制；`pd-init` = 清理陈旧的 `.pid` |
| `/unitree/ota/` | OTA 更新通道：`pipe/ota_pipe_service`（二进制，`tcp://192.168.123.164:1026` 监听）、`update/module/unitree_patch_pc4`、`backup/module/unitree_patch_pc4`（升级前备份） |
| `/unitree/var/` | 运行期数据：`run/master_service.sock`、`log/master_service/`（实测该日志已 14 MB） |
| `/unitree/robot/pkg/{module,version}` | 已安装模块清单 + 版本。实测：`master_service_pc4: 1.0.0.2`、`unitree_patch_pc4: 1.0.0.1` |
| `/unitree/` 的 LSB 服务 | `/etc/init.d/{master_service, ota_pipe}` —— **root 守护，别 stop** |
| `/opt/ota_package/{t19x, t23x}` | L4T **BSP 固件包**（bootloader / kernel payload）。`t23x` = Orin 系列，`t19x` = Xavier 系列；两者都在，共 356 MB |
| `/etc/rc.local` | 硬件初始化：从 `/dev/rtc1` 同步时钟、`devmem 0x02430030`、导出并拉高 `PP.06` GPIO（**风扇/电源相关脚**） |
| `/etc/systemd/system/nv*.service`（约 20 个） | NVIDIA L4T 原生服务（`nvfancontrol` 风扇、`nvpmodel` 功耗模式、`nvfb`/`nvweston` 显示、`nvargus-daemon` 相机、`nvzramconfig` 内存压缩…） |
| `/opt/ros/{foxy, noetic}` | **ROS2 Foxy 与 ROS1 Noetic 同时预装**（宇树的标配镜像） |

### ② 平台与框架层（发行镜像预装，一般不直接改）

| 路径 | 作用 |
|---|---|
| `/usr/local/cuda-11.4` | CUDA 工具链（JetPack 5.1 标配）。`TensorRT 8.5.2.2` 实测可 import ⇒ 这台机器**有完整的推理能力** |
| `/usr/local/gtsam_toolbox` | GTSAM 因子图优化（SLAM / 位姿图用） |
| `~/cyclonedds_ws` | ROS2 用的 CycloneDDS 工作区（`build` / `install` / `src` / `cyclonedds.xml`） |
| `~/unitree/Odometer_service` | **ROS1 catkin 工作区**，内含 `rpg_svo_pro_open`（半直接法视觉里程计 SVO）、`ceres_catkin`、`opengv`、`dbow2`、`eigen_*`、`minkindr` ⇒ 一套 **VIO / 视觉里程计** 部署，共 604 MB |
| docker / containerd | 已装（`docker0` 存在），但**当前无运行中的容器**（`docker0` 无载波、路由表显示 `linkdown`） |
| NoMachine（`nxserver.service`、`~/nomachine.sh`、`~/.nx`） | 远程图形桌面。`nomachine.sh` 会先 `systemctl stop gdm3` 再 `init 3`，把机器切到无显示器的运行级别 |

> ⚠️ **一个已存在的配置 bug**：`~/cyclonedds_ws/cyclonedds.xml` 里写的是
> `<NetworkInterface name="eth0" .../>`，**但这台机器的网卡叫 `eth10`**（不是 Jetson 常见的 `eth0`）。
> 任何 source 了 `~/cyclonedds_ws/install/setup.bash` 的 ROS2 程序，都会绑到一个**不存在的网卡**上，DDS 发现不了对端。
> 我们的 C++ SDK 不使用这个文件（`ChannelFactory::Init(0, "eth10")` 直接指定接口），所以不受影响；
> 但若日后要在背包上跑 ROS2 的 `teleimager` / `unitree_ros2`，**必须先把这个名字改成 `eth10`**。

### ③ 用户业务层（`/home/unitree`，开发者的地盘）

家目录 40 个条目，按用途分成 6 类：

#### (a) SDK 副本 —— 三份并存（2026-09-23 已定角色）

| 目录 | 体积 | 角色（**认准这一列**） |
|---|---|---|
| **`unitree_sdk2`** | 352 MB | **宇树工厂原状，已恢复**。git 仓库，`main` @ `fa925bf`（2026-03-25），含已编好的 `build/`（39 个官方样例二进制，其中有 `r1_loco_client`、`r1_ankle_swing_example`）。**只读：回滚锚点 + 参考，绝不往这里同步我们的代码** |
| **`r1_teleop_sdk2`** | 86 MB | **我们的工作树**（SDK 骨架 = 上游 `origin/main` `63096d0` 2026-09-21 + 我们的改动）。**唯一的编译/运行来源**。当前无 `build/`（尚未编译） |
| `unitree_sdk2-main` | 84 MB | 2026-02-03 解压的官方 zip 副本（**无 `.git`**），另有 `unitree_sdk2-main.zip` 17 MB |

> 三份 SDK 并存 = 「供应商镜像 + 手工解压 + 自定义部署」的叠加痕迹。
> ⚠️ `~/r1_custom_head/DEPLOYMENT_INFO.txt` 明确要求 **不要改 `unitree_sdk2` 与 `unitree_sdk2-main`** ——
> 我们的代码现已固定在**独立目录 `~/r1_teleop_sdk2`**，与工厂原状彻底分开。
> `deploy_backpack.sh` / `deploy.sh` 支持 `VM_DIR=` 覆盖，默认已指向 `~/r1_teleop_sdk2`。

**为什么不能「复用工厂树的 SDK 当依赖」（2026-09-23 实测判死）**：
工厂版是 `fa925bf`（2026-03），我们的基线是 `63096d0`（2026-09）。两者之间我们**直接 include 的头文件**
发生了**新增**，不是小改：

| 头文件 | 工厂版 → 我们的版本 |
|---|---|
| `include/unitree/dds_wrapper/robots/r1/r1.h` | **新增文件**（工厂版没有） |
| `include/unitree/robot/r1/audio/audio_client.hpp` `audio_api.hpp` `audio_error.hpp` | **新增文件** |
| `include/unitree/dds_wrapper/robots/r1/r1_pub.h` / `defines.h` | 新增 / +56 行 |
| `include/unitree/robot/b2/robot_state/robot_state_client.hpp` | +1 行（`GetPkgVersion`） |
| `lib/{aarch64,x86_64}/libunitree_sdk2.a` | md5 不同（`f0de1d61…` vs `61cc6bf5…`） |

⇒ **工厂树的 `include/` 里连我们 `#include` 的头文件都不存在**，拿它当 SDK 必然编译失败。
结论：**我们的工作树必须自带一份与基线一致的 SDK 骨架**，这份 86 MB 副本消不掉。

**顺手量出来的减重空间**：`lib/x86_64/` 27 MB 在 aarch64 背包上**完全用不到**，可安全删除
（86 MB → 59 MB）。`example/` 里与我们无关的子目录加起来只有 ~3.6 MB，删了收益太小、且会让
我们的树与上游分叉更多，**不建议删**。

#### (b) 头/腰 展示类（**当前唯一在跑的我们的相关程序**）

| 目录 | 说明 |
|---|---|
| `r1_custom_head` (60 MB) | `r1_remote_actions eth10` + `r1_head_show`。订阅 `rt/lowstate` 读遥控器，`SELECT+DOWN/LEFT/RIGHT` → 展示 / 点头×3 / 摇头×3。**接管头/腰**（`kp_head=15`）。由 `r1-custom-head-remote.service`（enabled + `Restart=always`）托管。自带独立 `vendor/unitree_sdk2` 副本 —— 所以它**不依赖** `~/unitree_sdk2` |

> 它占用 `udp 7400/7401/51002/36715`。**我们的遥操程序与它抢同一组关节**，
> 正式运行前应 `sudo systemctl stop r1-custom-head-remote`（`stop` 不会被 `Restart=always` 拉起；**别用 `disable`**）。

#### (c) 跳舞 / 动作编排类（7 个目录，共约 600 MB）

`r1_custom_dance4`、`r1_dance_v7_staging`、`r1_dance4_stage1_20260910`、`r1_dance4_walk_music_20260912`、
`r1_dance4_energetic_6597_20260913`、`r1_dance4_stationary_6596_20260913`、`r1_dance4_walk_music_voice_20260913`。

判定依据：每个目录内都是 `bin/ build/ src/ vendor/ CMakeLists.txt README.md logs/ audio/`，
外加大量 `build_*.log` / `build_*.exit` / `current_trial.txt`。`r1_dance_v7_staging` 还带有
`controller.lock`、`controller_v2.lock`、`deployment_status.json`，以及
`EXECUTION_BLOCKED_AFTER_STATIC_FAILURE`、`EXIT_SEQUENCE_20260909_FAILED_BLOCKED` 这类**阻断标记文件**
⇒ 说明其中至少有一版**在静态检查阶段失败后被主动锁死**，属历史遗留状态，别当成「正常运行中的服务」。
另有 `r1_stationary6596_source_20260913.tar.gz` 13 MB 源码包。

> 这些是**按日期快照式的迭代目录**，同一套动作的多个版本并存。当前**没有任何一个在运行**。

#### (d) 灵巧手驱动（4 个 = 4 家不同厂商）

| 目录 | 体积 | 厂商 |
|---|---|---|
| `brainco_hand_service` | 58 MB | BrainCo（强脑） |
| `linker_hand_service` | 14 MB | LinkerHand |
| `h1_inspire_service` | 49 MB | Inspire（因时）+ `inspire_hand.sh` 一键启动脚本 |
| `stark-serialport-example` | 50 MB | Stark（含 linux/windows/python 多语言示例） |

> **四家驱动并存但没有一家在真正工作**：`brainco_hand.service` 处于
> **失败重启循环**（`No ttyUSB serial ports found`，已重启 547 次 / 每 5 秒一次，刷日志）。
> 根因很直接：`/dev/ttyUSB*` **不存在**，只有 `/dev/ttyTHS{0,3,4}`（Jetson 原生串口）⇒ **没有插灵巧手**。
> ⚠️ 四家同时装在同一台机上，将来接真手时要注意 **串口设备名与 DDS topic 冲突**；
> 我们的 `r1_hand_interface.h` 目前是 `NullHandDriver`，尚未与其中任何一家对接。

#### (e) 外设与工具链

| 目录/文件 | 作用 |
|---|---|
| `aic8800_linux_drvier` (79 MB) | **AIC8800 USB WiFi 网卡的驱动源码 + 安装脚本**。驱动与固件**已经装到内核里**（`/lib/modules/5.10.104-tegra/kernel/drivers/net/wireless/aic8800/`、`/lib/firmware/aic8800DC/`、`/etc/udev/rules.d/aic.rules`）⇒ 插上 AIC8800 芯片的网卡**大概率免编译即可用**。安装脚本会 `eject /dev/aicudisk*`，面向「免驱 U 盘形态」的 dongle |
| `AtomDeploy-R1.tar.gz` | **2.3 GB 部署包**（家目录最大单体），历史整机部署快照 |
| `dpkg_list_full.txt` | 503 KB 的已装软件包清单（用于环境复原） |
| `ebee920bb65ac486bb571b7fc429bd9ed (1).zip` | 1.5 MB，用途不明的下载残留 |

#### (f) 用户环境

`.bashrc`（含 **fishros 的 ROS 选择菜单**，交互登录会停在 `read` 等输入 —— 选 `1` = Foxy）、
`.ssh/`、`.config/`、`.local/`、`.ros/`、`.rviz/`、`.bash_history`，
以及桌面环境残留（`Desktop/ Documents/ Downloads/ Music/ Pictures/ Videos/ Public/ Templates/`）。

---

## 2.5 WiFi 能力档案（2026-09-23 三轮实测）

**结论要拆成两句，别混**：

- **① 没有任何内置无线模块** —— `ls /sys/class/net` = `eth10 docker0 dummy0 lo`；
  `ls -d /sys/class/net/*/wireless` 空、`iw dev` 空、`rfkill list` 空。
  ⚠️ `nmcli radio` 打印的 `WIFI-HW enabled` 是 **NetworkManager 的全局开关状态**，
  **不是硬件存在性**（没有 rfkill 设备时它也报 enabled），别被这一行误导。
- **② 但 AIC8800 的 USB WiFi 方案已整套就位** —— 插一张同芯片的 USB 网卡大概率零编译即用。

| 要件 | 实测值 |
|---|---|
| 驱动 | `/lib/modules/5.10.104-tegra/kernel/drivers/net/wireless/aic8800/aic8800_fdrv.ko`（`RivieraWaves 11nac driver for Linux cfg80211`，ver 6.4.3.0）+ `aic_load_fw.ko`（desc 写 `AIC BLUETOOTH`，厂商写死字符串，无碍） |
| vermagic | `5.10.104-tegra SMP preempt mod_unload modversions aarch64` —— **与运行内核逐字一致**，可直接 `modprobe` |
| 依赖 | `fdrv` 的 `depends=cfg80211,aic_load_fw`；`cfg80211.ko` 存在**且当前已加载**。无 `mac80211` 属正常（rwnx 是 fullmac，只用 cfg80211） |
| 固件 | `/lib/firmware/aic8800DC/`（20 文件，**只有 `8800dc` 一套** + `8800dw` 两个 txt 配置） |
| U 盘模式自动切换 | `/etc/udev/rules.d/aic.rules`：`a69c:5721/5722/572a` → `SYMLINK+=aicudisk/aicudiskv2/aicudiskv3` + `RUN+=/usr/bin/eject` |
| 用户态工具 | `iw` / `iwconfig` / `nmcli` / `wpa_supplicant` **全都有** |
| NM 托管策略 | `10-globally-managed-devices.conf` = `unmanaged-devices=*,except:type:wifi,...` ⇒ **wlan0 出现后会被 NM 自动接管**，不会掉进 unmanaged 坑 |
| 内核头文件 | `/lib/modules/5.10.104-tegra/build` **存在**（`dkms` 未装）⇒ 自编 out-of-tree 驱动技术上可行 |
| 源码/脚本 | `~/aic8800_linux_drvier/{drivers,fw,tools,install_setup.sh,uninstall_setup.sh}`（目录名 `drvier` 是**厂商自己的拼写错误**）。`install_setup.sh` 只做：拷 `fw/aic8800DC`→`/lib/firmware/`、拷 `tools/aic.rules`→`/etc/udev/rules.d/`、`udevadm trigger/reload` + `eject /dev/aicudisk*`；**不含编驱动**（.ko 随包预编译） |
| 历史痕迹 | NM 里已有连接 **`Unitree`**（`type=wifi`，`autoconnect=yes`，`/etc/NetworkManager/system-connections/Unitree.nmconnection`，root 600 / 389 B），**mtime `2026-05-22 19:33:21`，比驱动安装（19:29:10）与源码目录（19:28:25）晚约 4 分钟** ⇒ 那次装机是「装驱动 → 装固件/udev → 配 WiFi 连接」成套完成，**当时确实插过卡** |

### 驱动声明的 USB VID:PID 白名单（判断"你的卡能不能用"的唯一依据）

`modinfo -F alias aic8800_fdrv` / `-F alias aic_load_fw` 实测：

- `aic8800_fdrv`：`368b:88e5` `368b:88de` `368b:8d99` `368b:8d91` ·
  `a69c:88dc` `a69c:88dd` `a69c:88de` `a69c:8d41` `a69c:8d81` `a69c:8801`
- `aic_load_fw`：`368b:8d90` `368b:8d91` `368b:8d92` `368b:8d99` ·
  `a69c:8800` `a69c:8801` `a69c:8d40` `a69c:8d41` `a69c:8d80` `a69c:8d81`

**规律：同一张卡换 ID 二次枚举** —— 先以 `8d80/8d40/8800` 经 `load_fw` 下载固件，
固件跑起来后重新枚举为 `88dc/88dd/88de/8d41/8d81/8801`，由 `aic8800_fdrv` 接管成网卡。
**两组都命中才算支持。** `a69c` = AICSemi，`368b` = 另一贴牌商使用的 ID。

（udev 处理的那三个 `a69c:5721/5722/572a` **不在驱动白名单里** —— 它们是「U 盘形态」的 ID，
正是靠 `eject` 把卡从存储模式踢回 WiFi 模式。）

### 两个已知风险

1. **固件只装了 `8800dc` 一套。** fdrv 二进制内部字符串含 `aic8800DC`/`aic8800D80`/`aic8800D80X2`
   （源码是全家桶），但 `rwnx_platform.c:1589` 是
   `sprintf(aic_fw_path, "%s/%s", aic_fw_path, "aic8800DC")` —— **目录名拼死**，而 `/lib/firmware/`
   下只有 `aic8800DC/`。若卡是 **D80 变体**（ID `a69c:8d80/8d81`）可能缺对应固件 ⇒ **插上后必看 dmesg**。
2. 两个 `.ko` 的 `modinfo` **没有 `signature:` 字段**（未签名）。L4T 默认不强制签名，通常能加载；
   报 `Key was rejected by service` 才是这个问题。

### 插卡后的判读步骤

⚠️ **`dmesg` 现在必须 `sudo`**：该机 `kernel.dmesg_restrict=1`，非 root 读 `dmesg` 得到**空输出**，
别误判成"没日志"。

```bash
lsusb                        # ← 第一步：拿 VID:PID 对上面白名单
sudo dmesg -w                # 另开窗口，看识别 + 固件下载过程
ip -br link                  # 应出现 wlan0
iw dev; nmcli dev status
nmcli dev wifi list          # 能看到 SSID 就成了
sudo eject /dev/aicudisk     # 若 dongle 以 U 盘形态出现（udev 一般已自动 eject）
```

判读：`lsusb` 出现 `a69c:` / `368b:` 且 PID 在白名单 → 有戏；
出现 `0bda:`(Realtek) / `0e8d:`(MediaTek) / `148f:`(Ralink) → **不是 AIC8800，这颗驱动帮不上**，
只能换用主线自带驱动的老芯片卡（`MT7601U` / `RTL8188CU-8192CU` / `RT5370` / `AR9271`），或改网络拓扑。

### 连通性不受影响（重要）

给背包加上 WiFi **不影响**它与 `192.168.123.x` 的连通 —— 那是接在机器人交换机上的**有线** `eth10`；
WiFi 只是多了一条**默认路由**，而 `192.168.123.0/24` 是**直连网段路由**，天然优先。因此：

- **主程序第 1 个参数仍传 `eth10`**（DDS 必须绑机器人内网那张卡），**不要传 `wlan0`**；
- PICO 的 UDP 收端绑的是 `INADDR_ANY:9999`（`r1_pico_udp.h:442`），从 WiFi 网卡进来的包照样收得到，
  **一行代码都不用改**；只需把 **PICO 端目标地址改成背包的 WiFi IP**。

---

## 2.6 SDK 副本的角色 —— 以及「把自研代码加进工厂原版」的真实代价

### 三个并存的 SDK 副本（2026-09-23 定案）

| 路径 | 体积 | 身份 | 谁在用 |
|---|---|---|---|
| `~/unitree_sdk2` | 352 MB | **宇树工厂原状**。git 仓库 `main` @ `fa925bf`（2026-03-25），含已编好的 `build/`（39 个官方样例二进制） | 谁也不在用（`/proc/*/exe|cwd` 全扫无命中）。**回滚锚点 + 参考** |
| `~/unitree_sdk2.factory.20260923_110501` | 352 MB | 上者的**字节级只读副本**（`cp -a` + `chmod -R a-w`） | 恢复用备份 |
| `~/r1_teleop_sdk2` | 86 MB | **我们的工作树**：SDK 骨架 = 上游 `origin/main` `63096d0`（2026-09-21）+ 自研代码 | **唯一的编译/运行来源** |

工厂副本的校验口径（`cp -a` 不做这一步等于没有副本）：文件数 **1457**、目录数 **348**、体积 352 MB、
`.git` 在、`build/bin` 39 个、`diff -rq --exclude=build` 无输出、`lib/aarch64/libunitree_sdk2.a` md5 同为
`f0de1d61c7c0f4009a4cce434a8328d2`、写入实测被拒。

**恢复命令**（副本是只读的，`mv` 后要加回写权限；`mv` 本身只需父目录可写）：

```bash
rm -rf ~/unitree_sdk2 && mv ~/unitree_sdk2.factory.20260923_110501 ~/unitree_sdk2 && chmod -R u+w ~/unitree_sdk2
```

### 为什么不能拿工厂树的 `include/` + `lib/` 当依赖（判死）

工厂版（`fa925bf`，2026-03）与我们的基线（`63096d0`，2026-09）之间上游只动了 64 个文件，但
**我们 `#include` 的头文件在工厂版里是「根本不存在」而不是「小改」**：`dds_wrapper/robots/r1/{r1.h, r1_pub.h, defines.h}`、
`robot/r1/audio/{audio_api,audio_client,audio_error}.hpp` 全是新增；`robot_state_client.hpp` 也只有工厂版的一半长。
⇒ **我们的工作树必须自带一份与基线一致的 86 MB 骨架，这份体积消不掉**（唯一可减的是 `lib/x86_64/` 的 27 MB）。

### 「直接加进工厂原版」要动多少 —— 实测 ≈31 个文件

| 组 | 数量 | 内容 |
|---|---|---|
| 骨架 | **11** | `lib/aarch64/libunitree_sdk2.a`（28.8→29.1 MB，**不换就 undefined reference**）、`lib/x86_64/…`（可省）、**新增 6 头**、**修改 3 头**（`common/Subscription.h`、`robot_state_{client,api}.hpp`） |
| 自研代码 | **19** | `example/r1/high_level/` 12 个 + `scripts/` 11 个 |
| CMakeLists | **1** | `example/r1/CMakeLists.txt` —— **只能追加 2 个 target，绝不能整文件覆盖**：我们的版本带着上游新增的 5 个 target（`r1_audio_client_example`、`r1_A5/A7_wrist_swing_example`、`r1_arm_sdk_dds_example`、`r1_arm_action_example`），它们引用的源文件在工厂树里不存在 → cmake 配置阶段直接失败 |

**一条好消息**：`ldd` 实测工厂 `build/bin/*` 对 `libunitree_sdk2` 是 **0 处动态链接**（静态链接），
所以换静态库**不影响**已编好的 39 个官方二进制。

**为什么不推荐**：净效果是工厂原版变成「第三种东西」—— 11 个骨架文件被改、git 工作区永久脏、
未来 `git pull` / 宇树 OTA 必冲突；而它唯一能带来的收益「**路径统一成 `~/unitree_sdk2`**」，
用**一条 `mv`** 就能达成且零文件覆盖。

### 「让 `~/unitree_sdk2` 就是我们的工作树」—— ✅ 2026-09-23 已执行（方案 A）

```bash
rm -rf ~/unitree_sdk2                                    # 内容已在只读副本里，删除是安全的
mv ~/r1_teleop_sdk2 ~/unitree_sdk2
```

**执行后的最终布局**（`deploy_backpack.sh` 的 `VM_DIR` 已同步改回 `~/unitree_sdk2`）：

| 路径 | 权限 | 角色 |
|---|---|---|
| `~/unitree_sdk2` | `drwxrwxr-x` | **我们的工作树**（906 文件 / 86 MB，无 `build/`）。唯一编译/运行来源 |
| `~/unitree_sdk2.factory.20260923_110501` | `dr-xr-xr-x`（只读） | 工厂原状的 **`cp -a` 字节级副本**，1457 文件 / 352 MB，含 `.git` 与 39 个官方二进制 |
| `~/unitree_sdk2-main`(+`.zip`) | — | 2026-02 官方 zip 解压本，与我们无关，别动 |

恢复工厂原状（副本是只读的，`mv` 后要加回写权限）：

```bash
rm -rf ~/unitree_sdk2 && mv ~/unitree_sdk2.factory.20260923_110501 ~/unitree_sdk2 && chmod -R u+w ~/unitree_sdk2
```

磁盘占用从 790 MB 降到 438 MB，**且 `~/unitree_sdk2` 上没有任何"半新半旧"的骨架**。

---

## 3. 当前在跑的服务（与上机相关）

| 服务 | 状态 | 影响 |
|---|---|---|
| `r1-custom-head-remote.service` → `r1_remote_actions eth10` | **enabled + active，`Restart=always`，PID 2093** | ⚠️ 与我们的遥操抢头/腰。运行前 stop |
| `brainco_hand.service` → `brainco_hand_server` | enabled，**失败重启循环（547+ 次）** | 无害刷日志（无串口设备）。接了手才会启动成功 |
| `~/linker_hand_service/build/linker_hand_server` | 无 systemd，需手动起 | Linker O6 手的串口↔DDS 桥 |
| `~/h1_inspire_service/build/inspire_hand` | 无 systemd，`~/inspire_hand.sh` 手动起 | Inspire 手的串口↔DDS 桥（`-s /dev/ttyUSB0`） |
| `~/stark-serialport-example` | 无服务 | **只是串口 SDK 示例**，不是可跑的服务 |
| `/etc/init.d/master_service`（root） | active | 出厂守护，只监管 `ota_pipe` 一个子服务（日志每 5 s 一行） |
| `/etc/init.d/ota_pipe`（root） | active，监听 `tcp://192.168.123.164:1026` | 出厂 OTA 通道，**别动** |
| `nxserver.service` | enabled | NoMachine 远程桌面 |
| docker / containerd | enabled，**无运行中容器** | 无关 |
| `go2_test.service` | **未 enable**（`/usr/bin/python3 /unitree/module/test/test.py`，文件已不存在） | Go2 时期的历史残留，已禁用，可忽略 |
| `tmux` | ✅ **已装**（3.0a，2026-09-23，见 §4.7.10） | 主程序靠 stdin 收 `s`/`d`/`t`/`q`，裸 `ssh host "cmd &"` 会立刻 EOF 使这些键失效 ⇒ **必须跑在 tmux 里** |
| `screen` / `dtach` | 没装（不需要） | tmux 已覆盖用途 |

**`9999/udp` 空闲**，可直接绑。

---

## 4. 对我们的三条操作含义

1. **构建（cmake + g++）对机器人零影响** —— 只读写 `~/unitree_sdk2/build/`，不发 DDS、不碰网络。
   `/proc/*/exe` 与 `/proc/*/cwd` 全扫确认**没有任何进程指向我们的树**，编译不影响在跑的头/腰服务。
2. **运行才会动状态**：首个 `disp=teleop` 包只解锁底盘速度闸门（**2026-09-29 起不再 `StandUp()`**，
   见 §4.10.5）；构造控制器时 `setWeight(1.0)` → `homeHeadAndWaist(3.0)` 把头/腰回零；
   退出/急停 → `Damp()` 由站立变软且**不自动复位**。且与 `r1_remote_actions` 抢头/腰 ⇒ 先 stop 它。
3. **主程序第 1 个位置参数必须传 `eth10`**（本机唯一物理网卡，也是 `192.168.123.x` 那张）。
   `cyclonedds_ws/cyclonedds.xml` 里写的 `eth0` 是错的，但与我们无关（我们不走 ROS）。

---

## 4.5 灵巧手接入：背包上已有四家现成方案

### 4.5.1 一句话结论

背包上四家灵巧手的接入**全部是同一种结构 —— 「串口 ↔ DDS 桥」**，而且**三家用的是同一个 DDS 消息类型**。
所以「加一款新手」不是要在遥操主程序里写串口代码，而是：**跑厂商的桥进程 + 在主程序里加一条映射配置**。

```
   ┌─ 厂商侧（已有 / 自己写）─────────┐   ┌─ 我们的侧（遥操主程序）──────┐
   │  *_hand_server  （独立进程）      │   │  DdsHandDriver               │
   │  串口 ⇄ Modbus/私有协议 ⇄ DDS     │◀──│  只做「PICO 6 路 → 手关节」映射 │
   └──────────────────────────────────┘   │  再 Write 到 rt/<厂商>/.../cmd │
            ▲ /dev/ttyUSB·ttyHand·ttyCH343   └──────────────────────────────┘
            │
        灵巧手（USB-转-串口）
```

### 4.5.1b 手的物理接口在**背包**，不在 PC1（2026-09-23 实测确认）

`/etc/udev/rules.d/99-rosmaster-serial.rules` 全文两行 —— **宇树出厂就为灵巧手预留了接口**：

```
SUBSYSTEM=="tty", KERNELS=="1-1:1.0", SYMLINK+="ttyHand0"
SUBSYSTEM=="tty", KERNELS=="1-1:1.2", SYMLINK+="ttyHand1"
```

文件时间 `2026-03-04 12:23`，与 `brainco_hand_service/`、`linker_hand_service/` 目录同日 ⇒ 是**灵巧手适配包**的一部分，
不是 NVIDIA 镜像自带的（`dpkg -S` 查不到归属）。

- 规则按 **USB 物理端口路径**匹配（不是 VID/PID，也不是 tty 名）⇒ 换任何芯片的双口板，只要插在同一个口，名字照样是 `ttyHand0/1`。
- 现状：`/dev/ttyUSB* /dev/ttyACM* /dev/ttyHand*` **全不存在** ⇒ 手未接入；`brainco_hand.service` 卡在 `activating`（重启 1000+ 次）正是打不开 `/dev/ttyHand0`。
- 结论：**PC1 完全不参与手部链路**（PC1 只管腿/臂，走 `rt/lowstate`、`rt/arm_sdk`，固件闭源）。手的串口是**直接进背包**的，
  所以「串口协议」这份代码在**背包上的厂商桥进程**里（`~/brainco_hand_service/main.cpp`、`~/linker_hand_service/src/*`，**源码在、可重编**），
  我们能改但通常不需要改 —— 我们要写的只是 DDS 侧的 `HandProfile` 映射。

### 4.5.1c 「预留的那两组串口线」到底接到哪个口 —— 2026-09-23 深查结论

**① 那两组线不是 USB 线，是设备侧串口线；背包侧还需要一块「双路 USB-转-串口板」。**

推理链（全从规则本身读出来的，不需要拆机）：

| 观察 | 推论 |
|---|---|
| 两条规则指向**同一个设备** `1-1` 的**不同接口**（`1.0` / `1.2`） | 不是两个独立 USB 设备，是**一块板卡的两个串口通道** |
| 如果两只手各是一根 USB 线（手内部带 USB 芯片），会是两个设备 → 规则应写 `1-1:1.0` + `1-2:1.0` | ❌ 与规则不符 |
| 如果走 USB hub 分两口，会是 `1-1.1:1.0` + `1-1.2:1.0` | ❌ 与规则不符 |
| 接口号是 0 和 2，中间夹着接口 1 | 典型的 **USB 复合设备**：两个数据接口之间夹一个控制/配置接口（单芯片双串口） |

⇒ **两组预留串口线（左手一组、右手一组）要接到一块双路 USB-转-串口板，板卡的 USB 插头插进背包
USB 总线 1 的第 1 个口（`/sys/bus/usb/devices/1-1`）。** 系统随后自动把两个通道命名为
`/dev/ttyHand0`（接口 0）/ `/dev/ttyHand1`（接口 2）。

**② 驱动成分已核查 —— 这是唯一可能踩的硬件坑。**

```
内核里 USB-串口驱动只有：ch341 / cp210x / ftdi_sio / pl2303 / option / usb_wwan / keyspan / xsens_mt …
ch341.ko 支持的 VID:PID = 1A86:7523, 1A86:7522, 1A86:5523   ← 只有 CH340 / CH341 / CH345
lib/modules 下没有 ch343.ko（WCH 厂商驱动），全盘 .ko 里搜不到 "CH343USB" 字符串
```

- 主线的 `ch341` 驱动**从 5.13 才加入 CH343/CH342 支持**，本机内核是 **5.10.104-tegra** ⇒
  **CH342/CH343 双口板插上会完全不出现设备**（一点日志都没有，最容易误判成"线坏了"）。
  而 `linker_hand_service/src/main.cpp:39` 偏偏写死扫 `/dev/ttyCH343USB*` —— 这个名字来自 WCH
  厂商驱动，**在这台背包上永远不会出现**（该代码是从 G1 的仓库照搬的）。
- 若板子是 **CDC-ACM 复合设备**（两个虚拟串口），`cdc_acm` 是内核内置，**免驱**，插上立刻有 `ttyACM*` + 两个 symlink。
- ⇒ **拆板前先确认芯片型号**：CDC 复合 / CP2105 / FT2232H / CH340×2（都不必装驱动），
  还是 CH342/CH343（**需要补 `ch343.ko`** 或换板）。判断法：插上后 `lsusb` 看 VID:PID —— 看到 `1a86:55d3`/`55d4` 就是必须补驱动。

**③ 从未插过。** 从 2026-09-22 起的全部内核日志里，USB 只枚举过两个 root hub，
`/sys/bus/usb/devices/` 下连一个 `1-*` 都没有，`/dev/ttyTHS*` 是 Jetson 板载 UART（与手无关）。

**④ 板子在哪个物理口 —— 5 秒定位法。**

`1-1` = USB2 总线 1 第 1 口。物理位置用插拔法确认：

```bash
# 背包上开着这个，然后拿任意 U 盘/鼠标逐个插背包的 USB 口
udevadm monitor --udev --subsystem-match=usb | grep -E "add|KERNEL"
# 或者看 devpath 目录名
ls /sys/bus/usb/devices/      # 出现的 1-1 就是"手部口"，1-2 是另一个
```

**⑤ 哪只手在哪个通道 —— 软件看不出来，但能读出来。**

不要靠"插左边还是右边"猜：Linker O6 的读寄存器表里 **32–34 = 手编号、35 = 左右方向**，
插上一只手后用探针 `--read 30 8` 直接读出这只手自报是左还是右，再决定它该当 `ttyHand0` 还是 `ttyHand1`。
（BrainCo Revo2 是**两只手挂同一条总线**、靠 Modbus 从站 ID `0x7e`/`0x7f` 区分，见 §4.5.2，
所以那时只有 `ttyHand0` 有数据，`ttyHand1` 是多余的 —— 这正是 `brainco_hand_service/main.cpp:32` 把 `ttyHand1` 注释掉的原因。）

**⑥ 工厂的 "USB3.0 激活" 脚本（`/etc/rc.local`）已经在跑 —— 不是当前阻塞点，但要记着。**

```sh
busybox devmem 0x02430030 w 0x004     # pinmux 把 PP.06 配成 GPIO
echo 446 > /sys/class/gpio/export
echo out > /sys/class/gpio/PP.06/direction
echo 1   > /sys/class/gpio/PP.06/value
```

内容与 `~/unitree/usb3.0激活.docx` 完全一致。**实测已生效**（`0x02430030 = 0x00000004`，`PP.06 = 1`）。
若以后 USB 外设插上没反应，第一件要复查的事就是这个 —— 那段 `rc.local` 每行都用反引号包着，
是 Unitree 自己写的（反引号在这里等价于直接执行，能跑通，但不是好写法）。

### 4.5.2 三家实测参数对照（2026-09-23 勘察）

| 厂商 | 服务目录 / 入口 | DDS topic（订阅 cmd / 发布 state） | 元素数 | 指序（文档定义） | q 语义 | 串口 | 波特 | 协议 |
|---|---|---|---|---|---|---|---|---|
| **BrainCo** Revo2 | `brainco_hand_service/bin/brainco_hand_server`（systemd enabled） | `rt/brainco/{left,right}/cmd` / `.../state` | 6 / 手 | 拇指, 拇指副指, 食指, 中指, 无名指, 小指 | `[0,1] ×1000`（`main.cpp:98`），**方向无证据** | `/dev/ttyHand0`（**左右手同一条口**，`ttyHand1` 在代码里被注释掉了 `main.cpp:32`） | 460800 8N1 | Modbus RTU（slave `0x7e`=左 / `0x7f`=右），经 **`stark-sdk`** 语义 API，**不手搓寄存器** |
| **Linker** O6 | `linker_hand_service/build/linker_hand_server`（手动） | `rt/linker/{left,right}/cmd` / `.../state` | 6 / 手 | 同上 | **0=张开, 1=握紧** | `/dev/ttyCH343USB*`，回退 `ttyACM`/`ttyHand` | **4000000 8N1** | Modbus RTU（`libmodbus`；slave **`0x28`=左 / `0x27`=右**；**FC04 读 / FC16 写**，见 §4.6.1 寄存器表） |
| **Inspire**（H1 版） | `h1_inspire_service/build/inspire_hand -s /dev/ttyUSB0` | `rt/inspire/cmd` / `rt/inspire/state` | **12（左右合一）** | **小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转** | **0=握紧, 1=张开** | `-s` 指定，默认 `/dev/ttyUSB0` | 115200 | **私有协议，不是 Modbus**（`inspire.h`，ID 1=右 / 2=左） |
| Stark | `stark-serialport-example/` | — | — | — | — | — | — | 只是串口 SDK 示例，**不是服务**（`brainco_hand_service` 用的 `stark-sdk` 是它的头文件版） |

三家都用 `unitree_go::msg::dds_::MotorCmds_`（cmd）/ `MotorStates_`（state）—— 消息类型相同，
**差异只在 topic 名、元素数、指序、q 方向**这四件事上。

**证据位置**（背包上可直接复核）：

```
~/brainco_hand_service/main.cpp:24     baud 460800；:98  positions = clamp(q,0,1)*1000
~/brainco_hand_service/main.cpp:127,131 topic rt/brainco/<ns>/{cmd,state}
~/linker_hand_service/src/hand_o6.hpp:44   baud 4000000
~/linker_hand_service/src/hand_dds_service.cpp:38  angles = normalizeToByte(1.0f - q)
     ↑ 注释原文「Upper layer: 0 (open) -> 1 (closed); Device: 255 (open) -> 0 (closed)」
~/h1_inspire_service/src/... inspire_ctrl.cpp:23,26   topic rt/inspire/{cmd,state}，resize(12)
~/h1_inspire_service/example/h1_hand_example.cpp     ctrl(): cmds()[i]=右, cmds()[i+6]=左
     注释「angles in [0,1]. 0: close  1: open」  ← 与 Linker 方向相反
```

### 4.5.3 ⚠️ 三个必须记住的坑

1. **Inspire 的 12 元素是左右合一** —— `[0..5]` 是右手、`[6..11]` 是左手；
   而 BrainCo / Linker 是**左右各一对 topic、每 topic 6 元素**。布局不同，代码不能混用。
2. **Inspire 的指序与 q 方向都与另外两家相反**（它从"小指"数起，且 0=握紧）。
   写错的表现是「手反向乱抓」，而且因为不报错，很容易误判成硬件问题。
3. **BrainCo 的 q 方向在代码里看不出来**（只知 `[0,1]→×1000`、NORMALIZED 模式）。
   **必须实物标定才能定死** —— 不要照抄任何默认值上机。

### 4.5.4 加一款新手的推荐做法（已实现为 `r1_hand_dds.h`）

**不要**让遥操主程序直接碰串口 —— 那会让主程序被某个桥的 `read()` 卡住，且每款手都要改主程序。
正确做法是把「适配」拆成两件互不耦合的事：

| 步骤 | 做什么 | 谁来写 | 我们的代码要不要改 |
|---|---|---|---|
| **① 桥进程** | 串口协议 → `MotorCmds_` 发到 `rt/<厂商>/<side>/cmd` | 厂商（四家都给了）/ 自己照 `linker_hand_service` 抄 | **不改** |
| **② 映射表** | 声明「topic 前缀 / 元素布局 / 指序 / q 方向」 | 我们，一份 `HandProfile` 结构体 | **只加一条配置，不改驱动** |

已落地文件：**`example/r1/high_level/r1_hand_dds.h`**（新增，header-only，默认不接管 ——
主程序仍用 `NullHandDriver`，所以现在编译/运行行为**完全没变**）。

它提供：

- `HandProfile` —— 一款手的完整描述：`ns`(topic 前缀)、`combined`(左右是否合一)、
  `dof`、`joints[6]`(每个关节的来源 `JointMap{src, src_idx, scale, offset}`)、`dq`。
  **方向用 `scale` 表达**：与源同向 `+1`，反向 `-1`（Inspire 就是整体 `-1/+1`）。
- 三个**已按实测参数填好**的预设：`hand_profile_brainco()` / `hand_profile_linker()` / `hand_profile_inspire()`。
- `DdsHandDriver` —— 实现既有 `HandDriver` 接口，`init()` 建 channel、`update()` 发 `MotorCmds_`、
  并可选订阅 `state` 读回手指位置反填 `HandState`。

接入只需三步：

```cpp
// 1) 换驱动（r1_dual_arm_loco.cpp:171 那一行）
r1skeleton::DdsHandDriver hand(r1skeleton::hand_profile_linker());  // 或 brainco / inspire

// 2) 编译同步（本地改完 → 背包）
BACKPACK_PASS=<密码> example/r1/high_level/scripts/deploy_backpack.sh --incremental

// 3) 跑厂商桥，再跑主程序
sudo systemctl start brainco_hand.service        # 或手动起 linker/inspire
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5
```

### 4.5.5 源侧的实话：PICO 只给 5 个可用维度

`r1_pico_udp.h:137` 注释写明 `robot_control.hands` 每侧 6 路是
**`[固定, trigger, trigger, grip, grip, grip]`** —— 这**不是**某款灵巧手的关节序，
只是 PICOHandLink 自己下发的一排 0..10000。主程序把它归一化成 `HandAction.<side>_finger[5]`
（0=张开, 1=握紧）备用。

⇒ 对 6 自由度的手，**第 6 个关节没有独立来源**，只能"跟随"某个手指（`JointMap` 里指向同一个
`src_idx` 即可）。想真正独立控制 6 指，得先在 PICO 端 App 扩 `hands` 字段 —— 这是**设备端**的改动，
不在机器人侧。

此外，PICO 端不区分"哪根手指"，所以「源 → 具体哪根手指」的分配本身就是一个**标定项**：
`r1_hand_dds.h` 里三份预设的前 5 项都按 `finger[0..4]` 顺序排，**上机后需要按实际手感调整**
（改 `src_idx` 即可，不用改驱动）。

---

## 4.6 适配一款**全新**灵巧手：从"什么都不知道"到手能动（runbook）

> 场景：你手里的手不是 BrainCo / Linker / Inspire 三家中的任何一款，背包上没有现成的桥。

### 4.6.1 先纠正一个直觉：Modbus ≠ 寄存器表是公开的

「既然是 Modbus，那我写寄存器就行」——**方向对，但缺了最贵的那块信息**。

Modbus RTU 只是**帧格式**（公开、很小）：`[从站ID][功能码][数据][CRC16]`。它规定了*怎么发*，
**完全没规定*哪个地址是什么*、量纲多少、要不要先使能**。这部分是每家厂商自己定的私有 mapping，
只能来自**数据手册**或**实测**。所以"能不能自由控制"和"知道该写什么值"是两件事：
前者你可能已经具备，后者必须现做功课。

三家现成手的实测对照（都是从背包上的源码读出来的，可直接复核）：

| | 用的库 | 波特/校验 | 从站 ID | 读 | 写 | 量纲 |
|---|---|---|---|---|---|---|
| **BrainCo Revo2** | `stark-sdk.h`（`modbus_open`/`modbus_set_finger_*`） | 460800 8N1 | 左 `0x7e` / 右 `0x7f`（**同一条串口**） | `modbus_get_motor_status` | `modbus_set_finger_positions_and_speeds` | **归一化 0..1000**（`clamp(q,0,1)*1000`，`main.cpp:98`）；也可切 `FINGER_UNIT_MODE` 用「度」；位置 `0xFFFF` = **保持原角度** |
| **Linker O6** | `libmodbus` (`modbus_new_rtu(port, 4000000, 'N', 8, 1)`) | 4000000 8N1 | 左 `0x28` / 右 `0x27` | FC04 输入寄存器 0..44 | FC16 保持寄存器 0..17 | 角度 **0..255**（`0`=弯曲 `255`=伸直），另有转矩/速度寄存器 |
| **Inspire** | 自研 `SerialPort.h` + `inspire.h` | 115200 | 1=右 / 2=左 | — | — | **私有协议，不是 Modbus** |

Linker O6 的完整寄存器表（`hand_o6.hpp:70-83`，源头 `https://document.linkeros.cn/developer/91`）——
这就是"不同寄存器写什么值"的标准长相：

```
读 (FC04)：0-5 当前角度(0-255)  6-11 转矩  12-17 速度  18-23 温度(0-70℃)
           24-29 错误码  30 自由度数 31 手版本 32-34 手编号 35 左右手方向 36-44 硬/软/机版本
写 (FC16)：0-5 目标角度(0-255)  6-11 转矩  12-17 速度
```

⇒ 你要找的其实是**三组数字**：`目标位置寄存器起始地址` + `数量` + `值域`，
再加一个可选但常常必须的 `使能/单位模式寄存器`。

### 4.6.2 探针工具（已交付，`scripts/hand_modbus_probe.py`）

背包上 **Python 3.8、没有 pyserial、不一定能联网**，所以这个脚本**纯标准库**（termios 直开串口），
拷过去 `chmod +x` 即可用，无需装任何包。**默认全部只读**。

```bash
cd ~/unitree_sdk2/example/r1/high_level/scripts

python3 hand_modbus_probe.py --list                  # 有哪些口 / udev 符号链接 / USB VID:PID / 有没有桥在占
python3 hand_modbus_probe.py --scan                  # 盲扫 端口×波特率×从站ID（8N1 与 8E1 都试）
python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 --id 0x7e --read 0 12
python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 --id 0x7e --sweep 0 60
python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 460800 --raw '7e 04 00 00 00 06'
python3 hand_modbus_probe.py --port /dev/ttyHand0 --baud 4000000 --id 0x27 \
        --write 2=0 3=0 4=0 5=0 --yes                # ⚠️ 危险，必须 --yes
```

脚本内置的四个判读要点（都是实战坑，别跳过）：

1. **串口参数不只是波特率**：数据位/校验位/停止位一样致命。三家实测都是 8N1，
   但 **Modbus 标准其实是 8E1** —— 所以 `--scan` 两种都试，别只试一种就下结论"手不响应"。
2. **设备回"异常帧"≠ 坏**：`func|0x80` + 异常码（如 `0x02` 非法数据地址）说明**设备活着**，
   只是这个功能码/地址它不认。脚本会把异常码翻译成人话。而"超时"和"CRC 错"是两回事：
   后者多半是波特率/校验位不对（收到的字节是错位的）。
3. **帧间隔**：Modbus RTU 要求两帧间隔 ≥3.5 字符时间（脚本按波特率自动算，≥1 ms），
   且发送前要清空输入缓冲（否则上一次超时的残字节会被当成这次的应答）。libmodbus 用的是 1 ms 定值。
4. **量纲只能靠"动手"定**：读回来的同一个数，既可能是 `0..255` 的电流，也可能是 `0..1000` 的
   归一化位置。**捏住/拨动某根手指，看哪个地址在变、往哪个方向变** —— 这是唯一可靠的办法。

### 4.6.3 三条路线，按"你手上有什么"选

| 你有的 | 怎么做 | 工作量 |
|---|---|---|
| **① 手带官方 SDK / 官方桥**（像 BrainCo 的 `stark-sdk`、Linker 的 `hand_o6`） | 直接跑官方的桥进程，你只在 DDS 侧加一条 `HandProfile`（见 4.5.4）。**寄存器完全不用碰** | 最小 |
| **② 只有寄存器表**（数据手册给地址+量纲） | 照 `~/linker_hand_service` 抄一个桥进程：`libmodbus` 收发 + `MotorCmds_`/`MotorStates_` 转发。**建议直接复制 linker 那份改**，它的结构就是现成模板 | 中 |
| **③ 什么都没有**（连是不是 Modbus 都不确定） | 先 `--scan` 判断是 Modbus 还是私有协议；是 Modbus 就 `--sweep` 摸地址、动手摸量纲；不是就得上逻辑分析仪抓厂商上位机的报文 | 大 |

**路线 ② 的做法（推荐给"新手"）**：不要在主程序里写串口（会让主循环被 `read()` 卡住，
且每换一款手都要改主程序）。正确做法是**照抄 linker_hand_service 的目录结构**写一个自己的桥：

```
your_hand_service/
├── utils/modbus_rtu_channel.{hpp,cpp}   ← 直接复制 linker 的（libmodbus 封装，已处理帧间隔/超时/低延迟）
├── src/hand_xxx.{hpp,cpp}               ← 只改这里的寄存器常量（照 hand_o6.hpp 那张表）
├── src/hand_dds_service.cpp             ← 复制 linker 的（topic 名改成你的）
└── src/main.cpp
```

桥对外**必须**维持与三家一致的契约，这样 `r1_hand_dds.h::DdsHandDriver` 一行都不用改：

- 订阅 `rt/<你起的名字>/{left,right}/cmd`，类型 `unitree_go::msg::dds_::MotorCmds_`
- 发布 `rt/<你起的名字>/{left,right}/state`，类型 `MotorStates_`
- **`cmds()[i].q()` 语义统一为 0..1**（0=张开 1=握紧），桥内部再换算成你的量纲 —— 换算放桥里，
  别放主程序里
- 左右手布局：**要么**左右各一对 topic 各 6 元素（BrainCo/Linker 风格），
  **要么**一个 topic 12 元素左右合一（Inspire 风格）。二选一，别混

### 4.6.4 上车前的安全清单（盲写寄存器真的会把设备写坏）

1. **先只读**。`--sweep` 是安全的，`--write` 不是。
2. **别碰"会改通信参数"的寄存器**。Stark SDK 里有 `modbus_set_slave_id`、`modbus_set_rs485_baudrate`、
   `modbus_set_canfd_baudrate` —— 写错就是**当场把设备变成"失联"**，只能重新上电或走恢复默认流程
   （`stark-sdk.h:737` 有"恢复默认 ID"的说明，说明这事厂商也知道会有人踩）。
3. **第一次写只写一个寄存器、写最小值**（通常是最松/张开位置），看它往哪动；
   方向反了就把 `HandProfile.scale` 置 `-1`，**不要去改桥**。
4. **手边留断电开关**，眼睛盯着手。首次给 6 个关节同时发目标 = 只发一次、慢慢来。
5. **写完回读**。回读值没变，多半是：使能寄存器没置位 / 单位模式不对（Linker 是 0..255、
   BrainCo 是 0..1000，差了 4 倍）/ 速度上限为 0。
6. 桥进程**独占串口** —— 测之前先 `sudo systemctl stop brainco_hand.service` 或
   `pkill -f linker_hand_server`，否则两边抢同一个 tty，全都超时。

---

## 4.7 背包上「官方接口自检脚本」的当前状态（2026-09-23 只读核查）

### 4.7.1 是哪两个脚本、各自依赖什么二进制

| 脚本 | 定位 | 依赖的二进制 | 不存在时 |
|---|---|---|---|
| `scripts/r1_tool.py` | **官方全功能菜单**（loco/msc/state/audio/config/lowstate） | `build/bin/r1_tool`（自研，`high_level/r1_tool.cpp`） | 自动 `build.sh r1_tool` |
| `scripts/loco_cli.py` / `loco.sh` | 只做运控（`LocoClient`） | `build/bin/r1_loco_client`（官方 example） | 自动 `build.sh r1_loco_client` |

调用约定不同，别记错：
- `r1_tool` 是**位置参数**：`r1_tool <iface> <group> <action> [args...]`（`r1_tool.cpp:385`）。
- `r1_loco_client` 是**命名参数**：`r1_loco_client --network_interface=<iface> ...`（`loco_cli.py:85`）。
- 两个 Python 菜单都会自动探测 `192.168.123.x` 那张网卡 → 本机自动得到 **`eth10`**（不是 `eth0`）。

### 4.7.2 结论：脚本本身不用编译，但它们调的二进制**从来没编过**

`~/unitree_sdk2/build/` **整个目录不存在**（方案 A 落地时 rsync 显式 `--exclude=build`），
所以 `build/bin/r1_tool`、`build/bin/r1_loco_client`、甚至 `r1_dual_arm_loco_skeleton` **都不存在**。
两个菜单都会自动兜底调 `build.sh`，但**首次编译建议手动跑**，好在出错时看清原因：

```bash
cd ~/unitree_sdk2
example/r1/high_level/scripts/build.sh r1_tool              # 官方全功能 CLI
example/r1/high_level/scripts/build.sh r1_loco_client       # 只要运控档
example/r1/high_level/scripts/build.sh r1_dual_arm_loco_skeleton   # 遥操主程序（默认 target）
example/r1/high_level/scripts/run_tests.sh                  # 解析层单测，不接机器人
```

### 4.7.3 首次编译的可行性已逐项验证（都是只读核对）

| 项 | 实测 | 判定 |
|---|---|---|
| cmake | **3.16.3** | OK（`example/r1` 要求 ≥3.10；`target_link_directories` 要求 ≥3.13） |
| g++ | **9.4.0** (Ubuntu 20.04) | OK —— 代码里**没有**超出 C++17 的用法（已 grep `filesystem`/`from_chars`/`span`/`concept`/designated initializer 等，全无） |
| 依赖头 | Eigen / yaml-cpp / fmt 全在 | OK（`build.sh` 会自检这三项） |
| aarch64 SDK 静态库 | `lib/aarch64/libunitree_sdk2.a` 29 MB | OK，**不需要**编 SDK 本体 |
| thirdparty | 只有预编译 `.so`（`ddsc`/`ddscxx` 是 `IMPORTED`，无源码） | OK，configure 很快 |
| `example/` 14 个被 `add_subdirectory` 的子目录 | 全部存在 | OK，configure 不会报"找不到目录" |
| 磁盘 / 核数 | 427 GB 空闲 / 8 核 / 15 GB RAM | OK |

⚠️ **注意背包与测试 VM 的工具链版本不同**：背包是 cmake 3.16.3 + g++ **9.4**，
Parallels 测试 VM 是 cmake 3.22 + g++ 11.4。在 VM 上编过 ≠ 背包上一定编过 —— 以背包为准。

### 4.7.4 动态库怎么找到的：靠编译期烤进去的 RUNPATH

`thirdparty` 的 `ddsc`/`ddscxx` 是 `SHARED IMPORTED` + `IMPORTED_NO_SONAME`，
CMake 会自动把导入库所在目录写进 `DT_RUNPATH`。实测工厂二进制的 RUNPATH：

```
RUNPATH = /home/unitree/unitree_sdk2/thirdparty/lib/aarch64
NEEDED  = libddsc.so.0, libddscxx.so.0
```

⇒ **在 `~/unitree_sdk2` 里编出来的东西天生就能找到库**，前提是那两个 `*.so.0` 软链接在
（实测在：`libddsc.so.0 -> libddsc.so`、`libddscxx.so.0 -> libddscxx.so`）。
另有一层兜底：`/usr/local/lib/libddsc.so.0`（cyclonedds_ws 装的），已在 `ldconfig` 里。
**所以不需要设 `LD_LIBRARY_PATH`。**

### 4.7.5 ⚠️ 唯一会让"脚本看起来失效"的坑：`CYCLONEDDS_URI` 指向不存在的 `eth0`

`.bashrc:124-131` 的 fishros 菜单，选 `1`（ROS Foxy）时会导出：

```
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
```

而该文件里写死了 `<NetworkInterface name="eth0" .../>` —— **本机根本没有 `eth0`，是 `eth10`**。
CycloneDDS 会读这个环境变量，绑一张不存在的网卡 ⇒ **所有官方服务调用静默超时**，
现象和"SDK 挂了 / 网线断了"一模一样。

- **非交互 SSH**（`ssh host "cmd"`）：`read` 直接 EOF ⇒ `case` 不匹配 ⇒ 变量为空，干净。
- **交互式登录 shell / tmux**：只要手滑选了 `1` 就中招。

⇒ 排查时先看 `echo $CYCLONEDDS_URI`；中招了就 `env -u CYCLONEDDS_URI <命令>`，
或干脆别在那个 shell 里加载 ROS。

### 4.7.6 零编译的备用验证手段

工厂只读副本 `~/unitree_sdk2.factory.<TS>/build/bin/` 里留有 39 个**出厂已编好**的二进制，
其中 `r1_loco_client` 与 `r1_ankle_swing_example` 是 R1 相关的（`-r-xr-xr-x`，可执行）。
它们的 RUNPATH 也指向 `~/unitree_sdk2/thirdparty/lib/aarch64`（出厂时树就叫这个名字），
而该路径现在依然存在且库文件一致 ⇒ **不编译也能先验证链路**：

```bash
~/unitree_sdk2.factory.20260923_110501/build/bin/r1_loco_client --network_interface=eth10 status
```

⚠️ 但它是**出厂那份源码**编的（2026-04-03），不含我们的 `r1_tool` 与遥操程序；
且**命令一执行就是真的动运控**，和跑自研二进制同等危险。

### 4.7.7 安全自检顺序（先只读、再动）

```bash
cd ~/unitree_sdk2/example/r1/high_level/scripts
python3 r1_tool.py state services      # ① 只读：列出服务，验证 DDS 通不通
python3 r1_tool.py state version       # ② 只读：版本
python3 r1_tool.py lowstate dump 3     # ③ 只读：订阅 rt/lowstate 打 IMU + 35 电机
python3 r1_tool.py loco get-fsm        # ④ 只读：查 FSM（这一步通了就说明链路完好）
# ---- 以下会真的改变机器人状态，确认周围安全、有保护再执行 ----
python3 r1_tool.py loco damp           # ⑤ 阻尼
```

①～④ 全是只读。**只要 ④ 能返回 FSM 值，就说明"脚本还能用"这条已确认**，不必急着发运动指令。

### 4.7.8 菜单里哪些动作会问你、哪些不会（手滑就出事）

| 菜单路径 | 动作 | 二次确认 |
|---|---|---|
| `1 → 1` | 查 FSM ID / FSM Mode | 无（只读） |
| `1 → 2` | 阻尼 Damp (fsm 1) | **无，直接执行** |
| `1 → 3` | 站立 StandUp (fsm 4) | **无，直接执行** |
| `1 → 4` | 启动主运控 Start (fsm 811) | **无，直接执行** |
| `1 → 6` | 停止移动 StopMove | **无，直接执行** |
| `1 → 7` | 设置速度档位 | **无，直接执行** |
| `1 → 9` | 设置任意 FSM ID | **无，直接执行** |
| `1 → 5` | 移动 move | 要输 `YES` |
| `1 → 8` | 零力矩 ZeroTorque (fsm 0) | 要输 `YES` |
| `2 → 3` | 运动模式 release | 要输 `YES` |

⇒ **8 个动作一按回车就生效**（`r1_tool.py:107-139`）。`-y/--yes` 会连那三个确认也关掉，
**别随手加**。想先看每个菜单项到底发什么，用 `--dry-run`（只打印不执行）。

⚠️ **反直觉的安全差异（会被坑）**：确认逻辑写在**菜单 handler 里**，`Tool.run()` 里没有。
所以 `python3 r1_tool.py` **进菜单**时有 YES 确认，但**脚本化写法没有**：

```bash
python3 r1_tool.py loco zero-torque     # ← 没有确认，直接下发零力矩
```

对比 `loco.sh`（官方 LocoClient 封装，命令式）：`./loco.sh zero-torque` **默认就带 YES 确认**
（`loco.sh:142`），要 `-y` 才跳过；且它还有 `status` / `watch` 能立刻验证 FSM 真的切过去了。
⇒ **想一条条敲命令切 FSM，优先用 `loco.sh` 而不是 `r1_tool.py` 的脚本化形式。**
代价是它需要 `build/bin/r1_loco_client`（`build.sh r1_loco_client`，与 `r1_tool` 是两个 target）。

**运行方式的两条硬要求**：
1. **必须有交互式 TTY**。`input()` 读不到就 `EOFError` → 菜单直接退出。
   所以 `ssh host "python3 r1_tool.py"` 是**看不到菜单的**；要登录后跑，或用 `ssh -t`。
   短命命令无所谓，可直接脚本化：`python3 r1_tool.py loco get-fsm`。
2. **`tmux` 出厂没装** —— 2026-09-23 已装好（`/usr/bin/tmux`，`tmux 3.0a`，见 §4.7.10）。
   靠 stdin 收键的程序（遥操主程序、`r1_arm_manual`）用 `ssh host "cmd &"` 会立刻 EOF 使这些键失效。
   **tmux 与「直接在 SSH 终端里跑」都能用**，差别只在会话存活 —— 见 §4.7.11。

### 4.7.11 tmux 里为什么还会弹 ROS 菜单，以及「到底要不要 tmux」（2026-09-23）

**现象**：`tmux new -s armtest` 之后同样出现 `ros:foxy(1) noetic(2) ?`。

**原因（已核对源码，不是 tmux 的锅）**：tmux 默认起的是**登录 shell** ⇒ 走 `.profile`
⇒ `.profile:14-15` 再 `. ~/.bashrc` ⇒ 撞上 `.bashrc` 第 **121–132 行**那段 fishros 初始化：

```bash
# >>> fishros initialize >>>
echo "ros:foxy(1) noetic(2) ?"
read choose
case $choose in
1) source /opt/ros/foxy/setup.bash;
   source ~/cyclonedds_ws/install/setup.bash;
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;
   export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml;   # ← 这个 xml 绑的是 eth0
   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH;;
2) source /opt/ros/noetic/setup.bash;;
esac
# <<< fishros initialize <<<
```

这段**没有任何守卫**（不判 `$PS1`、不判 `$TMUX`、不判 `$-`），所以只要开交互 shell 就会执行。

**处理**：那行 `read choose` **收空值也不会匹配任何分支** ⇒

| 你的输入 | 结果 |
|---|---|
| 直接按 **回车** | `choose` 为空 → `case` 不匹配 → **什么都不 source**。安全 |
| 按 **Ctrl+C** | `read` 被中断 → 同样什么都不 source。安全 |
| 按 **`1`** | 导出 `CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml`，而该 xml 里 `<NetworkInterface name="eth0"/>`，**本机是 `eth10`** ⇒ DDS 绑不存在的网卡 ⇒ **所有官方服务调用静默超时**（现象和"脚本坏了/网线断了"一样） |
| 按 `2` | source noetic，不导出 `CYCLONEDDS_URI`，本机也没装 noetic ⇒ 基本无害但无意义 |

⛔ **唯一要记住的一条：出现这个提示就按回车，永远不要按 1。** 进程序前顺手 `echo $CYCLONEDDS_URI` 复核。

**两个根治选项**（都改背包上的文件，可选，不做也行）：

1. 让 tmux 起非登录/不读 rc 的 shell —— 新建 `~/.tmux.conf`：
   ```
   set -g default-command "bash --norc"
   ```
   代价：tmux 会话里不再有 `.bashrc` 导出的环境（本项目用不到）。
2. 把 `~/cyclonedds_ws/cyclonedds.xml` 里的 `eth0` 改成 `eth10`：
   ```bash
   sed -i 's/name="eth0"/name="eth10"/' ~/cyclonedds_ws/cyclonedds.xml
   ```
   这样**即使误按 1 也不会有害**（该文件当前绑 eth0 本来就永远不可能生效，改成 eth10 只可能变好）。
   ⚠️ 会动到背包的 ROS2 网络配置，改前先备份。

### 4.7.12 到底要不要 tmux：真正的理由是 **SIGHUP**，不是"方便"

**功能上等价**：只要 SSH 连接在、终端是交互式 TTY，直接在 SSH 终端里跑与在 tmux 里跑**完全一样**。

**差别在连接断掉的那一刻**：SSH 断开 / 终端窗口被关时，内核给**前台进程组发 `SIGHUP`**，
其**默认动作是立即终止进程** ⇒ 进程来不及做任何收尾。

对「只订阅不下发」的程序（`r1_tool.py`、`loco.sh`）这无所谓；
但对**持有 `rt/arm_sdk` 权重的程序**，这正是官方警告的失效模式：
**权重停在 100 且不再发布 ⇒ 手臂冻结在最后一帧姿态，不会自动回落给运控**。

⇒ 两条应对，**都已落地**：
1. `r1_arm_manual` **已接住 `SIGHUP`**（与 `SIGINT`/`SIGTERM` 同一路径）⇒ 掉线时自动走 1 s 线性交还，
   所以**直接在 SSH 终端里跑也安全**（详见 §4.9.8）；
2. 用 tmux 的话，进程根本不随 SSH 断开而死（sshd 只影响 tmux 客户端），会话可 `attach` 回来，
   还能 `Ctrl+B d` 脱离去做别的事。
   ⚠️ 但 **`kill -9` / tmux server 被杀 / 机器重启** 依然救不回来 —— 那时只能靠 `loco.sh damp` 兜底，
   或按官方说明重新接管后释放。

⇒ 结论：**tmux 不是"必须"，但强烈建议**（它把"网络抖一下就冻结手臂"的概率降到 0）。

### 4.7.10 `tmux` 安装实录 + 背包 apt 环境的两处硬伤（2026-09-23）

结论：**已装好**（`/usr/bin/tmux`，`tmux 3.0a-2ubuntu0.4`，`ldd` 无未解析项，
`tmux new-session -d` / `ls` / `kill-session` 实测通过）。但过程踩了两个**以后装任何包都会再遇到**的坑：

**坑 1 —— 背包没有外网，DNS 直接解析失败**

`apt-get download tmux` 报 `Temporary failure resolving 'mirrors.ustc.edu.cn'`，`curl` 同样
`Could not resolve host`。原因：这台机的 `default via 192.168.123.1` 是**机器人内网网关**，
不是能上网的路由；无 WLAN（见 §2.5），所以**只有 192.168.123.x 内网可达**。
⇒ **背包上装任何包都不可能靠 `apt-get install` 直接拉**，必须在本机下好 `.deb` 再 `scp` 过去。
⇒ 排查时别把"解析失败"误判成镜像挂了或 DNS 配错。

**坑 2 —— apt 解析器被一组版本错位卡死，会拒绝安装*任何*包**

`apt-get install -y tmux` 直接失败，但报错跟你想要的包**毫无关系**：

```
libcurl4-openssl-dev : Depends: libcurl4 (= 7.68.0-1ubuntu2.20) but 7.68.0-1ubuntu2.25 is to be installed
libnss-systemd       : Depends: systemd (= 245.4-4ubuntu3.19) but 245.4-4ubuntu3.24 is to be installed
libpython3.8         : Depends: libpython3.8-stdlib (= 3.8.10-0ubuntu0.20.04.8) but ...0.20.04.18 is to be installed
libpython3.8-dev     : Depends: libpython3.8-stdlib (= ...0.20.04.8) 但已装 ...0.20.04.18
python3.8-dev        : Depends: python3.8 (= ...0.20.04.8) 但已装 ...0.20.04.18
E: Unmet dependencies. Try 'apt --fix-broken install' ...
```

精确定性（`dpkg -C` **空**、`apt-mark showhold` **空**）：**不是**包半装坏在 `unpacked` 态，
也没有 hold 钉住，而是**同一套包在库/开发包之间版本错位**——运行时库已被升到新补丁版，
而对应的 `-dev` / `-stdlib` / `libnss-systemd` 还停在旧补丁版，于是它们的 `= <精确版本>` 依赖
永远不成立。apt 的全局检查一旦发现这种状态，**连装一个毫不相关的 tmux 都会拒绝**。

⇒ **零副作用解法（本次采用）**：`apt-get download` 不走依赖解析，但背包没网；
所以**在本机下 `tmux_3.0a-2ubuntu0.4_arm64.deb`**（`mirrors.ustc.edu.cn/ubuntu-ports/pool/main/t/tmux/`），
`scp` 到背包，`sudo dpkg -i` —— `dpkg` 只看这个包自己的依赖，
实测 tmux 要的 4 个依赖 (`libc6 ≥2.27` / `libevent-core-2.1-7` / `libtinfo6` / `libutempter0`) **全部已装且满足**，
所以一次成功，**没碰 systemd / python3.8 / libcurl 中的任何一个**，`dpkg -C` 事后仍为空。
⛔ **不要顺手跑 `apt --fix-broken install`**：那会真的把 `libcurl4-openssl-dev`、`libnss-systemd`
（连带 systemd）、`libpython3.8*` 一起升级 —— 机器上跑着 ROS2/Foxy、cyclonedds、厂商手服务，
为一个 tmux 去动 systemd 和 python3.8 不值当。**除非有明确理由，就让这个错位留着。**
✅ 同路子可复用到任何后续要装的包：**本机 `apt-get download`（或直连镜像）+ `scp` + `dpkg -i`**。

**装了之后怎么用（遥操主程序）**

```bash
ssh unitree@192.168.123.164
tmux new -s teleop            # 起会话
# 里面跑主程序；按键 s / d / t / q 由 tmux 透传
# 断开连接：Ctrl+B 然后 d（会话继续跑）
tmux ls                       # 看会话
tmux attach -t teleop         # 回来
```
⚠️ 交互式登录会撞上 `.bashrc` 的 fishros ROS 菜单（选 `1` 会导出错的 `CYCLONEDDS_URI`，见 §4.7.5）；
**在 tmux 里同样中招**。主程序启动前确认 `echo $CYCLONEDDS_URI` 为空。

### 4.7.9 部署树与本地工作区的版本差

背包上的 `example/r1/` 是 **2026-09-22 的快照**；今天新增的 `r1_hand_dds.h`
（以及 `R1_BACKPACK_ARCHITECTURE.md`）、修改过的 CMakeLists **还没同步过去**。
要带过去用 `deploy_backpack.sh --incremental`（只同步 `example/r1/`，不动别的）。

---

### 4.7.13 背包时钟比开发机慢 ~90 s：`make` 的 `Clock skew` 警告会**静默跳过编译**（2026-09-23）

**现象**：`build.sh` 正常结束、也打印了 `Linking CXX executable`，但**产物其实是旧的**。
伴随一行很容易被 `tail` 截掉的警告：

```
make[3]: warning:  Clock skew detected.  Your build may be incomplete.
```

**根因**：背包系统时钟比 Mac 慢约 **93 s**（`date` 实测），而 `rsync -a` 保留源文件 mtime
⇒ 同步过去的源文件 mtime = "背包的将来" ⇒ `make` 的时间戳比较失去意义。

**两个直接后果**（都会骗人）：
1. **增量构建不可靠**：可能跳过真正需要重编的文件（本次就跳过了一次 `.cpp` 重编，
   只重新链接了旧 `.o`）；
2. **§4.9.5 那条「`bin` mtime 必须晚于 `src`」的判据失效** —— 时钟不同步时，
   即使编译成功，`bin` 的 mtime 看起来也可能**早于** `src`（反向误判）。

**处理**：
```bash
# 背包无外网、无 NTP ⇒ 手动校时（本机先取时间）
sudo date -s "$(date '+%Y-%m-%d %H:%M:%S')"     # 在背包上执行，用本机的当前时间
```
校时后 `Clock skew` 消失、mtime 判据恢复可靠。⚠️ 会漂移，**感觉构建可疑时先 `date` 对一下**。

**如何可靠验证"新代码到底进没进产物"**：

```bash
grep -ac "中文特征串" build/bin/<target>     # ✅ 直接搜原始字节
strings build/bin/<target> | grep -c "中文"  # ❌ 永远 0，会误判成"没编译进去"
```
原因：`strings` 默认按 **7-bit ASCII** 切分可打印串，**UTF-8 中文每个字节都 ≥0x80**，
会被当成不可打印字符截断 ⇒ 中文串根本不会出现在 `strings` 输出里。
（搜 ASCII 特征串如 `SIGHUP` 时 `strings` 才有效。）

---

## 4.8 上遥操程序之前的分步验证：三个官方示例里只有**一个**能用（2026-09-23）

问题：想先单独验证「**腿保持运控（能走/能站）的同时，手臂关节受我控制**」，等这条通了再上 PICO 遥操。
`example/r1/` 下有 5 个与手臂相关的官方示例，但**机制完全不同**，选错会直接把运控关掉：

| 示例（target） | 所在层 | 机制 | 腿的运控 |
|---|---|---|---|
| **`r1_arm_sdk_dds_example`** ✅ | `high_level/` | 发 `rt/arm_sdk`（`LowCmd_`，`mode_pr`=接管权重 0..100） | ✅ **完全保留** —— 机载 `ai_sport` 照常管下肢，手臂是**叠加层** |
| `r1_arm_action_example` | `high_level/` | DDS RPC 调机载 **arm action 服务**（`rt/api/arm/request`） | ✅ 保留，但只能播**预置/示教动作**，**做不了关节级控制** |
| `r1_A5_wrist_swing_example` ❌ | `low_level/` | **先 `MotionSwitcher.ReleaseMode()` 把运控整个释放**，再直发 `rt/lowcmd` | ❌ **运控被关掉**（这正是它 `msc_` 那段 `while(... !name.empty()) ReleaseMode()` 在干的事） |
| `r1_A7_wrist_swing_example` ❌ | `low_level/` | 同上（A7 版） | ❌ 同上 |
| `r1_ankle_swing_example` ❌ | `low_level/` | 直接 `rt/lowcmd` 控踝关节 | ❌ 跑它就没运控了 |

⇒ **要测的就是 `r1_arm_sdk_dds_example`。** 而且它和我们的遥操主程序**走的是同一条通道**
（`rt/arm_sdk` + `mode_pr` 权重 + 同一组 13 槽位），所以它是**理想的跳板**：
它通过 = 遥操程序里「手臂接管」这一段已经通了，剩下要调的只有 IK 与参考系。

**它的行为（`r1_arm_sdk_dds_example.cpp`）**：
1. 订阅 `rt/lowstate` → 等连接；
2. `enable()`：`weight(1.0)`（`mode_pr=100`，**一步到位不渐变**）+ 把每个关节的 `q` **种子设为当前实测位置**（所以不会跳）+ 写 `kp/kd`；
3. `movej(q_target, max_vel=1.0)`：**线性插值**从当前位姿摆到一个固定姿势，100 Hz 下发。
   目标姿势写死为 `{0,1.57,0,1.57,0} × 双臂` + 腰 0 + 头 pitch 0 / **头 yaw 1.0**；
   走完约 `max|Δq| / 1.0` 秒 —— 实测路径就是 **≈1.6 s 内把双肘摆到 1.57 rad**；
4. `release(1.0)`：1 s 内把权重线降到 0（`unlockAndPublish` @100 Hz），然后**程序自己退出**。

**四个必须提醒的点**：
- ⚠️ **它会同时写头部**（HeadPitch/HeadYaw 在 `ArmSdk::JOINTS` 里，13 个槽位含 `WaistYaw`）。
  ⇒ 运行前**必须先停** `r1-custom-head-remote`（`Restart=always`，会抢头/腰关节）；
  否则两边抢同一组关节。
- ⚠️ **它是"一步到位"的权重**（`weight(1.0)`），没有渐变。位置是连续的（种子=当前实测），
  但**力矩接管是瞬间的** —— 手臂此时若被外力/夹具约束，第一帧就可能顶。
- ⚠️ **姿势写死、且幅度不小**（双肘 1.57 rad）。跑之前清空双臂活动范围，人别站在臂展内。
- ⚠️ **需要 `ai_sport` 在跑**：`mode_pr` 是「叠加权重」，权重为 0 时头/腰指令会被 `ai_sport` 忽略
  （`r1_arm_controller.h:118`）⇒ **先把 FSM 切到 `811`（主运控）** 再跑这个示例，否则观察不到效果。
  它也**不发 `crc()`**（`rt/arm_sdk` 不校验；`rt/lowcmd` 才校验）。

**你手动执行的顺序（每步都由你决定何时按）**：

```bash
# ① 编译（背包上没有 build/bin/r1_arm_sdk_dds_example，需先编；这就是首次编译）
cd ~/unitree_sdk2
example/r1/high_level/scripts/build.sh r1_arm_sdk_dds_example

# ② 停掉会抢头/腰的出厂服务（不要 disable，收工要 start 回来）
sudo systemctl stop r1-custom-head-remote

# ③ 确认 DDS 环境干净（交互 shell 里 fishros 菜单选过 1 就会中招）
echo $CYCLONEDDS_URI          # 必须为空；非空就 env -u CYCLONEDDS_URI 再跑

# ④ 把腿交给运控：FSM → 811（先确保机器人已站立/有支撑，周围清空）
example/r1/high_level/scripts/loco.sh status
example/r1/high_level/scripts/loco.sh start      # fsm 811
example/r1/high_level/scripts/loco.sh status     # 复验

# ⑤ 跑手臂示例（tmux 里跑；它约 3 s 后自己退出，中途可 Ctrl+C）
tmux new -s armtest
~/unitree_sdk2/build/bin/r1_arm_sdk_dds_example eth10
#   ↑ 第 1 个参数是网卡名，必须是 eth10（不是 eth0）

# ⑥ 收工：腿回安全态 + 恢复出厂服务
~/unitree_sdk2/example/r1/high_level/scripts/loco.sh damp
sudo systemctl start r1-custom-head-remote
```

**它通过 = 什么**：双臂平滑摆到目标姿势且**腿全程保持站立/可运控**，约 1 s 后手臂平滑交还给
`ai_sport`。这就证明了「手臂叠加层」可用。
**它不覆盖什么**：不含 IK、不含 PICO、不含灵巧手、不做速度/关节限幅（`r1_arm_controller.h` 里才有
250 Hz + 30 rad/s 限幅）。所以它通过之后，再上 `r1_dual_arm_loco_skeleton` 时剩下的风险面就只集中在
**IK / 参考系 / 安全策略**这三块。

> ⚠️ 别用 `low_level/` 那三个示例来做这个测试 —— 它们第一步就是 `ReleaseMode()`，
> 运控一关，腿立刻失去平衡控制。要"腿在跑 + 手臂动"就只走 `high_level/rt/arm_sdk`。

---

## 4.9 双臂关节手动调试工具 `r1_arm_manual`（2026-09-23 新增，已编译）

上遥操程序之前要**分两步**人工确认「双臂接管」这条链路，两步用**同一个程序**：

| 步骤 | 机器人状态 | 这一步在确认什么 |
|---|---|---|
| ① | **锁定站立 FSM 4** | 没有行走干扰时，逐个关节确认**转向、量纲、限位**都对 |
| ② | **走跑运控 FSM 811** | 下肢运控在跑时，双臂**仍能被并行接管**（且行走不受影响） |

- ① 正是宇树官方推荐的手臂 SDK 测试姿态。G1 developer《手臂控制例程》原文：
  「DDS 接口由内置运动服务提供，**在测试时，建议将机器人悬挂并进入锁定站立模式**」。
- ② 与遥操主程序 `r1_dual_arm_loco_skeleton` 的真实工作条件一致，是上遥操前最后一道闸。

源码 `example/r1/high_level/r1_arm_manual.cpp` → CMake 目标 `r1_arm_manual`。

### 4.9.1 与另外两个程序的分工（别混用）

| 程序 | 接管范围 | 目标怎么来 | 什么时候退出 |
|---|---|---|---|
| `r1_arm_sdk_dds_example`（官方） | 双臂 + 头 + 腰（13 槽） | 写死的一个姿势 | 约 3 s 后自己退 |
| **`r1_arm_manual`**（本工具） | **只双臂 10 关节**；头/腰只「就地保持」 | **人在终端逐个关节给角度** | 输 `q` / Ctrl+C |
| `r1_dual_arm_loco_skeleton`（遥操） | 双臂 + 头 + 腰 | PICO 位姿 → IK | 输 `q` / 关 tmux |

### 4.9.2 关键设计（六条"不做就会出事"）

1. **头/腰槽位必须写入，但值取「当前位置」。** `rt/arm_sdk` 的覆盖范围是 13 个关节：
   双臂 10 + `WaistYaw(13)` + `HeadPitch(29)` + `HeadYaw(30)`。权重生效时若这些槽位
   还是默认值（`q=0`），**头/腰会被直接拉到零位**。本工具逐帧把它们的命令值写成当前实测值
   （权重 >0 时冻结在接管瞬间的读数），所以**头/腰全程不动**。
   注：`WaistRoll(12)` 由内置运控独占（官方原文），写它不生效；本工具仍写当前实测值，
   与 `r1_arm_controller.h` 保持同构，行为上是无害空操作。
2. **未接管（权重=0）时目标持续跟随实测。** 于是权重从 0 升到 100 的过程中双臂命令 == 实测，
   **接管瞬间零跳变**；`take` 里还会再显式同步一次。
3. **软限位取自官方 A5 URDF**（`submodules/xr_teleoperate/assets/r1/r1_a5.urdf`）。
   特别注意 **肩 roll 是不对称的** —— 左 `[-13.0°, +142.0°]`、右 `[-142.0°, +13.0°]`，
   按对称写会在实机上撞机械限位。`s`/`+`/`-`/`m`/`a` 全部夹取并打印告警：
   `⚠ left_shoulder_pitch 200.0° 超出限位 [-180.0°, 120.0°] → 夹到 120.0°`。
4. **逐帧速度限幅**（`--vel`，默认 2.0 rad/s）。与 `r1_arm_controller.h::clipTargets` 同思路：
   按「当前实测 → 目标」的最大偏差整体缩放。所以一次设 90° 也是平滑过去的，**不会甩臂**。
5. **`mode_machine` 逐帧从 lowstate 透传**（官方示例只在启动时抄一次）。这样程序**运行期间切
   FSM（4↔811）**不会因 `mode_machine` 过期被低层丢包。
6. **退出必定交还**：`q` / Ctrl+C（SIGINT、SIGTERM）/ **SSH 掉线（SIGHUP）** / stdin EOF
   四条路径都走 `shutdown()` → 权重 1 s 线性降到 0（`[exit] 交还控制（权重 → 0，1 s 线性）...`）。
   ⚠️ 这四条路径的降权**都实测为 1.02 s**（§4.9.8）—— 早期版本在信号路径上会被 `g_quit`
   提前中断而硬切到 0，已修。
   ⚠️ **绝不要 `kill -9`**：官方明确「若程序异常崩溃、权重停留在 1.0 且不再发布指令，
   **上肢会停在最后一帧指令的姿态**」—— 不会自动回落给内置运控。真发生了就重跑本程序
   输 `r`/`w 0` 交还，或用遥控器切阻尼。
7. **电机故障监控**：任一双臂关节 `motor_state[].motorstate != 0` 时，立即把权重在约 0.5 s 内
   降到 0 并**闩锁**，之后 `t` 会拒绝接管（直到故障码清零）。这是官方要求
   （「`motorstate` 非 0 时应立即停止下发并释放控制权」）。`p` 会打印各关节 motorstate。
   若确认该字段在某固件上另有语义，可用 `--no-fault-check` 关掉（默认开）。

### 4.9.3 交互命令

在**交互式终端 / tmux** 里跑（靠 stdin 收命令；`ssh unitree@… "python3 …"` 这种非交互写法收不到）。

| 命令 | 作用 | | 命令 | 作用 |
|---|---|---|---|---|
| `l` | 关节表（当前角 / 目标角 / 限位） | | `m <L\|R> d0 d1 d2 d3 d4` | 整条臂一次设定（度） |
| `t` | **接管**（权重 0→100，1.5 s） | | `a <deg>` | 10 个臂关节全部设为该角度 |
| `r` | **交还**（权重 →0，1.5 s） | | `+ <关节> [deg]` | 该关节加 N 度（默认 5） |
| `c` | 目标 ← 当前实测（偏差归零） | | `- <关节> [deg]` | 该关节减 N 度（默认 5） |
| `h` | 双臂目标回零位 | | `w <0..100>` | 直接设接管权重 |
| `s <关节> <deg>` | 设某关节到指定角度 | | `p` | 运行状态（权重/频率/`mode_machine`/最大残差） |
| `?` | 帮助 | | `q` | 交还并退出 |
| `<空行>` | **重复上一条命令**（配合 `+`/`-` 连续微调很顺手） | | | |

关节写法：`l0..l4`（左）、`r0..r4`（右）、`0..9`（0-4 左，5-9 右）、或 URDF 全名。
顺序：`0=肩pitch 1=肩roll 2=肩yaw 3=肘 4=腕roll`（左右各一组，槽 15-19 / 22-26）。

### 4.9.4 运行前四项检查（顺序别换）

1. 机器人已**吊挂**或有可靠支撑，臂展范围内无人无物；
2. `sudo systemctl stop r1-custom-head-remote`（enabled + `Restart=always`，会抢头/腰槽位；**别 disable**）；
3. `echo $CYCLONEDDS_URI` **必须为空**（非空就 `env -u CYCLONEDDS_URI` 再跑）；
   ⚠️ **tmux 里同样会弹 `ros:foxy(1) noetic(2) ?`**（`.bashrc:121-132` 那段没有 `$TMUX` 守卫）
   —— 见到就**按回车或 Ctrl+C**，**千万别按 1**（按 1 会导出绑 `eth0` 的 `CYCLONEDDS_URI`，
   本机是 `eth10` ⇒ DDS 静默超时）。详见 §4.7.11；
4. FSM 用 `loco.sh` 切，本程序**不切 FSM**；**别和 PICO 遥操程序同时跑**。

### 4.9.5 两步运行流程

```bash
# ---- 编译（已完成，需要重编时）----
cd ~/unitree_sdk2
example/r1/high_level/scripts/build.sh r1_arm_manual
#   → build/bin/r1_arm_manual
#   ⚠️ 改过源码后**必须重编，并核对 `bin` 的 mtime 晚于 `src`**：
#      stat -c "%y %n" build/bin/r1_arm_manual example/r1/high_level/r1_arm_manual.cpp
#      `rsync -a` 保留 mtime ⇒ 两边源码时间一致**不代表**产物是新的。
#      2026-09-23 真实踩到：bin 14:36 < src 14:37，二进制里缺的正是刚修的 SIGHUP/降权缺陷。

# 通用准备
sudo systemctl stop r1-custom-head-remote
echo $CYCLONEDDS_URI                     # 必须为空
tmux new -s armtest
#   ⚠️ 这里会先弹 `ros:foxy(1) noetic(2) ?` —— 按回车（或 Ctrl+C）跳过，**不要按 1**（§4.7.11）
#   再复核一次：echo $CYCLONEDDS_URI   （应为空）

# ---- 第 1 步：锁定站立 (FSM 4) ----
example/r1/high_level/scripts/loco.sh status          # 看基线
example/r1/high_level/scripts/loco.sh stand           # → fsm 4
example/r1/high_level/scripts/loco.sh status          # 复验
~/unitree_sdk2/build/bin/r1_arm_manual eth10 --vel 2
#   进去后: l（看表）→ t（接管）→ s l0 20 → + l0 5 回车回车… → l（看效果）
#   做完全部想验证的关节后输 q

# ---- 第 2 步：走跑运控 (FSM 811) ----
#   ① 先确认机器人已站稳、周围清空（这一步腿是真的能走的）
example/r1/high_level/scripts/loco.sh start           # → fsm 811
example/r1/high_level/scripts/loco.sh status          # 复验
#   ② 再起本工具（同样的命令）
~/unitree_sdk2/build/bin/r1_arm_manual eth10 --vel 2
#   ③ 先 t 接管，再做关节动作；若要让机器人真的走，另开一个 shell 用 loco.sh move
#      （注意：接管后 ai_sport 不再摆臂，重心会变，先原地走小速度试）

# ---- 收工 ----
#   在工具里输 q（或 Ctrl+C）交还权重
example/r1/high_level/scripts/loco.sh damp
sudo systemctl start r1-custom-head-remote
```

### 4.9.6 已验证 / 未验证（诚实边界）

**已在背包实机验证**（2026-09-23）：

- 编译：`build.sh r1_arm_manual` 一次通过，产物 3.0 MB；`ldd` 无未解析项，
  `RUNPATH=/home/unitree/unitree_sdk2/thirdparty/lib/aarch64` ⇒ **不需要 `LD_LIBRARY_PATH`**。
- 功能：`--dry-run`（**不初始化 DDS、不构造任何发布器、不下发任何指令**，对机器人零影响）
  跑通 5 组用例 —— 命令解析 / 超限夹取（`200°→120°`、`-50°→-13°`）/ 增量 / 整臂 /
  空行重复 / **未接管时拒绝下发** / 非法输入健壮性 / **EOF 无 `q` 也安全退出并交还**。
- 故障路径：`--dry-run` 下用 `f <关节>` 注入模拟 `motorstate`，验证「自动降权 → 闩锁 →
  `t` 被拒 → 清零后可重新接管」整条链路。
- 频率实测：设定 250 Hz 实测 **228 Hz**；`--rate 100` 实测 **92 Hz**（循环按设定周期工作）。
- 语法：本机 `./sim/check_syntax.sh r1_arm_manual.cpp` → `0 error`（迭代中被抓到 2 处
  `std::atomic` 按值传给 `printf` 的编译错误，已修）。

**尚未验证（需要真人手动做）**：

1. 真实 DDS 下发（本工具到目前**从未**在真机上发过一帧）；
2. **FSM 4 下低层是否接受 `rt/arm_sdk`**。若第 1 步里 `t` 之后关节毫无反应，说明该固件只在
   811 下认这条通道 —— 退路是**在 811 里、`loco.sh stop`（速度归零）后做第 1 步**，
   效果等价于"站立不动时调臂"；
3. 实机转向 / 量纲（本次默认值全部来自 URDF 与官方示例，未做实物标定）。

### 4.9.7 官方《R1 上肢控制例程（arm_sdk）》要点（2026-09-23 抄录自宇树文档中心）

本工具与遥操主程序的实现都以这份文档为准，几条**容易踩**的原文结论：

| 要点 | 原文/结论 | 对本项目的影响 |
|---|---|---|
| 权重字段 | **承载于 `mode_pr`**，`0~100` 对应 `0.0~1.0`；「与部分其他机型把权重写入某一号关节指令的做法不同」 | `r1_arm_controller.h` 用 `mode_pr` 是对的，别改成写某个关节 |
| 可控关节 | **13 个**：双臂 10 + `WaistYaw(13)` + `HeadPitch(29)` + `HeadYaw(30)` | **`WaistRoll(12)` 写不生效**（内置运控独占） |
| 单臂自由度 | **5**（肩 3 + 肘 1 + 腕 Roll 1），**没有腕 Pitch/Yaw** | A5 关节表与 IK 都按 5 自由度 |
| 推荐增益 | 肩 p/r `50/2`、肩 yaw/肘 `40/2`、腕 roll `30/2`、WaistYaw `50/3`、头 `15/1` | 与本工具/遥操完全一致 |
| 接管姿势 | 置 `weight=1.0` 前，**先把各关节目标初始化为当前实测位置**，再写增益 | 已验证；本工具靠"未接管时逐帧跟随实测"实现 |
| 释放姿势 | **不要 1.0 直接置 0**；在 **~1 s** 内线性降到 0，期间持续发布 | 本工具/遥操都做渐变释放 |
| 发布频率 | **建议 100 Hz**（每 10 ms） | 本项目用 250 Hz（更密、无副作用）；`--rate 100` 可对齐官方 |
| 安全限位 | **接口不做任何安全限位与碰撞检测**，跳变会高速甩臂 | 本工具的 URDF 软限位 + 速度限幅不是"多余功能"，是必需 |
| 电机状态 | 建议控制循环检查 `motor_state[].motorstate`，**非 0 立即停止下发并释放** | 已实现为自动降权 + 闩锁（见 §4.9.2 第 7 条） |
| 崩溃行为 | 程序异常崩溃、权重停在 1.0 且不再发布时，**上肢停在最后一帧姿态** | ⇒ 退出必须走 `q`/Ctrl+C，**绝不 `kill -9`** |
| 调试模式 | 进入调试模式后**内置运控完全退出**，`arm_sdk` 随之失效 | 「锁定站立 FSM 4」不是调试模式，运控仍在跑 ⇒ 可用 |
| 控制权冲突 | 占用 `arm_sdk` 期间别调手臂动作服务；被占用时该服务返回错误码 **`7400`** | 与 `r1-custom-head-remote` 的冲突同理，跑前先 stop |

> 文档同时给了完整可编译示例 `example/r1/high_level/r1_arm_sdk_dds_example.cpp`
> （即 §4.8 里"三个候选里唯一能用"的那个）。它用 `movej` 做关节空间线性插值，
> 与本工具"逐帧速度限幅 + 目标跟随"是同一思路的两种实现。
> 文档地址：`https://support.unitree.com/home/zh/R1_developer/arm_control_routine`

### 4.9.8 修掉两个真缺陷：未接 `SIGHUP` + 降权被 `g_quit` 提前中断（2026-09-23）

都是在"tmux 里为什么又弹 ROS 菜单"这个追问里顺手查出来的，**都属于会冻结手臂的那一类**。

**缺陷 1：只捕了 `SIGINT`/`SIGTERM`，没捕 `SIGHUP`。**
SSH 掉线 / 终端窗口被关时内核发的是 **`SIGHUP`**，默认动作 = **立即终止进程**，
进程来不及走 `shutdown()` ⇒ 权重停在 100 且不再发布 ⇒ 手臂冻结（§4.9.7 最后两行）。
⇒ `std::signal(SIGHUP, onSignal)`，与 Ctrl+C 同一路径。

**缺陷 2（更隐蔽）：`shutdown()` 里的 1 s 线性降权被 `g_quit` 提前中断。**
`rampWeight()` 内原本有一句 `if (g_quit.load()) break;`，而 `shutdown()` 恰恰是在
`g_quit` 已置位时被调用的 ⇒ **循环第一步就 break**，随后 `weight_.store(to)` **一步硬切到 0**，
"1 s 线性释放"完全没发生（手臂会突然失力、也没有过渡帧）。
按 `q` 退出时 `g_quit` 仍为 false，所以走 `q` 一直是对的 —— 这个差异很不容易被注意到。
⇒ 给 `rampWeight` 加 `bool abortable = true`，交还路径传 `false`。

**对照实测**（背包实机、`--dry-run`、同一脚本、命名管道喂 stdin 模拟交互会话，
测「信号/命令 → 进程退出」的墙钟耗时）：

| 退出方式 | 修复前 | 修复后 |
|---|---|---|
| `SIGHUP`（SSH 掉线） | **0.00 s** | **1.02 s** ✅ |
| `SIGTERM` | **0.01 s** | **1.02 s** ✅ |
| `q`（正常退出） | 1.02 s | 1.02 s |

（`--dry-run` 不初始化 DDS、不构造发布器、不下发任何指令；`q` 一栏修复前后一致，
正好反证差异只出在"`g_quit` 已置位"的路径上。修复后 `ldd` 无未解析项。）

---

### 4.9.9 修掉第三个真缺陷：速度限幅写成「基于实测的小步」⇒ 低限速下**纹丝不动**（2026-09-23 首次上机暴露）

**现象**（用户首次上机，FSM 811 下）：
`p` 显示 `权重 100 / 100`、`l` 表里**目标列在变**，但机器人**实际位置一动不动**；
按 `t` 接管时只有"轻微姿态变化"、`q` 后恢复；`loco.sh status` 始终是 811。

**排障过程**（供下次复用）：先把"环境类"逐项排除 —— DDS 通（帧计数在涨）、
`r1-custom-head-remote` = `inactive`、无其它进程占 `rt/arm_sdk`；再比对字段 ——
`mode_pr` 语义（`r1_pub.h:24`：`weight(1.0)` 就是 `mode_pr=100`）、`kp/kd`、
`mode_machine` 透传、`mode=1`（`g1_dual_arm_example.cpp:293` 注释 "1:Enable"）、
CRC 计算位置，**全部一致**。⇒ 问题不在字段，在**我们自己算出来的那个指令值**。

**根因**：`publishOnce()` 原写法

```cpp
allowed = vel_limit * dt;                    // --vel 2 @250Hz ⇒ 0.008 rad
scale   = allowed / maxd;
cmd[i]  = 实测 + (目标 - 实测) * scale;       // 每帧只比【实测】超前 allowed
```

每帧加在电机上的偏差只有 `allowed` ⇒ 偏差力矩 = `kp × allowed`。
`--vel 2` 时 = `50 × 0.008 ≈ 0.4 N·m`，**低于关节静摩擦/伺服死区** ⇒ 实测不动 ⇒
下一帧又从"没动的实测"重新算 ⇒ **指令永远推不动电机**。

**现场分界（实测）**：`--vel 2`（0.46°/帧）**完全不动**；`--vel 30`（6.9°/帧、≈6 N·m）
**立刻能动并跟随命令** —— 这条对照把根因钉死。

**修复**：改成**指令轨迹积分**（与官方 `r1_arm_sdk_dds_example::movej` 的绝对轨迹同源）

```cpp
cmd = cmd_prev + clamp(目标 - cmd_prev, ±allowed);        // 基于【上一帧指令】积分
if (|cmd - 实测| > kCmdLagMax) cmd = 实测 ± kCmdLagMax;    // 兜底，防卡滞解除暴冲
```

超前量会持续累积直到克服死区；`kCmdLagMax = 0.35 rad (20°)` 只在"电机真被卡住"时生效
（正常工况领先量就等于 `allowed`，远小于它）。未接管 / `c` 复位时置 `cmd_reset_` ⇒
轨迹从实测重新起步，**接管仍然零跳变**。

**同一病灶还有一处**：`r1_arm_controller.h::clipTargets()`（移植自 `xr_teleoperate
clip_arm_q_target`），默认 `arm_vel_limit = 30` 所以侥幸能用，但把限速调小（或关节阻力变大）
就会在**遥操主链路**上复现同样症状。**已一并改为指令积分**，并新增参数
`arm_cmd_lag_max = 0.35`。⇒ **两处同源，改的时候别只改一处。**

**新增诊断量**：`p` 里多一行「指令领先实测」，它就是真正加在电机上的偏差：

| 读数 | 含义 |
|---|---|
| 小幅波动（个位数 deg） | 正常，实测在跟随 |
| 一直贴着上限 20° | 电机没跟上 —— 死区过大 / 被卡住 |
| ≈0 且不动 | 指令没推进（目标没变，或限速为 0） |

**两条教训**：
1. **速度限幅必须作用在「指令」上，不能作用在「实测 + 一小步」上** —— 后者依赖电机响应，
   一旦偏差力矩小于静摩擦就永久死锁；
2. 这类缺陷 **`--dry-run` 测不出来**（dry-run 的模拟关节是理想跟随、没有静摩擦）
   ⇒ **dry-run 全绿 ≠ 真机能动**。上机时先看 `p` 的「指令领先实测」这一行。

---

## 4.10 遥操上机流程：准备模式 → 实际遥操（2026-09-23）

### 4.10.1 先纠正一个直觉：**「启动程序」本身就等于「接管」**

`r1_dual_arm_loco.cpp` → `R1ArmController::start()`（`r1_arm_controller.h:126`）：

```cpp
setWeight(params_.motion_mode ? 1.0 : 0.0);   // --motion(默认) ⇒ mode_pr = 100
homeHeadAndWaist(3.0);                        // 头/腰 3 s 内回零
```

所以：

- **「准备模式」不是一个开关或参数**，而是"**程序在跑、但还没收到任何可执行报文**"这段状态；
- 这段状态里**双臂已经脱离 `ai_sport`**，静止在启动瞬间的姿态（目标 = 启动时实测 ⇒ 零跳变），
  腿照常由运控管；
- ⚠️ `homeHeadAndWaist(3.0)` 是**真实运动 3 秒**（头 pitch/yaw + `WaistYaw` 回零），别让人/物在里面；
- 退出（`q`/Ctrl+C/SIGHUP/SIGTERM）走 `goHomeAndRelease()`：双臂回零 → **2 s** 线性降权 → 交还运控
  （`release_dur` 默认 **2.0**，见 `r1_arm_controller.h:205`；**别把 `r1_arm_manual` 的 1 s 记到这里**）。

### 4.10.2 硬前置：PICO 的 UDP 怎么进背包（当前**不通**，必须先解决）

> 📌 **本节已修正 + 展开**，完整方案见独立文档 **`R1_TELEOP_STARTUP.md` 第 5 章**；
> 配套脚本 `scripts/backpack_ap.sh`（体检 / 选卡 / 开热点）。

背包现状（2026-09-23 只读核查，两次复核一致）：

- `ls /sys/class/net` = `lo dummy0 docker0 eth10`；`/sys/class/net/*/wireless` **不存在**
  ⇒ **背包没有任何无线网卡**；
- `lsusb` 只有两个 root hub ⇒ 没插任何 USB 设备；
- `ufw` 未安装 ⇒ 无防火墙问题；
- 内网网关 `192.168.123.1` **ARP FAILED** ⇒ 那台设备**不在线**。

⚠️ **推翻了本文件早先的一条推断**：「`Unitree` 那条 NM 连接就是机器人自己的 AP」是**错的**。
实测该 profile 的 `802-11-wireless.mode = infrastructure` ⇒ 它是**客户端**配置（"去连"一个叫 Unitree 的
外部 AP），**不是能广播的热点**；且文件创建于出厂装机日（2026-05-22），说明出厂时有人用这块背包连过它。
更合理的解释是：`Unitree` 属于**机器人配套的那台路由器/AP**（内网网关 `.1` 本该是它），
而它现在不在线。⇒ **第一件事是确认那台设备是否存在、能否上电接进内网**。

四条路线的对比与操作细节见 `R1_TELEOP_STARTUP.md` §5，要点：

| 路线 | 需要什么 | 今天可用 |
|---|---|---|
| A 机器人配套路由器/AP | 确认实物并开机（`.1` 从 FAILED 变 REACHABLE） | 待确认 |
| B 背包插 USB 网卡自建热点 | 一块**支持 AP** 的卡（内核有无线子系统 ⇒ 即插即用） | 等卡到 |
| C Mac 热点 + UDP 中继 | **零新增硬件** | ✅ 可以 |
| D 头显转有线插内网交换机 | USB-C 转网口 | 取决于头显 |

方案 B 的可行性依据（实测，**不是** `backpack_wifi.sh` 早先那种悲观假设）：
`/lib/modules/5.10.104-tegra/kernel/{net/wireless,net/mac80211,drivers/net/wireless}` **都在**，
且 `/lib/modules/5.10.104-tegra/build` **存在**（有内核头文件）；固件 `htc_9271.fw` / `rt2870.bin` /
`rtlwifi/rtl8192cufw.bin` 也都在 ⇒ **AR9271 / RT5370 / RTL8192CU 三类卡插上即可用**。
⛔ 别买 MT7601U（驱动无 AP 模式）、RTL8188EU/8192EU（`rtl8xxxu` 只有 STA）、
MT7921AU（需 ≥5.18）、RTL8852AU（需 ≥6.4）—— 本机 5.10 内核统统不行。
⛔ **不需要装 hostapd**：NM 1.22.10 自带 AP 模式（`mode ap` + `ipv4.method shared`），
DHCP 由已装的内置 dnsmasq 提供 —— 背包 apt 是坏的（§4.7.10），能少装就少装。

端口一律 `9999`；收端绑 `INADDR_ANY:9999`（`r1_pico_udp.h`）⇒ **从哪张网卡进来都收得到，一行代码都不用改**。

### 4.10.3 准备模式：三步逐步放开（每步都可单独停）

```bash
export PY=/Users/plf/.workbuddy/binaries/python/envs/default/bin/python   # 本机 Mac，已有 mujoco+numpy
```

**第 0 步 · 纯本机回归（不动机器人，1 分钟）**

```bash
./sim/check_syntax.sh
$PY sim/safety_scenarios.py          # 7 个安全场景，末行应 ALL SCENARIOS PASSED
$PY sim/check_pico_e2e.py            # 合成报文 → 全链路 → 独立 FK 比对
```

**第 1 步 · 干跑（程序起来，不发任何报文；机器人只被"接管 + 头腰回零"）**

```bash
# 背包，tmux 内
cd ~/unitree_sdk2 && echo $CYCLONEDDS_URI          # 必须为空
sudo systemctl stop r1-custom-head-remote          # 必须停（抢头/腰槽位）
loco.sh status                                     # 记下 FSM；建议先到 811
build/bin/r1_dual_arm_loco_skeleton eth10 --pico 9999 --variant a5
```

预期日志（**看到最后一行才算准备就绪**）：

```
[arm] lowstate connected.
[arm] publishing to rt/arm_sdk @ 250 Hz
[hand] NullHandDriver initialized (no hardware).
[pico] UDP receiver listening on 0.0.0.0:9999
[loco] LocoClient ready.
[arm] controller ready.                ← 到此头/腰已回零、权重已 100
[main] ready. 等待 PICO HandLink 数据...
[pico] frame mode: head_yaw | target loop 30 Hz ...
```

此阶段判据：**双臂静止保持，不抖不漂**；腿正常。

**先确认 UDP 真的到了**（在另一个终端，独立于程序）：

```bash
ss -lun | grep 9999                       # 应看到 0.0.0.0:9999 已绑定（程序起来了才有）
echo 123 | sudo -S timeout 10 tcpdump -n -i any udp port 9999 -c 5   # 应连续打印若干条包
```

`tcpdump` 有包但程序日志没有 `[pico] first packet` ⇒ 是**解析/字段**问题（例如
`operator_mode` 不是 `active_stream`、`quality != "live"`），不是链路问题；
`tcpdump` 没包 ⇒ 链路问题（目标 IP/端口、网段、PICO 端没在发）。

**第 2 步 · 用模拟发送器排练（真机第一次动；必须在吊挂/有支撑下做）**

```bash
# Mac（与背包同网段：本机 en5 = 192.168.123.200）
$PY example/r1/high_level/scripts/pico_sim_sender.py \
    --host 192.168.123.164 --port 9999 --duration 30 --hz 30
#   wave 模式：双臂会真的跟着正弦摆动。清空臂展！
```

同时**另开一个终端**跑 `loco.sh watch 1`，记录 FSM 变化（原因见 §4.10.5）。
想先确认安全分支，用 `--scenario estop --switch-at 3`（3 s 后软件急停 → 停移动 + 阻尼 + 双臂保持）。

⛔ **发送端一停，>600 ms 即判掉包 ⇒ `Damp()`（FSM 1）⇒ 腿变软**。所以：
排练结束 = 机器人阻尼，**必须有支撑**；恢复要 `loco.sh stand` → `loco.sh start`。

**第 3 步 · 真头显（实际遥操）**——同一条命令，把发送端换成 PICO 头显里的 PICOHandLink App。

### 4.10.4 实际遥操的判据与收工

PICO 端必须同时满足三件事，本端才进入 `disp=teleop` 并真的跟随
（`r1_pico_safety_policy.h::decideDisposition`）：

1. `safety.emergency_stop_latched == false`（急停闩锁优先级最高，压过一切）；
2. `safety.safe_to_execute == true`（本端把它当"唯一许可"，`calibration_ready` 是 PICO 端
   折进这个字段的，**本端不消费标定字段**）；
3. `operator_mode == "active_stream"`（连续发送态）+ 包新鲜（< 600 ms）+ 位姿来源有效
   （`--pico-source controllers` 时看 `ctrl.output_valid`，即左右手 `quality=="live"`）。

任一不满足 → `kHold`（只停移动、双臂保持）；停发 >600 ms 或 `operator_mode=="stop_signal"` → 追加 `Damp()`。

**退出：四条路径等价，但过程是「先阻尼、再回零」**

**退出四条路径**（`q`/Ctrl+C/SIGTERM/SIGHUP）**都进同一条清理路径**，`q` 与 Ctrl+C 完全等价。
但清理过程**不是"立刻停住"**，真实动作顺序是：

```
main 循环退出 → ① loco 线程收尾：StopMove() + Damp()   ← 腿在这一点变软（FSM 1）
              → ② arm.goHomeAndRelease()：
                   目标置 0 → 等 |q| < 0.05 rad（最多 100×50 ms = 5 s，双臂真实摆回零位）
                   → 再 2 s 线性把权重降到 0（release_dur 默认 2.0，交还运控）
              → ③ hand.stop() → arm.stop()
```

⇒ 整个退尾短则约 2 s、长则约 7 s（回零等满 + 2 s 降权）。

⇒ ⚠️ **两条必须知道的事**：
1. **退出 = 底盘阻尼（FSM 1），腿会变软**（这是 loco 线程收尾的既定行为，不是故障）。
   机器人若没有吊挂/支撑，会在"双臂还没回完零"的过程中就开始塌。
2. **退出期间双臂会主动摆回零位**（最多 5 s），不是原地保持 ⇒ 退出前清空臂展。

收工顺序：

```bash
# 1) 在主程序里输 q + 回车（或 Ctrl+C）——等它打印 [main] done. 才算走完
#    注意：Q/Ctrl+C 期间机器人会 ① 阻尼 ② 双臂回零，别在此时扶人
example/r1/high_level/scripts/loco.sh status      # 复验：应为 FSM 1（阻尼）
sudo systemctl start r1-custom-head-remote        # 恢复出厂服务
```

（`loco.sh damp` 通常**不用再补**——程序退出时已 Damp 过；`status` 看到已不在 811 即可。）

### 4.10.5 首包切 FSM 的悬案 —— **2026-09-29 已消解（代码已改）**

原第 1 条「首包 `StandUp()` 会把 FSM 从 811 拉到 4」已被**移除**：

- 官方 `xr_teleoperate` 的 `--motion` 明写要求「可在机器人运控程序运行下进行遥操作」，
  且「只支持 `Regular mode`(R1+X)」；**它自己从不切 FSM**。我们的自动 `StandUp()` 是偏离官方的一步，
  且正好切进「FSM 4 认不认 `rt/arm_sdk`」这个未验证状态，起身还会与开始跟随同帧 ⇒ 无法二分故障。
- **新行为**：首个可执行包只把 `first_exec_pkt` 置位（解锁底盘速度闸门），**不调 `StandUp()`**。
  旧行为降级为 `--auto-stand`（默认关）。
  ⇒ 前置要求变成硬性的：**起程序前机器人必须已在 811**（`loco.sh status` 确认）。
- **原第 2 条仍然成立**：程序从不调 `Start()`（只在 `s` / `--move` / `--auto-stand` 时调 `StandUp()`）。
  若腿不听摇杆，手动 `loco.sh start` 补上。

> 语义澄清：`loco_ready` 现在表示「允许下发速度」，**不等于**我们改过 FSM ——
> 旧代码里它叫 `standing`，容易让人误以为"程序让机器人站起来了"。

---

## 5. 快速复现本次普查的命令

```bash
# 拓扑：同网段都有谁（ICMP 被禁，靠 ARP）
ssh unitree@192.168.123.164 'ip neigh; ip -4 -o addr show; ip route'

# 出厂层
ssh unitree@192.168.123.164 'find /unitree -maxdepth 3; cat /version.txt; \
  cat /unitree/robot/pkg/version/version.json; cat /unitree/ota/pipe/ota_pipe_service.json'

# 服务与端口
ssh unitree@192.168.123.164 'systemctl list-unit-files --type=service --state=enabled; ss -lntup'

# 业务层分类
ssh unitree@192.168.123.164 'du -sh ~/* | sort -h | tail -20; ls ~/*/ | head -0'
```

```bash
# 灵巧手：四家的 topic / 指序 / q 方向（§4.5 的数据来源）
ssh unitree@192.168.123.164 '
  for d in brainco_hand_service linker_hand_service h1_inspire_service stark-serialport-example; do
    echo "==== $d"; ls ~/$d;
  done
  systemctl cat brainco_hand.service
  grep -nE "rt/|baud|tty"          ~/brainco_hand_service/main.cpp
  grep -nE "rt/|baud|slave|/dev/"  ~/linker_hand_service/src/*.cpp ~/linker_hand_service/src/*.hpp
  cat                              ~/h1_inspire_service/include/param.h
  cat                              ~/h1_inspire_service/inspire_ctrl.cpp
  ls -la /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS*
'

# 目录角色（方案 A 执行后的最终布局）
ssh unitree@192.168.123.164 'ls -ld ~/unitree_sdk2 ~/unitree_sdk2.factory.* ~/unitree_sdk2-main'
```
