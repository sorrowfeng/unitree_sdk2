# 环境备忘 —— 三台机器与主机级坑

> 这些是**主机层面**的事实，跟业务代码无关，但踩上就很难查。
> 业务相关的（协议、算法、已修复清单）见 `AGENTS.md`。
> ⚠️ 不写凭据：密码只经环境变量传递（`VM_PASS` / `BACKPACK_PASS`）。

## 三台机器

| 角色 | 地址 | 说明 |
|---|---|---|
| **PC1** 运控计算单元 | `192.168.123.161` | **禁 SSH**。全端口不通 **≠ 离线**（禁 ICMP，别用 ping/nc 判）。**绝不部署**。 |
| **PC2 = 算力背包** | `192.168.123.164` | 用户 `unitree`。Ubuntu 20.04.5 + JetPack 5.1 + aarch64，内核 `5.10.104-tegra`。唯一物理网卡 **`eth10`**。**唯一部署目标**。日志里的 `pc4` 就是它（宇树 OTA 的内部编号）。 |
| **测试 VM**（Parallels） | `10.211.55.6` | 用户 `plf-virtual`，Ubuntu 22.04，cmake 3.22 / g++ 11.4。⚠️ **与背包工具链不是一套**（背包 cmake 3.16 / g++ 9.4）⇒ **VM 上编过不代表背包上编得过**。 |

- **认证间歇失败**：同一脚本同一密码时而全败 —— 服务端抽风，**重试 2~5 次**，别一失败就怀疑密码。
- 测试 VM **只走密码认证，不推公钥、不配免密**（已定）。

---

## 背包（最容易踩的）

### 没网、apt 是坏的
- ⛔ **没有外网 DNS**（`default via 192.168.123.1` 是机器人内网网关，无 WLAN）
  ⇒ 装包只能「本机下 `.deb` → `scp` → `sudo dpkg -i`」。
- ⛔ **apt 已废**：`libcurl4-openssl-dev` / `libnss-systemd` / `libpython3.8*` 版本错位
  ⇒ 它**拒绝安装任何包**，且报错跟你要装的包毫无关系。
  **别跑 `apt --fix-broken install`**（会真去升 systemd / python3.8，可能把系统搞坏）。
- 纯 Python 的依赖**不需要 pip**：直接把包目录拷进去即可
  （例：pyusb 1.2.1 的 `usb/` 目录；⚠️ pyusb 1.3.x 要求 Python ≥ 3.9，背包是 3.8，用不了）。

### 目录角色（别乱动）
- `~/unitree_sdk2` = **我们的工作树**，唯一编译/运行来源。
- `~/unitree_sdk2.factory.20260923_110501` = 工厂原状**只读副本**
  （要恢复：`rm -rf` 现工作树 → 把副本 `mv` 回原名 → `chmod -R u+w`）。
- `unitree_sdk2-main`、`r1_custom_head`、`r1_dance4_*`、`*hand_service` 等 → **别动**。
- ⛔ **禁「复用工厂树的 SDK 当依赖」**：工厂版连我们 `#include` 的 r1 头文件都没有 ⇒ 必然编译失败。

### 时钟漂移
⏰ 背包时钟会**大幅落后**（记录里 09-23 慢 ~90 s，10-08 已慢到 ~200 s）⇒ `make` 报
`Clock skew detected` 后**静默跳过重编**，产物还是旧的，却显示"编译成功"。
**改代码后先校时**（`$(date)` 必须在 **Mac** 上展开）：

```bash
ssh unitree@192.168.123.164 "sudo date -s '$(date '+%Y-%m-%d %H:%M:%S')'"
```

验证"新代码真进了产物"：`grep -ac "中文特征串" build/bin/<target>`。
❌ **不要**用 `strings build/bin/x | grep 中文` —— UTF-8 中文会被截断，**永远返回 0**，会误判。

### ⚠️ CRLF 污染（极具迷惑性）
工作区里上游文件是 **CRLF**、git 里是 **LF** ⇒ 用 `diff -rq` 比两棵树会报
**~858 行差异，几乎全是假的**；而 `git status` 却是干净的。
**做任何"两棵树差在哪"的判断前，先排掉 CRLF。**

### DDS 被 fishros 菜单带偏
`~/cyclonedds_ws/cyclonedds.xml` 写死 `eth0`，而本机是 **`eth10`**。
交互 shell 会弹 `ros:foxy(1) noetic(2) ?`，选 `1` 就导出 `CYCLONEDDS_URI`
⇒ **DDS 静默超时**，现象跟"脚本坏了/网线断了"一模一样。
- ⛔ **tmux 里同样会弹**（tmux 起登录 shell → `.profile` → `.bashrc`，那段没有 `$TMUX` 守卫）
  ⇒ **见到就按回车，永不按 1**。
- 排查先 `echo $CYCLONEDDS_URI`（应为空）；中招用 `env -u CYCLONEDDS_URI`。
- 非交互 ssh 不受影响。

### 其它
- `dmesg` **必须 sudo**（`kernel.dmesg_restrict=1`），非 root 读到**空输出** —— 别误判成"没日志"。
- `tmux` 已装（3.0a）。主程序靠 stdin 收 `s`/`d`/`t`/`q` ⇒ 必须在**交互式 TTY**
  （tmux 或 `ssh -t`）里跑；`nohup`/`setsid` 的 stdin 不可用。

---

## macOS 侧

- **不能编译**：SDK 只带 Linux 静态库（`lib/{x86_64,aarch64}/`），且代码用 Linux 专有头文件
  ⇒ 只能 `-fsyntax-only` 做语法校验（`sim/check_syntax.sh`，靠 `sim/shim` 垫掉专有类型）。
- **BSD grep 不支持 `\|` 与 `\b`**（当字面量）⇒ 用 `grep -E` 或 ripgrep。已因此误报多次。
- 没有 `timeout` 命令（GNU coreutils 才带）。
- USB-CANFD 适配器在 macOS 上也能用，但 pyusb 需要
  `DYLD_LIBRARY_PATH=/opt/homebrew/lib`（否则 `no USB backend available`）；
  `scripts/r1_hand/hand_canfd.sh` 会自动处理。

---

## Git

- `origin` = 官方上游（**只读**，push 必 403）；`fork` = `github.com/sorrowfeng/unitree_sdk2`。
  **唯一冲突热点是 `example/r1/CMakeLists.txt`**（两边都保留）。
- 仓库 `core.fileMode=false` ⇒ 新脚本要 `git update-index --chmod=+x <file>`。
- 代理 `http://127.0.0.1:7897` 对 `github.com` 可用，对 `api.github.com` 不通
  （`gh` 要 `env -u HTTPS_PROXY …`）。
- **泄密扫描**别只看 grep 有没有输出：二进制 blob 与 `<密码>` 占位符会稳定假阳性，
  先判「是否本次引入」。
