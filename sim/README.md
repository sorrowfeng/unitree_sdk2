# sim/ — 本机双臂遥操仿真验证

在 **macOS 本机**（不需要机器人、不需要 DDS）验证 `example/r1/high_level/r1_arm_ik.h`
的 IK 精度，并做可视化回放。

## 为什么能本机跑

| 模块 | 依赖 | 本机可编译 |
|---|---|---|
| `r1_arm_ik.h`（FK / IK） | 仅 Eigen | ✅ |
| `r1_xr_pose_alignment.h`（对齐） | 仅 Eigen | ✅ |
| `r1_pico_udp.h`（UDP 解析） | + SDK 的 `FromJsonString()`（实现在 Linux 私有库里） | ❌ 缺符号 |
| `r1_arm_controller.h`（臂控） | unitree DDS 封装 | ❌ 需 SDK |

因此本层验证聚焦 **运动学 + IK**（遥操准确性的核心），UDP/DDS 部分留到 VM 或上机。

## 方法论：避免自证循环

用 **MuJoCo 加载同一份官方 URDF**（`submodules/xr_teleoperate/assets/r1/r1_a*.urdf`）
做**独立正运动学**作为真值，而不是拿 `r1_arm_ik.h` 自己的 FK 去验证它自己的 IK。

```
目标 EE 位姿 ──┬─→ C++ IK (r1_arm_ik.h)      ─→ q ─┐
              │                                    ├─→ MuJoCo FK（同一 URDF）→ 实际 EE 位姿 → 残差
              └─→ (可选) 官方 CasADi/IPOPT IK ─→ q'─┘
```

## 安装依赖

```bash
# 1) C++ 探针（自动下载 Eigen 3.4 到 sim/thirdparty/，已 gitignore）
./sim/build_probe.sh

# 2) Python 侧（建议用隔离 venv）
python3 -m venv .venv && .venv/bin/pip install mujoco numpy matplotlib imageio imageio-ffmpeg
```

## 用法

```bash
# 精度评估：FK 交叉校验 + IK 残差统计 + 轨迹跟踪 → 图表 + JSON
.venv/bin/python sim/eval_ik.py                      # a5 + a7 全跑
.venv/bin/python sim/eval_ik.py --variant a5 --n-ik 500
#   产物：sim/out/{v}_ik_error_box.png、{v}_traj_error.png、{v}_traj_xy.png、eval_summary.json

# 可视化回放：MuJoCo 窗口，每只手两个标记球
#   绿球（下方）= 目标末端（人手给的位置）
#   红球（上方）= 实际末端（IK 解算后到达的位置）
#   两球水平对齐 = 跟踪准确；红球相对绿球偏移 = 误差
.venv/bin/python sim/view_traj.py --variant a5
.venv/bin/python sim/view_traj.py --variant both --mode ik      # 含 WMA 平滑（实机口径）
.venv/bin/python sim/view_traj.py --variant both --headless --record sim/out/teleop.mp4

# 根因诊断：复刻 solveArm 的更新公式，对比"原样/加权/限位"三组
.venv/bin/python sim/diag_ik_rootcause.py
```

## 文件

| 文件 | 作用 |
|---|---|
| `ik_probe.cpp` | C++ 探针：把 `r1_arm_ik.h` 的 FK/IK 暴露成行式文本接口（stdin/stdout） |
| `build_probe.sh` | 编译探针（自动拉 Eigen） |
| `r1_model.py` | MuJoCo 模型封装：URDF 加载、关节映射、独立 FK、限位采样 |
| `eval_ik.py` | L1 精度评估：FK 交叉校验、IK 残差统计、轨迹跟踪、出图 |
| `view_traj.py` | L2 可视化：MuJoCo 回放 + 目标/实际末端标记 + 可选导出 mp4 |
| `diag_ik_rootcause.py` | 发散问题的根因对照实验 |

## 探针接口（供二次开发）

```
ik_probe --variant a5|a7 --mode fk|ik|ik-raw

fk     : q[2n]                                  → L(p3 q4) R(p3 q4)
ik     : L(p3 q4) R(p3 q4) qcur[2n]             → q[2n]（solve，含 WMA 平滑）
ik-raw : 同上                                    → q[2n]（solveArm，无平滑，逐臂）

坐标：骨盆系 = URDF waist_yaw_link 系（waist_yaw 锁 0）；EE 含手安装偏移
      （A5 +0.20m x / A7 +0.05m x）；四元数顺序 x y z w。
```

## 当前基线（2026-09-21，`r1_arm_ik.h` 修复后）

| 指标 | A5 | A7 |
|---|---|---|
| FK 交叉校验（C++ vs MuJoCo） | 位置差 p95 6.3e-10 mm | 6.4e-10 mm |
| IK 冷启动 home→典型位姿（±0.5 rad） | mean 0.13 mm | 0.39 mm |
| IK 任务空间（±0.6 rad，n=300） | mean 0.09 / p95 0.40 mm，100% ≤5mm | 0.33 / 1.07 mm，98.5% |
| 轨迹跟踪 800 帧（warm start，无平滑） | mean 0.03 mm | 0.03 mm |
| 轨迹跟踪（含 WMA 平滑） | mean 4.33 mm | 3.45 mm |
| IK 全域采样（全关节范围，n=300） | mean 29.6 mm，71.8% ≤5mm | 34.8 mm，66.2% |

> 全域采样下的大误差来自两方面：① 5 DOF 手臂跟踪 6D 全姿态的固有折衷；
> ② 目标本身可能不满足关节限位。真实遥操工作空间在任务空间采样一档。
