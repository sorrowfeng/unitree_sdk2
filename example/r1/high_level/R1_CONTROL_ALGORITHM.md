# R1 双臂控制算法提取与双线程骨架说明

> 来源：`unitreerobotics/xr_teleoperate`（本机 clone 于 `D:\Project\RobotProject\xr_teleoperate`）
> 对应上游文件：`teleop/robot_control/robot_arm.py`、`teleop/robot_control/robot_arm_ik.py`、`assets/r1/r1_a5.urdf`、`assets/r1/r1_a7.urdf`
>
> 本目录骨架文件：
> - `r1_arm_ik.h` —— R1 双臂运动学 + 数值 IK（DLS，移植官方目标函数）
> - `r1_arm_controller.h` —— 双臂控制器（250 Hz 发布 rt/arm_sdk，速度限幅，接手/释放）
> - `r1_hand_interface.h` —— 灵巧手驱动接口（预留）
> - `r1_dual_arm_loco.cpp` —— LocoClient + ArmSdk 双线程主程序
>
> 骨架按官方 **rt/arm_sdk 覆盖模式** 设计：行走归 ai_sport 管，双臂由你的进程接管。
> 骨架默认 **R1_A5**；R1_A7 在 xr_teleoperate 中暂不支持 motion_mode（rt/arm_sdk），
> 接入 A7 前请确认固件是否开放该话题（代码已保留 A7 路径）。

## 1. 控制链路（官方 teleop 的做法）

```
XR 末端位姿（骨盆系，head-yaw 参考系）
        │
        ▼
R1_A5/A7_ArmIK.solve_ik(left_wrist, right_wrist, current_q, current_dq)
        │  ① 目标函数：位置(50) >> 姿态(A5:0.5 / A7:1.0) + 正则(0.02) + 平滑(0.1)
        │  ② 关节限位：URDF lower/upper 硬约束
        │  ③ warm start = 当前关节角；输出过 WMA[0.4,0.3,0.2,0.1]
        ▼
R1_A5_ArmController.ctrl_dual_arm(sol_q, 0)
        │  ④ 独立线程 250 Hz：速度限幅 30 rad/s → 写 motor_cmd 槽位 → CRC → Write
        ▼
rt/arm_sdk（LowCmd_，mode_pr = 权重 0..100，100=完全接管双臂）
        │
        ▼
机载 ai_sport：下肢平衡/行走照常；双臂按你给的 PD 目标执行
```

## 2. 关键参数（与 xr_teleoperate 当前 head 一致）

| 项 | 值 | 出处 |
|---|---|---|
| 发布频率 | 250 Hz（control_dt = 1/250） | R1_A5_ArmController |
| arm_sdk 权重 | mode_pr = int(weight*100)，release 时 2 s 线性降到 0 | _set_arm_sdk_weight |
| 关节速度限幅 | 30 rad/s（arm_vel_limit，可改） | set_arm_velocity_limit |
| 上电头/腰回零 | 3 s 线性到 0（头 29/30，腰 12/13） | ctrl_head_and_waist_go_home |
| 双臂回零 | target=0 直到 |q|<0.05（最多 5 s） | ctrl_dual_arm_go_home |
| 肩 pitch/roll | kp=50, kd=2 | kp_low/kd_low |
| 肩 yaw、肘 | kp=40, kd=2 | kp_medium/kd_medium |
| 腕（A5 为腕滚） | kp=30, kd=2 | kp_wrist/kd_wrist |
| 头 | kp=15, kd=1 | kp_head/kd_head |
| 腰 | kp=50, kd=3 | waist_kp/kd |
| 腿（仅 rt/lowcmd 模式） | kp=200, kd=3；踝 pitch 弱化 50/2 | kp_high/kd_high、_Is_weak_motor |

关节槽位（LowCmd_ 35 槽）：
- **A5**：左臂 15-19（肩P/肩R/肩Y/肘/腕滚），右臂 22-26，头 29/30，腰 12/13，腿 0-11。
- **A7**：左臂 15-21（多腕P/腕Y），右臂 22-28，其余同上。
- rt/arm_sdk 模式下：只更新 双臂+头+腰 槽位，下肢/躯干交给 ai_sport；其余槽位置 0（不使能）。
- rt/lowcmd 模式下：35 槽全量使能（腿 PD 锁定）。

## 3. IK 公式（官方，CasADi+IPOPT NLP）

```
min  50·||p_ee(q) - p_target||^2 + c_rot·||log3(R_ee(q)·R_target^T)||^2
     + 0.02·||q||^2 + 0.1·||q - q_last||^2
s.t. q_lower ≤ q ≤ q_upper          （URDF 限位）
where c_rot = 0.5 (A5) / 1.0 (A7)
      EE 定义：A5 = 腕滚关节 + 0.20 m (x)；A7 = 腕yaw关节 + 0.05 m (x)
求解器：IPOPT（max_iter=30，warm start = 当前 q），输出 WMA[0.4,0.3,0.2,0.1] 平滑
失败：回退返回当前关节角、零力矩
```

`r1_arm_ik.h` 用**阻尼最小二乘（DLS）**逼近同一目标：
错误向量 = [√50·Δp；√c_rot·rotvec(R_ee·R_t^T)]，Δq = (J^T·J + λ·I)^-1 · J^T·e，
数值雅可比（6×n）来自 URDF 原点/轴线参数，关节限位每步硬 clamp。
位置权重远大于姿态 → 行为与官方一致：优先末端够得到，姿态为软目标。

> 追求最高保真度：直接跑 xr_teleoperate 的 Python IK（Pinocchio+CasADi），
> 或把 C++ 侧换成 Pinocchio/CasADi 同名实现。

## 4. 灵巧手预留

`r1_hand_interface.h` 提供：
- `HandAction` / `HandState`（每帧动作与反馈的抽象结构）
- `HandDriver` 接口（init/update/stop）——接你自己的手时实现它
- `NullHandDriver` 默认空实现（只打日志）
- 注释里给出官方手（Dex3/Inspire/BrainCo）的 DDS 话题与 Dex3 驱动骨架

主循环每 100 Hz 调用 hand.update(action, state, dt)，与双臂目标生成同频同步，
方便以后把手部动作直接作为胳膊 IK 目标姿态 + 手指目标一并发给机器人。

## 5. 依赖与构建

- C++17；Eigen（libeigen3-dev）；unitree_sdk2（本仓库根目录构建）
- 在仓库根目录：mkdir build && cd build && cmake .. && make（生成 r1_dual_arm_loco_skeleton）
- 运行：./build/example/r1/r1_dual_arm_loco_skeleton <网卡> --ik --move
## 6. PICO HandLink UDP 接入（--pico）

新增模块：
- `r1_pico_udp.h` —— PICOHandLink UDP-JSON v3 接收与解析（POSIX UDP + unitree::common::json）
- `r1_xr_pose_alignment.h` —— OpenXR→宇树机器人参考系对齐（移植 televuer tv_wrapper.py）

数据链路：

```
PICO HandLink App (openxr-app, UDP JSON v3, 默认 9999 端口, 100~1000 Hz)
   │  teleop.left/right.pose（校准后 grip pose：position m + pitch/yaw/roll deg）
   │  hmd.pose / safety.* / robot_control.{hands,base,...}
   ▼
r1_pico_udp.h :: PicoUdpThread（最新包 + 收包时间戳）
   ▼
安全闸门：r1_pico_safety_policy.h 判据优先级 急停 > 回零 > 保持 > 遥操
          （急停闩锁 / safe_to_execute+active_stream+收包新鲜(<600ms) / return_zero）
   ▼
位姿重建：R = Rz(yaw)·Ry(pitch)·Rx(roll)（与 PICO 端 EulerDegToQuaternion 互逆）
   ▼
xralign::alignWristToRobot(wrist, hmd, mode)   --pico-frame head_yaw|head_trans|basis
   ▼
R1DualArmIk.solve() → arm.setTargets()（250Hz 内部发布）
   ▼
HandAction：robot_control.hands（0..10000）优先，否则手柄 trigger/grip
底盘：robot_control.base 优先，否则摇杆限幅 0.3 -> LocoClient.Move
```

运行：

```bash
./build/example/r1/r1_dual_arm_loco_skeleton eth0 --pico 9999 --pico-frame head_yaw
```

位姿来源（--pico-source，默认 controllers）：
- `controllers`：PICOHandLink 的 ControllerJson() 用 SelectedLeft/Right(false) 输出，即未应用校准的原始手柄 grip pose（OpenXR app space），与宇树 televuer/vuer 的 WebXR 原始手柄数据同源；对齐方式与官方完全一致（basis + head-yaw + 腰部偏移）。
- `teleop`：ApplyCalibration() 后的位姿（pose - 校准锚点，坐标系已相对化），此时不要再叠加 head 减法，否则双参考系错位；仅供对比调试。

注意：
- 判据优先级见 `r1_pico_safety_policy.h`：急停闩锁 > 一键回零 > 保持 > 正常遥操。
  急停 = 停移动 + 阻尼且不自动复位（需显式站立）；`stop_signal`/心跳超时 → 停 + 阻尼；
  `return_zero` 期间双臂目标归零（PICO 端一键回零），该分支不要求 `active_stream`。
- 位姿已同步 PICO 端新增的 `orientation_quat{x,y,z,w}`：`r1_pico_udp.h` 的 `toOpenXrPose()`
  优先用原始四元数重建姿态（无万向锁），缺省才回退欧拉 ZYX 重建。
- head-yaw 参考系 = 官方 xr_teleoperate 默认；若你的 PICO 校准已把位姿相对化，
  可改用 `--pico-frame head_trans` 或 `basis`，并按需调整腰部位移常量
  （r1_xr_pose_alignment.h 中 +0.15/+0.45 m）。
- 灵巧手仍走 `HandDriver` 接口：PICO 的 hands 命令已解析进 `HandAction`，
  接你自己的手时实现 `HandDriver::update()` 即可。
## 7. 平滑方法（对齐官方，已确认无 slerp）

官方 xr_teleoperate / televuer **不对腕部位姿做四元数 slerp**（全仓库无 slerp 调用；
televuer 的 safe_mat_update 只做矩阵有效性保护，无效帧沿用上一帧）。平滑全部发生在 IK 之后：

| 环节 | 官方做法 | 我们的实现（已对齐） |
|---|---|---|
| 位姿输入 | 原始 4x4 矩阵直接进 solve_ik | PicoPose3.toOpenXrPose()：四元数转 4x4（等效官方矩阵输入） |
| IK 输出 | WeightedMovingFilter([0.4,0.3,0.2,0.1])，窗口 4 帧，未满直通最新值，重复帧跳过 | R1DualArmIk::smooth() 逐条移植（teleop/utils/weighted_moving_filter.py 语义） |
| 主循环频率 | --frequency 默认 30 Hz | PICO 链路目标循环固定 30 Hz |
| 发布端 | 250 Hz + clip_arm_q_target 关节速度限幅 30 rad/s | R1ArmController::publishLoop 250 Hz + clipTargets 30 rad/s |
| IK warm start | 每帧用当前 lowstate 关节角 | ik.solve(..., arm.currentArmQ()) |

> 结论：四元数只在重建 4x4 输入矩阵这一步被使用（官方拿到的本来就是完整矩阵，等效）；
> 平滑沿用官方的关节空间 WMA + 速度限幅，不引入 slerp。
### 灵巧手 6 路原始位置（已透传，控制映射预留）

- `robot_control.hands.left/right` = 每侧 **6 路 0..10000** 的手关节目标位置（`unit`=`raw`）。
- 解析：`r1_pico_udp.h` → `PicoRobotControl.left_hand/right_hand`（已在 `ParsePicoPacket` 中解析）。
- 透传：`makeHandAction()` 把每侧 6 路**原样**写入 `HandAction.left_joint_raw[] / right_joint_raw[]`，
  并置 `left_raw_valid / right_raw_valid = true`；同时给出归一化 `left_finger[]`（raw/10000）供简易驱动参考。
- 预留：**“0..10000 → 各关节实际运动”的换算写在 `HandDriver` 实现里**。
  接入你的灵巧手时继承 `HandDriver`，读 `action.left_joint_raw/right_joint_raw` 转成
  你手上的电机/舵机指令即可（当前 `NullHandDriver` 只打印 6 路数值验证链路）。
- 无 `hands` 字段时的回退：用手柄 `trigger/grip` 映射填 `left_finger[]`。