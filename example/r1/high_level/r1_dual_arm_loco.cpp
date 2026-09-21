// ============================================================================
// r1_dual_arm_loco.cpp
// R1 双线程控制骨架：LocoClient（运动） + ArmSdk（双臂并行接管）+ 手部接口预留。
//
// 用法：
//   ./r1_dual_arm_loco_skeleton <网卡名> [--variant a5|a7] [--motion|--lowcmd]
//         [--ik] [--move] [--vx 0.1] [--vy 0.0] [--vyaw 0.0]
//         [--pico [port]] [--pico-frame head_yaw|head_trans|basis]
//
// --pico 模式（PICOHandLink UDP-JSON v3 接入）：
//   - PICO 端 App 通过 UDP 发送 JSON（默认端口 9999，可按包内 transport.target_port 调整）。
//   - 本端只在 safety.safe_to_execute == true（且 operator_mode == active_stream）时执行；
//     return_zero 期间双臂目标归零；stop_signal / 心跳超时 -> 停止（停止移动 + 阻尼）。
//   - 双臂目标 = teleop.left/right.pose（校准后 grip pose）-> 欧拉(deg,ZYX) 重建 4x4
//     -> OpenXR->Robot 对齐（见 r1_xr_pose_alignment.h，默认 head-yaw 参考系）
//     -> R1ArmIk.solve() -> R1ArmController.setTargets()。
//   - 手中命令优先取 robot_control.hands（0..10000），否则用手柄 trigger/grip 映射。
//   - 底盘速度优先取 robot_control.base（linear_x/y, angular_z），否则手柄摇杆限幅 0.3。
//
// 非 --pico 模式：
//   运行互动（stdin）：v vx vy vyaw | s | d | t | q
//   --ik 任务空间演示 / 默认关节空间 movej 演示。
//
// 线程结构：
//   A: Loco 线程（r1::LocoClient）
//   B: 本线程（main）：目标生成（IK 或关节空间）+ 手部 update()
//   C: R1ArmController 内部 250Hz 发布线程
//   D: PICO UDP 接收线程（--pico 模式）
// ============================================================================

#include <csignal>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <poll.h>
#include <unistd.h>

#include <eigen3/Eigen/Dense>

// Unitree SDK
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/r1/loco/r1_loco_client.hpp"

// R1 骨架
#include "r1_arm_ik.h"
#include "r1_arm_controller.h"
#include "r1_hand_interface.h"
#include "r1_pico_udp.h"
#include "r1_pico_safety_policy.h"
#include "r1_xr_pose_alignment.h"

namespace {

std::atomic<bool> g_quit{false};

void onSigInt(int) { g_quit.store(true); }

bool isNumber(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) if (!std::isdigit(static_cast<unsigned char>(c))) return false;
  return true;
}

void printUsage(const char* prog) {
  std::cout <<
    "Usage: " << prog << " <network_interface> [options]\n"
    "  --variant a5|a7            R1 手臂型号（默认 a5）\n"
    "  --motion                  发布到 rt/arm_sdk 覆盖模式（默认，行走时并行接管双臂）\n"
    "  --lowcmd                  发布到 rt/lowcmd 全关节模式（需先释放机载运动服务）\n"
    "  --ik                      任务空间演示（默认关节空间 movej；--pico 时忽略）\n"
    "  --move                    自动站立并按 --vx/--vy/--vyaw 持续移动\n"
    "  --pico [port]             PICOHandLink UDP 接入（默认端口 9999）\n"
    "  --pico-frame head_yaw|head_trans|basis   双臂参考系（默认 head_yaw）\n"
    "  --pico-source controllers|teleop   位姿来源（默认 controllers=原始数据，对齐宇树摇操）\n"
    "  --vx --vy --vyaw          初始移动速度\n"
    "stdin(非pico): v vx vy vyaw | s | d | t | q\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }
  const std::string iface = argv[1];

  std::string variant_str = "a5";
  bool ik_demo = false;
  bool auto_move = false;
  bool motion_mode = true;
  bool pico_mode = false;
  int pico_port = 9999;
  std::string pico_frame = "head_yaw";
  std::string pico_source = "controllers";  // controllers=原始(对齐宇树) / teleop=校准后端点
  double vx = 0.0, vy = 0.0, vyaw = 0.0;

  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--variant" && i + 1 < argc) variant_str = argv[++i];
    else if (a == "--ik") ik_demo = true;
    else if (a == "--move") auto_move = true;
    else if (a == "--lowcmd") motion_mode = false;
    else if (a == "--motion") motion_mode = true;
    else if (a == "--vx" && i + 1 < argc) vx = std::atof(argv[++i]);
    else if (a == "--vy" && i + 1 < argc) vy = std::atof(argv[++i]);
    else if (a == "--vyaw" && i + 1 < argc) vyaw = std::atof(argv[++i]);
    else if (a == "--pico") {
      pico_mode = true;
      if (i + 1 < argc && isNumber(argv[i + 1])) pico_port = std::atoi(argv[++i]);
    } else if (a == "--pico-frame" && i + 1 < argc) {
      pico_frame = argv[++i];
      if (pico_frame != "head_yaw" && pico_frame != "head_trans" && pico_frame != "basis") {
        std::cerr << "[main] unknown --pico-frame: " << pico_frame << std::endl;
        return 1;
      }
    } else if (a == "--pico-source" && i + 1 < argc) {
      pico_source = argv[++i];
      if (pico_source != "controllers" && pico_source != "teleop") {
        std::cerr << "[main] unknown --pico-source: " << pico_source << std::endl;
        return 1;
      }
    }
  }

  if (variant_str != "a5" && variant_str != "a7") {
    std::cerr << "[main] unknown --variant: " << variant_str << " (expect a5|a7)" << std::endl;
    return 1;
  }
  const bool use_a7 = (variant_str == "a7");
  // R1_A7 的固件未开放 rt/arm_sdk 覆盖模式（官方 xr_teleoperate 直接 raise）。
  // 默认 motion_mode=true，A7 下必须显式改走 rt/lowcmd 全关节模式。
  if (use_a7 && motion_mode) {
    std::cerr << "[main] R1_A7 不支持 rt/arm_sdk (motion_mode)。"
              << "如需控制 A7，请显式添加 --lowcmd（使用前须先释放机载运控服务）。" << std::endl;
    return 1;
  }
  std::cout << "[main] R1 " << (use_a7 ? "A7" : "A5")
            << " motion_mode=" << motion_mode << " iface=" << iface
            << (pico_mode ? (" pico_mode port=" + std::to_string(pico_port)) : "")
            << std::endl;

  // ---- DDS 初始化（必须先于所有 Publisher/Subscriber） ----
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);

  // ---- 手臂控制器（内部：lowstate 订阅 + 250Hz 发布） ----
  r1skeleton::R1ArmController::Params arm_params;
  arm_params.motion_mode = motion_mode;
  r1skeleton::R1ArmController arm(
      use_a7 ? r1skeleton::R1ArmController::Variant::R1_A7
             : r1skeleton::R1ArmController::Variant::R1_A5,
      arm_params);

  try {
    arm.start();
  } catch (const std::exception& e) {
    std::cerr << "[main] arm controller start failed: " << e.what() << std::endl;
    return 2;
  }

  // ---- 灵巧手（预留接口，默认空驱动） ----
  r1skeleton::NullHandDriver hand;
  r1skeleton::HandState hand_state;
  hand.init();

  // ---- PICO UDP 接收线程 ----
  r1skeleton::pico::PicoUdpThread pico_udp(static_cast<uint16_t>(pico_port));
  std::atomic<bool> safe_to_move{!pico_mode};  // 运动执行闸门：PICO 模式下收到首个安全包后才置 true
  if (pico_mode) {
    if (!pico_udp.start()) {
      std::cerr << "[main] failed to start PICO UDP receiver on port "
                << pico_port << std::endl;
      return 3;
    }
  }

  // ---- 运动控制（LocoClient 线程） ----
  std::atomic<bool> damp_cmd{false};
  std::atomic<bool> stand_cmd{false};
  std::atomic<bool> auto_stand_pico{false};  // PICO 首个安全包后自动站立一次
  std::atomic<double> cmd_vx{vx}, cmd_vy{vy}, cmd_vyaw{vyaw};

  std::thread loco_thread([&]() {
    unitree::robot::r1::LocoClient client;
    client.Init();
    client.SetTimeout(10.f);
    bool standing = false;
    std::cout << "[loco] LocoClient ready." << std::endl;
    while (!g_quit.load()) {
      if (damp_cmd.exchange(false)) {
        client.Damp();
        standing = false;
        std::cout << "[loco] Damp." << std::endl;
      } else if (stand_cmd.exchange(false) || auto_stand_pico.exchange(false) || (!pico_mode && auto_move)) {
        client.StandUp();
        standing = true;
        auto_move = false;  // 只自动站一次，此后按 stdin 控制
        std::cout << "[loco] StandUp." << std::endl;
      } else if (standing) {
        if (pico_mode && !safe_to_move.load()) {
          // PICO 安全闸门关闭：停止移动（保持站立）
          client.StopMove();
        } else {
          client.Move(static_cast<float>(cmd_vx.load()),
                      static_cast<float>(cmd_vy.load()),
                      static_cast<float>(cmd_vyaw.load()),
                      true);  // continuous move
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
    }
    client.StopMove();
    client.Damp();
    std::cout << "[loco] stopped." << std::endl;
  });

  // ---- stdin 指令线程（支持手动控制/紧急命令） ----
  // 用 poll + read 代替 std::getline：getline 会阻塞，导致 q/Ctrl+C 后
  // 主线程 join 卡死、无法执行回零与释放。poll 带超时以轮询 g_quit。
  std::thread stdin_thread([&]() {
    std::string pending;
    while (!g_quit.load()) {
      struct pollfd pfd;
      pfd.fd = STDIN_FILENO;
      pfd.events = POLLIN;
      const int pr = ::poll(&pfd, 1, 100);  // 100ms 超时，便于及时响应退出
      if (pr < 0) {
        if (errno == EINTR) continue;       // 被信号打断，重试
        break;
      }
      if (pr == 0) continue;                // 超时，无输入
      if (!(pfd.revents & (POLLIN | POLLHUP))) continue;

      char buf[256];
      const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
      if (n <= 0) break;                    // EOF（如 stdin 关闭/重定向结束）
      pending.append(buf, static_cast<size_t>(n));

      size_t pos;
      while (!g_quit.load() && (pos = pending.find('\n')) != std::string::npos) {
        const std::string line = pending.substr(0, pos);
        pending.erase(0, pos + 1);
        std::istringstream iss(line);
        std::string tok;
        iss >> tok;
        if (tok == "v") {
          double x = cmd_vx.load(), y = cmd_vy.load(), w = cmd_vyaw.load();
          iss >> x >> y >> w;
          cmd_vx.store(x); cmd_vy.store(y); cmd_vyaw.store(w);
          std::cout << "[cmd] velocity = " << x << " " << y << " " << w << std::endl;
        } else if (tok == "s") {
          stand_cmd.store(true);
        } else if (tok == "d") {
          damp_cmd.store(true);
        } else if (tok == "t") {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
        } else if (tok == "q") {
          g_quit.store(true);
        } else if (!tok.empty()) {
          printUsage(argv[0]);
        }
      }
    }
  });

  // ---- 双臂目标生成（arm 线程，跑在 main） ----
  std::signal(SIGINT, onSigInt);
  std::cout << "[main] ready." << (pico_mode ? " 等待 PICO HandLink 数据..." : "")
            << " (stdin: 'v vx vy vyaw' / 's' / 'd' / 't' / 'q', Ctrl+C 退出)" << std::endl;

  const int n = arm.dofPerArm();
  r1skeleton::R1DualArmIk ik(use_a7 ? r1skeleton::R1ArmKinematics::R1_A7
                                    : r1skeleton::R1ArmKinematics::R1_A5);
  const double dt = 0.01;  // 100 Hz 目标生成
  Eigen::VectorXd q = arm.currentArmQ();
  Eigen::VectorXd zero_tau = Eigen::VectorXd::Zero(2 * n);

  // 手部动作映射（PICO -> HandAction）
  // PICO 的 robot_control.hands 每侧 6 路 0..10000，是手各关节的目标位置。
  // 骨架把 6 路原样透传到 HandAction.left/right_joint_raw；
  // "0..10000 -> 各关节实际运动"的换算预留给你的 HandDriver 实现。
  auto makeHandAction = [](const r1skeleton::pico::PicoSide& L,
                           const r1skeleton::pico::PicoSide& R,
                           const r1skeleton::pico::PicoRobotControl& rc) {
    r1skeleton::HandAction a;
    a.mode = r1skeleton::HandAction::Mode::kPose;
    a.left_enabled = L.valid;
    a.right_enabled = R.valid;

    // 6 路原始关节位置（0..10000），原样透传
    if (rc.has_hands && rc.left_hand.size() >= 6) {
      for (int i = 0; i < 6; ++i) a.left_joint_raw[i] = rc.left_hand[static_cast<size_t>(i)];
      a.left_raw_valid = true;
    }
    if (rc.has_hands && rc.right_hand.size() >= 6) {
      for (int i = 0; i < 6; ++i) a.right_joint_raw[i] = rc.right_hand[static_cast<size_t>(i)];
      a.right_raw_valid = true;
    }

    // 归一化手指目标（仅供简易驱动参考；权威数据是上面的 raw 6 路）
    auto fillFingers = [&a](std::array<double, 5>& f, const std::array<double, 6>& raw,
                            bool raw_valid, const r1skeleton::pico::PicoInput& in, bool valid) {
      if (raw_valid) {
        for (int i = 0; i < 5; ++i) f[i] = std::clamp(raw[static_cast<size_t>(i)] / 10000.0, 0.0, 1.0);
      } else {
        const double grip = valid ? std::clamp(in.grip, 0.0, 1.0) : 0.0;
        const double trig = valid ? std::clamp(in.trigger, 0.0, 1.0) : 0.0;
        f = {trig, trig, grip, grip, grip};
      }
    };
    fillFingers(a.left_finger, a.left_joint_raw, a.left_raw_valid, L.input, L.valid);
    fillFingers(a.right_finger, a.right_joint_raw, a.right_raw_valid, R.input, R.valid);
    return a;
  };

  if (pico_mode) {
    // ==================== PICO 遥操作链路 ====================
    r1skeleton::xralign::RefMode ref_mode = r1skeleton::xralign::RefMode::kHeadYaw;
    if (pico_frame == "head_trans") ref_mode = r1skeleton::xralign::RefMode::kHeadTranslation;

    constexpr double kTargetHz = 30.0;   // 官方 args.frequency 默认值
    constexpr double kStaleMs = r1skeleton::pico::kDefaultStaleMs;  // 掉包判据（停 + 阻尼）
    const double pico_dt = 1.0 / kTargetHz;
    std::cout << "[pico] frame mode: " << pico_frame
              << " | target loop " << kTargetHz << " Hz (官方 teleop 默认 30 Hz)"
              << " | WMA 关节平滑 4 帧 [0.4,0.3,0.2,0.1] + 250Hz 发布速度限幅 30 rad/s" << std::endl;
    bool first_packet = true;
    bool ever_safe = false;
    bool estop_active = false;         // 急停闩锁的边沿检测（进/出各日志一次）
    while (!g_quit.load()) {
      r1skeleton::pico::PicoTeleopPacket pkt;
      double rx_age_ms = 0.0;
      if (!pico_udp.tryGet(pkt, rx_age_ms)) {
        // 还没收到任何包：保持当前目标，什么都不发新指令
        std::this_thread::sleep_for(std::chrono::duration<double>(pico_dt));
        continue;
      }
      if (first_packet) {
        std::cout << "[pico] first packet seq=" << pkt.sequence
                  << " model=" << pkt.robot.model << std::endl;
        first_packet = false;
      }

      // 位姿来源：controllers=原始(无校准，与宇树 televuer 同源) / teleop=校准后端点
      const r1skeleton::pico::PicoSide leftS = (pico_source == "teleop") ? pkt.left : pkt.ctrl.left;
      const r1skeleton::pico::PicoSide rightS = (pico_source == "teleop") ? pkt.right : pkt.ctrl.right;
      const bool src_valid = (pico_source == "teleop") ? (pkt.teleop_valid && pkt.left.valid && pkt.right.valid)
                                                       : pkt.ctrl.output_valid;

      // 判据优先级集中在 r1_pico_safety_policy.h：急停 > 回零 > 保持 > 遥操
      const auto disp = r1skeleton::pico::decideDisposition(pkt, rx_age_ms, src_valid, kStaleMs);
      safe_to_move.store(disp == r1skeleton::pico::Disposition::kTeleop);  // 回零/急停/保持期间底盘不动

      r1skeleton::HandAction idle;
      idle.mode = r1skeleton::HandAction::Mode::kIdle;

      switch (disp) {
        // ① 急停闩锁：停移动 + 进入阻尼（对齐官方 xr_teleoperate「双摇杆按下 = 软急停
        //    = Damp()」），双臂保持最后目标、不再接收新指令。恢复必须由操作者显式
        //    重新站立（PICO 站立键或 stdin 's'）——本端不自动复位，避免闩锁解除瞬间
        //    机器人自行起身，与官方行为一致。
        case r1skeleton::pico::Disposition::kEmergencyStop: {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
          if (!estop_active) {
            estop_active = true;
            damp_cmd.store(true);
            std::cout << "[pico] 急停闩锁：停止移动 + 阻尼，双臂保持。"
                         "恢复需操作者显式重新站立（stdin 's'），本端不自动复位。"
                      << std::endl;
          }
          hand.update(idle, hand_state, pico_dt);
          break;
        }
        // ② 一键回零：双臂目标归零，底盘停止。只要求包新鲜——回零期间 operator_mode
        //    不是 active_stream，用 safeToExecute() 兜底会让该分支永不可达。
        case r1skeleton::pico::Disposition::kReturnZero: {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
          arm.setTargets(Eigen::VectorXd::Zero(2 * n), zero_tau);
          hand.update(idle, hand_state, pico_dt);
          break;
        }
        // ③ 保持：未安全/掉包/来源无效。不更新双臂目标（250Hz 内部循环保持上次目标），
        //    底盘速度归零；掉包或 stop_signal 时额外触发阻尼。
        case r1skeleton::pico::Disposition::kHold: {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
          if (r1skeleton::pico::holdTriggersDamp(pkt, rx_age_ms, kStaleMs)) damp_cmd.store(true);
          hand.update(idle, hand_state, pico_dt);
          break;
        }
        case r1skeleton::pico::Disposition::kTeleop:
          break;
      }

      // 急停解除的边沿日志（disp != kEmergencyStop 时才会走到这里）
      if (disp != r1skeleton::pico::Disposition::kEmergencyStop && estop_active) {
        estop_active = false;
        std::cout << "[pico] 急停闩锁已解除（等待操作者重新站立）。" << std::endl;
      }

      if (disp != r1skeleton::pico::Disposition::kTeleop) {
        std::this_thread::sleep_for(std::chrono::duration<double>(pico_dt));
        continue;
      }

      // 首个安全包：让机器人站立（此后由 PICO 底盘指令控制）
      if (!ever_safe) {
        ever_safe = true;
        auto_stand_pico.store(true);
      }

      // ——双臂：所选来源位姿 -> OpenXR 4x4 -> 对齐 -> IK——
      if (leftS.pose.valid && rightS.pose.valid) {
        const Eigen::Isometry3d Lxr = leftS.pose.toOpenXrPose();
        const Eigen::Isometry3d Rxr = rightS.pose.toOpenXrPose();
        Eigen::Isometry3d head = pkt.hmd_live ? pkt.hmd_pose.toOpenXrPose()
                                              : r1skeleton::xralign::defaultHeadPose();
        Eigen::Isometry3d Lrobot, Rrobot;
        if (pico_frame == "basis") {            // 只做基线系：留给调用方自行参考
          Lrobot = r1skeleton::xralign::basisToRobot(Lxr);
          Rrobot = r1skeleton::xralign::basisToRobot(Rxr);
        } else {
          Lrobot = r1skeleton::xralign::alignWristToRobot(Lxr, head, ref_mode);
          Rrobot = r1skeleton::xralign::alignWristToRobot(Rxr, head, ref_mode);
        }
        q = ik.solve(Lrobot, Rrobot, arm.currentArmQ());
        arm.setTargets(q, zero_tau);
      }

      // ——灵巧手——
      const r1skeleton::HandAction action = makeHandAction(leftS, rightS, pkt.robot);
      hand.update(action, hand_state, pico_dt);

      // ——底盘速度：优先 robot_control.base，否则手柄摇杆（限幅 0.3）——
      if (pkt.robot.has_base) {
        cmd_vx.store(pkt.robot.base_linear_x);
        cmd_vy.store(pkt.robot.base_linear_y);
        cmd_vyaw.store(pkt.robot.base_angular_z);
      } else {
        cmd_vx.store(std::clamp(-leftS.input.thumb_y * 0.3, -0.3, 0.3));
        cmd_vy.store(std::clamp(-leftS.input.thumb_x * 0.3, -0.3, 0.3));
        cmd_vyaw.store(std::clamp(-rightS.input.thumb_x * 0.3, -0.3, 0.3));
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(pico_dt));
    }
  } else if (ik_demo) {
    // ==================== 任务空间演示 ====================
    const r1skeleton::Iso Lt0 = ik.kinematics().forward(q.head(n), true);
    const r1skeleton::Iso Rt0 = ik.kinematics().forward(q.tail(n), false);
    double t = 0.0;
    while (!g_quit.load()) {
      const double w = 2.0 * M_PI / 8.0;  // 8 s 一个周期
      t += dt;

      r1skeleton::Iso Lt = Lt0;
      Lt.translate(Eigen::Vector3d(0.0, 0.12 * std::sin(w * t),
                                   0.06 * (1.0 - std::cos(w * t))));
      r1skeleton::Iso Rt = Rt0;
      Rt.translate(Eigen::Vector3d(0.0, -0.12 * std::sin(w * t),
                                   0.06 * (1.0 - std::cos(w * t))));

      q = ik.solve(Lt, Rt, arm.currentArmQ());
      arm.setTargets(q, zero_tau);

      r1skeleton::HandAction action;
      action.mode = r1skeleton::HandAction::Mode::kPose;
      action.left_enabled = action.right_enabled = true;
      const double grip = 0.5 + 0.5 * std::sin(w * t);
      for (int i = 0; i < 5; ++i) {
        action.left_finger[i] = grip;
        action.right_finger[i] = grip;
      }
      hand.update(action, hand_state, dt);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    // ==================== 关节空间演示 ====================
    Eigen::VectorXd q_goal(2 * n);
    if (n == 5) {
      q_goal << 0.0, 0.6, 0.0, 1.0, 0.2,     // 左臂微抬
               0.0, -0.6, 0.0, 1.0, -0.2;    // 右臂微抬
    } else {
      q_goal << 0.0, 0.6, 0.0, 1.0, 0.2, 0.0, 0.0,
               0.0, -0.6, 0.0, 1.0, -0.2, 0.0, 0.0;
    }
    double t = 0.0;
    const Eigen::VectorXd q_home = Eigen::VectorXd::Zero(2 * n);
    while (!g_quit.load()) {
      t += dt;
      const double seg = 3.0;  // 3 s 走完一半
      double frac = std::fmod(t, 2.0 * seg) / seg;
      if (frac > 1.0) frac = 2.0 - frac;  // 三角波 0->1->0
      Eigen::VectorXd qd = q_home + frac * (q_goal - q_home);
      arm.setTargets(qd, zero_tau);

      r1skeleton::HandAction action;
      action.mode = r1skeleton::HandAction::Mode::kPose;
      action.left_enabled = action.right_enabled = true;
      const double grip = 0.5 + 0.5 * std::sin(2.0 * M_PI * t / 4.0);
      for (int i = 0; i < 5; ++i) {
        action.left_finger[i] = grip;
        action.right_finger[i] = grip;
      }
      hand.update(action, hand_state, dt);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // ---- 清理：先退出各线程，再回零 + 释放臂 SDK + 阻尼停机 ----
  std::cout << "[main] shutting down..." << std::endl;
  g_quit.store(true);
  if (stdin_thread.joinable()) stdin_thread.join();
  if (loco_thread.joinable()) loco_thread.join();
  pico_udp.stop();
  arm.goHomeAndRelease();
  hand.stop();
  arm.stop();
  std::cout << "[main] done." << std::endl;
  return 0;
}
