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
//   - **前置状态要求：机器人已处于 811 主运控**（官方 xr_teleoperate 只支持 Regular mode，
//     且它自己从不切 FSM）。首个可执行包默认**不**调 StandUp()，只解锁底盘速度闸门。
//     若需要旧行为（首包自动 StandUp -> FSM 4），显式加 --auto-stand。
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
// 退出：
//   q / Ctrl+C（SIGINT）/ kill（SIGTERM）/ SSH 掉线或关窗口（SIGHUP）都会走同一条
//   清理路径：join 线程 -> arm.goHomeAndRelease() -> hand.stop() -> arm.stop()，
//   期间 loco 线程收尾 StopMove + Damp。**别用 kill -9** —— 那样进程来不及回零与
//   释放权重，双臂会冻结在最后一帧姿态（且本端不自动复位）。
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
#include <memory>
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
#include "r1_hand_canfd_bridge.h"
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
    "  --auto-stand              PICO 首个可执行包时自动 StandUp（-> FSM 4）。\n"
    "                            默认关：官方遥操不切 FSM，要求机器人已在 811 主运控\n"
    "  --pico-frame head_yaw|head_trans|basis   双臂参考系（默认 head_yaw）\n"
    "  --pico-source controllers|teleop   位姿来源（默认 controllers=原始数据，对齐宇树摇操）\n"
    "  --hand null|canfd[:port]  灵巧手驱动（默认 null=只打印 6 路 raw 验证链路）。\n"
    "                            canfd=把遥操手部位置经 UDP 转发给 scripts/hand_bridge.py，\n"
    "                            由它独占 CANFD 适配器落到手上；需**先起桥**（桥在启动时\n"
    "                            完成 使能→回零→位置模式→速度→电流，约 12 s/两只手）。\n"
    "                            急停/掉包/保持/回零时本端不下发，手保持在最后位置。\n"
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
  bool auto_stand_on_first_packet = false;  // 默认关：不切 FSM（见文件头）
  int pico_port = 9999;
  std::string pico_frame = "head_yaw";
  std::string pico_source = "controllers";  // controllers=原始(对齐宇树) / teleop=校准后端点
  std::string hand_mode = "null";           // 灵巧手驱动：null（默认，只打印）/ canfd（走 UDP 桥）
  int hand_port = 9998;                     // canfd 模式：hand_bridge.py 的监听端口
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
    else if (a == "--auto-stand") auto_stand_on_first_packet = true;
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
    } else if (a == "--hand" && i + 1 < argc) {
      hand_mode = argv[++i];
      // 支持 "canfd:9998" 形式就地指定桥的 UDP 端口
      const auto colon = hand_mode.find(':');
      if (colon != std::string::npos) {
        hand_port = std::atoi(hand_mode.c_str() + colon + 1);
        hand_mode = hand_mode.substr(0, colon);
      }
      if (hand_mode != "null" && hand_mode != "canfd") {
        std::cerr << "[main] unknown --hand: " << hand_mode
                  << " (expect null|canfd[:port])" << std::endl;
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

  // ---- 灵巧手 ----
  // null（默认）：只打印 6 路 raw 验证链路；canfd：把位置经 UDP 转发给
  // scripts/hand_bridge.py，由它独占 CANFD 适配器落到手上（见 r1_hand_canfd_bridge.h）。
  std::unique_ptr<r1skeleton::HandDriver> hand;
  if (hand_mode == "canfd") {
    hand = std::make_unique<r1skeleton::CanfdBridgeHandDriver>(
        static_cast<uint16_t>(hand_port));
  } else {
    hand = std::make_unique<r1skeleton::NullHandDriver>();
  }
  r1skeleton::HandState hand_state;
  hand->init();

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
  // PICO 首个可执行包：只解锁"底盘速度闸门"，**默认不去切 FSM**。
  // 官方 xr_teleoperate 从不切 FSM（明确要求事先处于运控态、只支持 Regular mode），
  // 我们原先的自动 StandUp() 会把 811 拉到 4，正好撞上"FSM 4 认不认 rt/arm_sdk"这个悬案。
  std::atomic<bool> first_exec_pkt{false};
  std::atomic<double> cmd_vx{vx}, cmd_vy{vy}, cmd_vyaw{vyaw};

  std::thread loco_thread([&]() {
    unitree::robot::r1::LocoClient client;
    client.Init();
    client.SetTimeout(10.f);
    // loco_ready 语义 = "允许下发速度"，**不代表我们改过 FSM**。
    // 仅当走 do_stand 分支时才真的切档；first_exec_pkt 分支默认保持当前 FSM。
    bool loco_ready = false;
    std::cout << "[loco] LocoClient ready." << std::endl;
    while (!g_quit.load()) {
      if (damp_cmd.exchange(false)) {
        client.Damp();
        loco_ready = false;
        std::cout << "[loco] Damp." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      bool do_stand = stand_cmd.exchange(false);   // stdin 's'
      if (!pico_mode && auto_move) {               // --move：只自动站一次
        do_stand = true;
        auto_move = false;
      }
      if (do_stand) {
        client.StandUp();
        loco_ready = true;
        std::cout << "[loco] StandUp." << std::endl;
      } else if (first_exec_pkt.exchange(false)) {
        if (auto_stand_on_first_packet) {
          client.StandUp();
          std::cout << "[loco] StandUp. (PICO 首包 + --auto-stand ⇒ FSM 4)" << std::endl;
        } else {
          std::cout << "[loco] PICO 首个可执行包：保持当前 FSM，不自动切档。"
                       "底盘速度已解锁（腿要动仍需 811 主运控）。" << std::endl;
        }
        loco_ready = true;
      }
      if (loco_ready) {
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
  std::signal(SIGTERM, onSigInt);
  // SSH 掉线 / 终端窗口被关时，内核给前台进程组发的是 SIGHUP，默认动作 = 立即终止进程。
  // 只接 SIGINT 的话，掉线会让进程来不及执行下面的 goHomeAndRelease() + Damp，
  // 权重停在 100 且不再发布 ⇒ **双臂冻结在最后一帧姿态**（官方明确警告的失效模式）。
  // 接住它，走与 Ctrl+C 完全相同的清理路径。
  std::signal(SIGHUP, onSigInt);
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
    if (!auto_stand_on_first_packet) {
      std::cout << "[pico] 首包不切 FSM（官方遥操前置：机器人已在 811 主运控）。"
                   "腿不动就先执行 example/r1/high_level/scripts/loco.sh start；"
                   "要旧的自动站立行为请加 --auto-stand" << std::endl;
    }
    bool first_packet = true;
    bool ever_safe = false;
    bool estop_active = false;         // 急停闩锁的边沿检测（进/出各日志一次）
    // 掉包/stop_signal 阻尼的边沿检测（与 estop_active 同源问题）。
    // ⚠️ 少了它就会按 30 Hz 刷屏：tryGet() 只要收到过**至少一个**包，之后就会永远返回
    //    最后一个包，且 rx_age_ms 单调增长 ⇒ 每帧都判「掉包 → 保持 → 触发阻尼」
    //    ⇒ loco 线程每帧 client.Damp() 并打印一行。Damp 幂等（重复发无害），
    //    但会把真正有用的日志淹掉，正是掉包时最需要看日志的时候。
    bool damp_latched = false;
    double next_damp_resend_s = 0.0;   // 闩锁期间 2 Hz 重发 Damp（DDS 尽力而为，单发可能丢）
    bool opmode_warned = false;        // operator_mode 缺失只告警一次
    double next_pose_warn_s = 0.0;     // 位姿缺失告警限速（1 Hz）
    const auto nowSeconds = [] {
      return std::chrono::duration<double>(
                 std::chrono::steady_clock::now().time_since_epoch()).count();
    };
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
                  << " model=" << pkt.robot.model
                  << " sdk=\"" << pkt.sdk << "\""
                  << " operator_mode=\"" << pkt.operator_mode << "\"" << std::endl;
        first_packet = false;
      }

      // 诊断：报文里根本没有 operator_mode 键 ⇒ 解析端 fail-safe 兜底成 "stop_signal"
      // ⇒ safeToExecute() 恒 false ⇒ 每个包都进 kHold 并 Damp()，腿一直是软的。
      // 现象与"包收得很正常"无法区分，必须显式告警。最常见原因：装了旧的 :app
      // （sdk=pico-openxr-bridge，从不发该字段），而不是实机主 App :openxr-app。
      if (!opmode_warned && !pkt.operator_mode_present) {
        opmode_warned = true;
        std::cout << "[pico] ⚠ 报文缺少 operator_mode 字段，已按 stop_signal 兜底 ⇒ 不会遥操。\n"
                     "       请确认头显里装的是 :openxr-app（sdk=\"pico-openxr-single\"），"
                     "不是旧的 :app（\"pico-openxr-bridge\"）。本包 sdk=\""
                  << pkt.sdk << "\"" << std::endl;
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
          hand->update(idle, hand_state, pico_dt);
          break;
        }
        // ② 一键回零：双臂目标归零，底盘停止。只要求包新鲜——回零期间 operator_mode
        //    不是 active_stream，用 safeToExecute() 兜底会让该分支永不可达。
        case r1skeleton::pico::Disposition::kReturnZero: {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
          arm.setTargets(Eigen::VectorXd::Zero(2 * n), zero_tau);
          hand->update(idle, hand_state, pico_dt);
          break;
        }
        // ③ 保持：未安全/掉包/来源无效。不更新双臂目标（250Hz 内部循环保持上次目标），
        //    底盘速度归零；掉包或 stop_signal 时额外触发阻尼。
        case r1skeleton::pico::Disposition::kHold: {
          cmd_vx.store(0); cmd_vy.store(0); cmd_vyaw.store(0);
          // 掉包 / stop_signal → 阻尼。**边沿触发 + 2 Hz 重发**（2026-10-08）：
          //   已闩锁时不再打日志（原来会 30 Hz 刷屏），但仍每 0.5 s 重发一次命令 ——
          //   DDS 是尽力而为，只发一次有可能丢，丢了腿就不会变软（这是安全相关的动作）。
          if (r1skeleton::pico::holdTriggersDamp(pkt, rx_age_ms, kStaleMs)) {
            const double now = nowSeconds();
            if (!damp_latched) {
              damp_latched = true;
              next_damp_resend_s = now + 0.5;
              damp_cmd.store(true);
              std::cout << "[pico] 掉包或 stop_signal：进入阻尼（腿变软），双臂保持。"
                           "恢复发送后需操作者显式重新站立（stdin 's'），本端不自动复位。"
                           "（本行只打一次，不再逐帧刷屏）" << std::endl;
            } else if (now >= next_damp_resend_s) {
              next_damp_resend_s = now + 0.5;
              damp_cmd.store(true);
            }
          } else {
            damp_latched = false;   // 包恢复新鲜 → 解除闩锁（下次掉包会重新提示）
          }
          hand->update(idle, hand_state, pico_dt);
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

      // 只要本帧不是「保持」，链路/状态就已经变了 ⇒ 解除掉包阻尼闩锁。
      // 少了这一句闩锁会一直粘住：掉包→恢复→再掉包时不会重新提示、也不再触发 Damp。
      if (disp != r1skeleton::pico::Disposition::kHold) damp_latched = false;

      if (disp != r1skeleton::pico::Disposition::kTeleop) {
        std::this_thread::sleep_for(std::chrono::duration<double>(pico_dt));
        continue;
      }

      // 首个可执行包：只解锁底盘速度闸门，**默认不切 FSM**（见文件头与 loco 线程注释）。
      if (!ever_safe) {
        ever_safe = true;
        first_exec_pkt.store(true);
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
      } else {
        // 判据已放行 kTeleop（quality == "live"），但 JSON 里连 pose 对象都没有
        // ⇒ 双臂**静默不动、且一行日志都没有**。这与"权重 100 但什么都不动"的
        // 历史故障现象完全一致，极难排查，故显式限速告警（1 Hz）。
        const double now = nowSeconds();
        if (now >= next_pose_warn_s) {
          next_pose_warn_s = now + 1.0;
          std::cout << "[pico] ⚠ 已放行遥操但位姿缺失：source=" << pico_source
                    << " left.pose=" << (leftS.pose.valid ? "ok" : "缺失")
                    << " right.pose=" << (rightS.pose.valid ? "ok" : "缺失")
                    << " ⇒ 双臂不会动。检查 App「发送内容」页是否勾选了该模块。"
                    << std::endl;
        }
      }

      // ——灵巧手——
      const r1skeleton::HandAction action = makeHandAction(leftS, rightS, pkt.robot);
      hand->update(action, hand_state, pico_dt);

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
      hand->update(action, hand_state, dt);

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
      hand->update(action, hand_state, dt);

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
  hand->stop();
  arm.stop();
  std::cout << "[main] done." << std::endl;
  return 0;
}
