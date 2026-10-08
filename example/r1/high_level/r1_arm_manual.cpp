// ============================================================================
// r1_arm_manual.cpp — R1 双臂关节角度手动调试工具（交互式）
//
// 用途：上遥操程序之前，分两步人工验证「手臂接管」这条链路：
//   第 1 步：机器人【锁定站立模式】(FSM 4)
//            —— 宇树官方推荐的手臂 SDK 测试姿态（"建议将机器人悬挂并进入
//               锁定站立模式"，见 G1 developer《手臂控制例程》）。
//   第 2 步：机器人【走跑运控模式】(FSM 811)
//            —— 下肢走跑运控照常，双臂被并行接管。
//   两步用的是同一个程序，差别只在机器人当前处于哪个 FSM（本程序不切 FSM）。
//
// 机制：订阅 rt/lowstate，按 <rate> Hz 发布 rt/arm_sdk（unitree_hg::msg::dds_::LowCmd_）。
//   - mode_pr 作接管权重（0..100）。100 = 双臂完全交给本程序；0 = 不干预，
//     双臂仍由机载 ai_sport 控制。
//   - 下肢完全不受影响：腿/踝槽位一律不写，由机载运控负责平衡与行走。
//   - 槽位与官方一致：左臂 15-19、右臂 22-26（A5 每臂 5 自由度，共 10 个臂关节）；
//     腰 13(Yaw)/头 29,30 只做「就地保持」，本程序不主动改变它们。
//     （臂 SDK 的覆盖范围含头/腰，所以必须给它们写入当前位置作为目标，
//       否则权重生效瞬间头/腰会被拉到 0 位。槽位 12 = 腰部 Roll 由内置运控独占，
//       写它不生效，这里仍写"当前实测值"，与 r1_arm_controller.h 保持同构。）
//   - 关节软限位取自官方 A5 模型 submodules/xr_teleoperate/assets/r1/r1_a5.urdf；
//     每帧再做速度限幅（--vel，默认 2.0 rad/s）。
//     ⚠️ 限幅作用在**指令轨迹的积分**上（cmd += clamp(目标-cmd, ±vel*dt)），不是
//        "实测 + 每帧一小步"。后者在有静摩擦/死区的关节上永远推不动电机
//        （09-23 真机踩到，详见 publishOnce 里的长注释）。
//   - 电机故障监控：任一双臂关节 motorstate != 0 时，立即把权重降到 0 并闩锁
//     （宇树 R1《上肢控制例程》原文：「motorstate 非 0 时应立即停止下发并释放控制权」）。
//
// 依据《R1 上肢控制例程（arm_sdk）》的几条硬约束：
//   - 权重承载于 mode_pr（不是写进某个关节）；0.0=运控独占，1.0=完全交给用户程序。
//   - 接管前必须先把目标初始化为**当前实测位置**，否则会跳变。
//   - 释放不要从 1.0 直接置 0，要在 ~1 s 内线性降权，期间持续发布。
//   - 官方建议发布频率 100 Hz（可用 --rate 100 对齐；本项目遥操主程序用 250 Hz）。
//   - 接口**不做安全限位与碰撞检测**；目标跳变会高速甩臂。
//   - 程序异常崩溃（权重停在 100 且不再发布）时，**上肢会冻结在最后一帧姿态**，
//     不会自动回落给运控 ⇒ 退出必须走 q / Ctrl+C，**绝不要 kill -9**。
//     SIGHUP 已接住：SSH 掉线 / 终端窗口被关时，内核发的就是 SIGHUP，
//     默认动作会让进程当场死亡（手臂冻结）；本程序改为走同一条 1 s 线性交还。
//   - 占用 arm_sdk 期间不要同时调手臂动作服务（会返回 7400 / 指令叠加）。
//
// 运行前置条件（会出事的都在这）：
//   1) 机器人已吊挂/有可靠支撑，臂展范围内无人、无物；
//   2) `sudo systemctl stop r1-custom-head-remote`（该出厂服务 enabled +
//      Restart=always，会与头/腰槽位抢控制权；收工后记得 start 回来）；
//   3) `echo $CYCLONEDDS_URI` 必须为空（交互式 shell 里 fishros 菜单选过 1
//      会导出绑定 eth0 的配置，本机是 eth10 ⇒ DDS 静默超时）；
//      ⚠️ 该菜单在 tmux 里**同样会弹**（tmux 默认起登录 shell → .profile → .bashrc，
//      而 `.bashrc:121-132` 那段没有 $TMUX/$PS1 守卫）⇒ 见到 `ros:foxy(1) noetic(2) ?`
//      按回车或 Ctrl+C 跳过，**一定不要按 1**；
//   4) FSM 用 loco.sh 切换，别和 PICO 遥操程序同时跑。
//
// 用法：见 printUsage()
// ============================================================================

#include "unitree/dds_wrapper/robots/g1/g1.h"    // g1::subscription::LowState
#include "unitree/dds_wrapper/robots/r1/r1.h"    // r1::publisher::ArmSdk
#include "unitree/dds_wrapper/common/crc.h"      // crc32_core
#include "unitree/robot/channel/channel_factory.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>

namespace {

// ---------------------------------------------------------------- 全局 ----
std::atomic<bool> g_quit{false};

void onSignal(int) { g_quit.store(true); }

constexpr int kNumMotor = 35;
constexpr int kNumArmJoint = 10;
constexpr int kNumHold = 4;  // 腰 2 + 头 2
constexpr double kPi = 3.14159265358979323846;

inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline double deg2rad(double d) { return d * kPi / 180.0; }

// ------------------------------------------------------------ 关节表 ----
struct ArmJoint {
  const char* key;   // 命令行简写
  const char* name;  // URDF 关节名
  int slot;          // LowCmd_ / LowState_ 槽号
  double lo;         // 下限 (rad)
  double hi;         // 上限 (rad)
  double kp;
  double kd;
};

// 限位来自官方 A5 模型 submodules/xr_teleoperate/assets/r1/r1_a5.urdf；
// 增益与官方 r1_arm_sdk_dds_example.cpp 及我们的 r1_arm_controller.h 一致
// （肩 pitch/roll 50/2，肩 yaw 与肘 40/2，腕 30/2）。
const std::array<ArmJoint, kNumArmJoint> kArmJoints = {{
    {"l0", "left_shoulder_pitch",  15, -3.14160, 2.09440, 50.0, 2.0},
    {"l1", "left_shoulder_roll",   16, -0.22689, 2.47840, 50.0, 2.0},
    {"l2", "left_shoulder_yaw",    17, -1.91990, 1.91990, 40.0, 2.0},
    {"l3", "left_elbow",           18, -0.97564, 2.18520, 40.0, 2.0},
    {"l4", "left_wrist_roll",      19, -1.91990, 1.91990, 30.0, 2.0},
    {"r0", "right_shoulder_pitch", 22, -3.14160, 2.09440, 50.0, 2.0},
    {"r1", "right_shoulder_roll",  23, -2.47849, 0.22680, 50.0, 2.0},
    {"r2", "right_shoulder_yaw",   24, -1.91990, 1.91990, 40.0, 2.0},
    {"r3", "right_elbow",          25, -0.97564, 2.18520, 40.0, 2.0},
    {"r4", "right_wrist_roll",     26, -1.91990, 1.91990, 30.0, 2.0},
}};

// 头 / 腰槽位：只做「就地保持」，本程序不主动改变（避免与出厂头服务抢关节）
const std::array<int, 2> kWaistSlots = {12, 13};
const std::array<int, 2> kHeadSlots = {29, 30};
constexpr double kWaistKp = 50.0, kWaistKd = 3.0;
constexpr double kHeadKp = 15.0, kHeadKd = 1.0;

// 指令领先实测的最大允许量（rad，≈20°）。仅作兜底：电机被物理挡住时，
// 指令积分器会一直往前跑，若无此上限，一旦卡滞解除就会暴冲。
// 正常工况下指令只领先实测"够克服静摩擦的那一点点"，远远到不了这个值。
constexpr double kCmdLagMax = 0.35;

inline int holdSlot(int i) { return i < 2 ? kWaistSlots[i] : kHeadSlots[i - 2]; }

// dry-run 模式的初始"实测"角（只为让输出有区分度）
const std::array<double, kNumArmJoint> kSimSeed = {
    0.30, 0.25, 0.00, 0.40, 0.00,
    0.30, -0.25, 0.00, 0.40, 0.00};

// --------------------------------------------------------------- 工具 ----
std::string trim(const std::string& s) {
  const std::string ws = " \t\r\n";
  const size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos) return std::string();
  const size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : s) {
    if (ch == ' ' || ch == '\t' || ch == ',') {
      if (!cur.empty()) { out.push_back(cur); cur.clear(); }
    } else {
      cur.push_back(ch);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

std::string lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

bool toNum(const std::string& s, double& v) {
  if (s.empty()) return false;
  char* end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return end != nullptr && *end == '\0';
}

int toJoint(const std::string& tok) {
  const std::string t = lower(trim(tok));
  if (t.empty()) return -1;
  if (t.size() == 1 && t[0] >= '0' && t[0] <= '9') {  // 0..9 => 0-4 左, 5-9 右
    return t[0] - '0';
  }
  for (int i = 0; i < kNumArmJoint; ++i) {
    if (t == kArmJoints[i].key || t == kArmJoints[i].name) return i;
  }
  return -1;
}

// ----------------------------------------------------- 带 CRC 的发布器 ----
// rt/arm_sdk 本身不校验 CRC，这里与 r1_arm_controller.h 保持一致（无害且同源）。
class CrcArmSdkPublisher : public unitree::robot::r1::publisher::ArmSdk {
 public:
  using unitree::robot::r1::publisher::ArmSdk::ArmSdk;

 protected:
  void pre_communication() override {
    msg_.crc(crc32_core(reinterpret_cast<uint32_t*>(&msg_), (sizeof(msg_) >> 2) - 1));
  }
};

// ------------------------------------------------------------ 主工具 ----
class ManualArmTool {
 public:
  struct Config {
    bool dry_run = false;
    bool auto_take = false;
    bool fault_check = true;  // motorstate != 0 时自动释放（官方要求；--no-fault-check 可关）
    double rate_hz = 250.0;
    double vel_limit = 2.0;  // rad/s
  };

  explicit ManualArmTool(const Config& cfg) : cfg_(cfg) {
    arm_target_.fill(0.0);
    hold_target_.fill(0.0);
    sim_.fill(0.0);
    sim_mstate_.fill(0);
  }

  ~ManualArmTool() { shutdown(); }

  // ---------------------------------------------------------- 生命周期 --
  void start() {
    printSafetyBanner();

    if (cfg_.dry_run) {
      std::cout << "[dry-run] 不连接 DDS、不下发任何指令，仅本地模拟。\n" << std::endl;
      for (int i = 0; i < kNumArmJoint; ++i) sim_[kArmJoints[i].slot] = kSimSeed[i];
      // 让未接管的头/腰也有一点非零值
      for (int i = 0; i < kNumHold; ++i) sim_[holdSlot(i)] = 0.0;
    } else {
      std::cout << "[init] 订阅 rt/lowstate ...（阻塞等待，检查网卡名与 CYCLONEDDS_URI）"
                << std::endl;
      lowstate_ = std::make_shared<unitree::robot::g1::subscription::LowState>();
      lowstate_->wait_for_connection();
      std::cout << "[init] lowstate 已连接。" << std::endl;

      publisher_ = std::make_unique<CrcArmSdkPublisher>("rt/arm_sdk");
      std::cout << "[init] 发布 rt/arm_sdk（接管权重由 mode_pr 承载）" << std::endl;
      seedCommand();
    }

    syncTargetsToFeedback();

    run_.store(true);
    thread_ = std::thread(&ManualArmTool::publishLoop, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    printTable();
    std::cout << "提示：确认姿态与臂展空间无误后，输入 t 接管；输入 ? 看全部命令。\n"
              << std::endl;
    if (cfg_.auto_take) take();
  }

  void shutdown() {
    if (shutdown_done_.exchange(true)) return;
    if (weight_.load() > 0) {
      std::cout << "[exit] 交还控制（权重 → 0，1 s 线性）..." << std::endl;
      // 必须 abortable=false：SIGINT/SIGTERM/SIGHUP 路径下 g_quit 已经为真
      rampWeight(weight_.load(), 0, 1.0, /*abortable=*/false);
    }
    run_.store(false);
    if (thread_.joinable()) thread_.join();
  }

  // -------------------------------------------------------- 交互命令分发 --
  // 返回 false 表示请求退出
  bool handleLine(const std::string& raw) {
    std::string line = trim(raw);
    if (line.empty()) {
      if (last_.empty()) return true;
      std::cout << "(重复) " << last_ << std::endl;
      line = last_;
    } else {
      last_ = line;
    }

    const std::vector<std::string> t = split(line);
    if (t.empty()) return true;
    const std::string c = lower(t[0]);

    if (c == "q" || c == "quit" || c == "exit") return false;
    if (c == "?" || c == "help") { printHelp(); return true; }
    if (c == "l" || c == "ls" || c == "list") { printTable(); return true; }
    if (c == "p" || c == "status") { printStatus(); return true; }
    if (c == "t" || c == "take") { take(); return true; }
    if (c == "r" || c == "release") { release(); return true; }
    if (c == "c" || c == "sync") { syncTargetsToFeedback(); std::cout << "  目标 ← 当前实测（偏差归零）。\n"; return true; }
    if (c == "h" || c == "home") { cmdHome(); return true; }

    if (c == "s" || c == "set") {
      if (t.size() < 3) { std::cout << "  用法: s <关节> <角度(deg)>\n"; return true; }
      double deg = 0.0;
      const int idx = toJoint(t[1]);
      if (idx < 0) { std::cout << "  认不出关节: " << t[1] << "（用 l0..l4 / r0..r4 / 0..9）\n"; return true; }
      if (!toNum(t[2], deg)) { std::cout << "  角度不是数字: " << t[2] << "\n"; return true; }
      setJointDeg(idx, deg);
      return true;
    }

    if (c == "+" || c == "-") {
      if (t.size() < 2) { std::cout << "  用法: " << c << " <关节> [增量deg，默认5]\n"; return true; }
      const int idx = toJoint(t[1]);
      if (idx < 0) { std::cout << "  认不出关节: " << t[1] << "\n"; return true; }
      double d = 5.0;
      if (t.size() >= 3 && !toNum(t[2], d)) { std::cout << "  增量不是数字: " << t[2] << "\n"; return true; }
      nudgeDeg(idx, (c == "+") ? d : -d);
      return true;
    }

    if (c == "m" || c == "arm") {
      if (t.size() < 7) {
        std::cout << "  用法: m <L|R> <d0> <d1> <d2> <d3> <d4>   （5 个角度，单位 deg）\n";
        return true;
      }
      const std::string side = lower(t[1]);
      int base = -1;
      if (side == "l" || side == "left") base = 0;
      else if (side == "r" || side == "right") base = 5;
      if (base < 0) { std::cout << "  侧别只能是 L 或 R\n"; return true; }
      for (int i = 0; i < 5; ++i) {
        double d = 0.0;
        if (!toNum(t[2 + i], d)) { std::cout << "  角度不是数字: " << t[2 + i] << "\n"; return true; }
        setJointDeg(base + i, d);
      }
      return true;
    }

    if (c == "a" || c == "all") {
      if (t.size() < 2) { std::cout << "  用法: a <角度(deg)>\n"; return true; }
      double d = 0.0;
      if (!toNum(t[1], d)) { std::cout << "  角度不是数字: " << t[1] << "\n"; return true; }
      for (int i = 0; i < kNumArmJoint; ++i) setJointDeg(i, d);
      return true;
    }

    if (c == "f" || c == "fault") {
      if (!cfg_.dry_run) {
        std::cout << "  `f` 只在 --dry-run 下可用（往模拟状态里注入电机故障）。\n";
        return true;
      }
      if (t.size() < 2) { std::cout << "  用法: f <关节> [0|1]  （默认 1 = 注入故障）\n"; return true; }
      const int idx = toJoint(t[1]);
      if (idx < 0) { std::cout << "  认不出关节: " << t[1] << "\n"; return true; }
      uint32_t v = 1;
      if (t.size() >= 3) {
        double n = 1;
        if (!toNum(t[2], n)) { std::cout << "  不是数字: " << t[2] << "\n"; return true; }
        v = (n == 0) ? 0u : 1u;
      }
      {
        std::lock_guard<std::mutex> lk(sim_mtx_);
        sim_mstate_[kArmJoints[idx].slot] = v;
      }
      std::printf("  [dry-run] 模拟 %s（槽位 %d）motorstate = %u\n",
                  kArmJoints[idx].name, kArmJoints[idx].slot, v);
      return true;
    }

    if (c == "w" || c == "weight") {
      if (t.size() < 2) { std::cout << "  用法: w <0..100>\n"; return true; }
      double v = 0.0;
      if (!toNum(t[1], v)) { std::cout << "  不是数字: " << t[1] << "\n"; return true; }
      const int w = static_cast<int>(std::lround(std::min(100.0, std::max(0.0, v))));
      syncTargetsToFeedback();
      weight_.store(w);
      std::cout << "  接管权重 = " << w << " / 100" << std::endl;
      return true;
    }

    std::cout << "  未知命令: " << t[0] << "（输入 ? 看帮助）\n";
    return true;
  }

  // ---------------------------------------------------------- 状态输出 --
  void printTable() const {
    std::array<double, kNumMotor> meas{};
    uint8_t mm = 0;
    snapshot(meas, mm);

    std::array<double, kNumArmJoint> tgt{};
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      tgt = arm_target_;
    }

    const int w = weight_.load();
    std::cout << "\n 接管权重 mode_pr = " << w << " / 100"
              << (w == 0 ? "   （未接管，双臂仍在机载运控手里）"
                         : "   （已接管）")
              << "   已发布 " << frames_.load() << " 帧\n";
    if (fault_latched_.load()) {
      std::printf(" ⚠ 电机故障闩锁中：槽位 %d (%s) motorstate = 0x%08X —— 已自动释放权重\n",
                  fault_slot_.load(), nameOfSlot(fault_slot_.load()), fault_code_.load());
    }
    std::cout << " idx  关节名                    槽   当前(deg)    目标(deg)        限位(deg)\n";
    std::cout << " ---------------------------------------------------------------------------\n";

    char buf[256];
    for (int i = 0; i < kNumArmJoint; ++i) {
      const ArmJoint& j = kArmJoints[i];
      std::snprintf(buf, sizeof(buf),
                    " %-4s %-24s %2d  %+8.2f     %+8.2f    [%+7.1f, %+7.1f]\n",
                    j.key, j.name, j.slot,
                    rad2deg(meas[j.slot]), rad2deg(tgt[i]),
                    rad2deg(j.lo), rad2deg(j.hi));
      std::cout << buf;
    }
    std::cout << " （腰部 13(Yaw) 与 头部 29/30 只做就地保持，本程序不主动改变；\n"
                 "   腰部 12(Roll) 由内置运控独占，写入不生效）\n" << std::endl;
  }

  void printStatus() const {
    std::array<double, kNumMotor> meas{};
    std::array<uint32_t, kNumMotor> ms{};
    uint8_t mm = 0;
    snapshot(meas, mm, &ms);

    std::array<double, kNumArmJoint> tgt{};
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      tgt = arm_target_;
    }

    double max_err = 0.0;
    int worst = 0;
    for (int i = 0; i < kNumArmJoint; ++i) {
      const double e = std::fabs(tgt[i] - meas[kArmJoints[i].slot]);
      if (e > max_err) { max_err = e; worst = i; }
    }

    std::cout << "\n--- 运行状态 ---\n"
              << "  接管权重 mode_pr : " << weight_.load() << " / 100\n"
              << "  发布频率         : " << cfg_.rate_hz << " Hz（已发布 " << frames_.load()
              << " 帧）\n"
              << "  速度限幅         : " << cfg_.vel_limit << " rad/s\n"
              << "  mode_machine     : " << static_cast<int>(mm) << "（逐帧从 lowstate 透传）\n"
              << "  最大剩余偏差     : " << rad2deg(max_err) << " deg @ "
              << kArmJoints[worst].name << "\n"
              << "  指令领先实测     : " << rad2deg(cmd_lag_max_.load()) << " deg"
              << (cmd_lag_max_.load() >= kCmdLagMax - 1e-6
                      ? "  ⚠ 已顶到上限 ⇒ 电机没跟上（死区过大/被卡住）\n"
                      : "（正常应在个位数）\n")
              << "  电机故障监控     : " << (cfg_.fault_check ? "开（motorstate != 0 自动释放）"
                                                            : "**已关闭**（--no-fault-check）")
              << "\n"
              << "  臂关节 motorstate: " << maxMotorStateText(ms) << "\n"
              << "  电机故障闩锁     : "
              << (fault_latched_.load()
                      ? (std::string("⚠ 有！槽位 ") + std::to_string(fault_slot_.load()))
                      : std::string("无"))
              << "\n"
              << "  模式             : " << (cfg_.dry_run ? "dry-run（无 DDS）" : "在线 rt/arm_sdk")
              << "\n" << std::endl;
  }

  static void printHelp() {
    std::cout <<
      "\n命令:\n"
      "  l, list                 关节表（当前角 / 目标角 / 限位）\n"
      "  t, take                 接管双臂（权重 0→100，1.5s 线性；接管前自动对齐当前位姿）\n"
      "  r, release              交还双臂给机载运控（权重 →0，1.5s 线性）\n"
      "  c, sync                 目标 ← 当前实测（把一切偏差归零，最安全）\n"
      "  h, home                 双臂目标回到零位\n"
      "  s <关节> <deg>          设某关节到指定角度（度）\n"
      "  m <L|R> d0 d1 d2 d3 d4  整条臂一次设定（5 个角度，度）\n"
      "  + <关节> [deg]          某关节加 N 度（默认 5）\n"
      "  - <关节> [deg]          某关节减 N 度（默认 5）\n"
      "  a <deg>                 全部 10 个臂关节设为该角度\n"
      "  w <0..100>              直接设接管权重\n"
      "  p, status               运行状态（权重/频率/mode_machine/最大剩余偏差/电机故障闩锁）\n"
      "  f <关节> [0|1]          仅 --dry-run：注入/清除模拟电机故障（默认 1）\n"
      "  ?, help                 本帮助\n"
      "  q, quit                 交还权重后退出\n"
      "  <空行>                  重复上一条命令（配合 + / - 连续微调很顺手）\n"
      "\n关节写法: l0..l4（左）、r0..r4（右）、0..9（0-4 左，5-9 右）、或全名。\n"
      "\n顺序（a5）: 0=肩pitch 1=肩roll 2=肩yaw 3=肘 4=腕roll（左右各一组）\n" << std::endl;
  }

  int weight() const { return weight_.load(); }

  bool faultLatched() const { return fault_latched_; }

 private:
  static const char* nameOfSlot(int slot) {
    for (int i = 0; i < kNumArmJoint; ++i) {
      if (kArmJoints[i].slot == slot) return kArmJoints[i].name;
    }
    for (int i = 0; i < kNumHold; ++i) {
      if (holdSlot(i) == slot) return "(头/腰)";
    }
    return "?";
  }

  static std::string maxMotorStateText(const std::array<uint32_t, kNumMotor>& ms) {
    std::string bad;
    for (int i = 0; i < kNumArmJoint; ++i) {
      const int s = kArmJoints[i].slot;
      if (ms[s] == 0) continue;
      if (!bad.empty()) bad += ", ";
      char buf[24];
      std::snprintf(buf, sizeof(buf), "%s=0x%X", kArmJoints[i].key, ms[s]);
      bad += buf;
    }
    return bad.empty() ? std::string("全 0（正常）") : bad;
  }

  // ------------------------------------------------------------ 反馈 --
  void snapshot(std::array<double, kNumMotor>& out, uint8_t& mode_machine,
                std::array<uint32_t, kNumMotor>* mstate = nullptr) const {
    if (cfg_.dry_run) {
      std::lock_guard<std::mutex> lk(sim_mtx_);
      out = sim_;
      mode_machine = 0;
      if (mstate != nullptr) *mstate = sim_mstate_;
      return;
    }
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    for (int s = 0; s < kNumMotor; ++s) {
      out[s] = lowstate_->msg_.motor_state().at(s).q();
      if (mstate != nullptr) (*mstate)[s] = lowstate_->msg_.motor_state().at(s).motorstate();
    }
    mode_machine = lowstate_->msg_.mode_machine();
  }

  /// 任一双臂关节报 motorstate != 0（官方要求：立刻停止下发并释放控制权）
  bool anyMotorFault(uint32_t* code, int* slot) const {
    std::array<double, kNumMotor> meas{};
    std::array<uint32_t, kNumMotor> ms{};
    uint8_t mm = 0;
    snapshot(meas, mm, &ms);
    for (int i = 0; i < kNumArmJoint; ++i) {
      const int s = kArmJoints[i].slot;
      if (ms[s] != 0) {
        if (code != nullptr) *code = ms[s];
        if (slot != nullptr) *slot = s;
        return true;
      }
    }
    return false;
  }

  // 目标 ← 当前实测（未接管时发布线程也在持续做这件事）
  void syncTargetsToFeedback() {
    std::array<double, kNumMotor> meas{};
    uint8_t mm = 0;
    snapshot(meas, mm);
    std::lock_guard<std::mutex> lk(tgt_mtx_);
    for (int i = 0; i < kNumArmJoint; ++i) arm_target_[i] = meas[kArmJoints[i].slot];
    for (int i = 0; i < kNumHold; ++i) hold_target_[i] = meas[holdSlot(i)];
    // 目标既然已归零到实测，指令轨迹也必须从实测重新起步，否则会带着旧的前瞻量冲过去
    cmd_reset_.store(true);
  }

  // ------------------------------------------------- 接管 / 交还 / 权重 --
  // abortable=false 专用于「交还 / 释放权重」路径：那时 g_quit 已经为真，
  // 若仍检查它会第一步就 break，把 1 s 线性降权变成硬切到 0（手臂突失力、
  // 也不会再持续发布指令）。发布线程在 ramp 期间继续按 250 Hz 发帧。
  void rampWeight(int from, int to, double dur, bool abortable = true) {
    const int steps = std::max(1, static_cast<int>(std::lround(dur * cfg_.rate_hz)));
    const auto period = std::chrono::duration<double>(1.0 / cfg_.rate_hz);
    for (int i = 1; i <= steps; ++i) {
      if (abortable && g_quit.load()) break;
      weight_.store(from + static_cast<int>(std::lround(double(to - from) * i / steps)));
      std::this_thread::sleep_for(period);
    }
    weight_.store(to);
  }

  void take() {
    if (weight_.load() >= 100) {
      std::cout << "  已经在接管状态（权重 100）。\n";
      return;
    }
    // 官方要求：motorstate != 0 时不得下发指令
    uint32_t code = 0;
    int slot = -1;
    if (anyMotorFault(&code, &slot)) {
      std::printf("  ⛔ 拒绝接管：关节槽位 %d 报 motorstate = 0x%08X。\n", slot, code);
      std::cout << "     先排查（`p` 看状态；必要时切 FSM 1 阻尼/重新上电），确认清零后再 t。\n";
      return;
    }
    if (fault_latched_) {
      fault_latched_ = false;
      std::cout << "  电机状态已恢复正常，清除故障标记。\n";
    }
    syncTargetsToFeedback();  // 接管前对齐，保证不跳
    std::cout << "  接管双臂：权重 → 100（1.5 s 线性）..." << std::endl;
    rampWeight(weight_.load(), 100, 1.5);
    std::cout << "  已接管。现在可以用 s / + / - 逐关节调；l 看关节表。\n";
  }

  void release() {
    if (weight_.load() == 0) {
      std::cout << "  当前权重已是 0（未接管）。\n";
      return;
    }
    std::cout << "  交还双臂：权重 → 0（1.5 s 线性）..." << std::endl;
    rampWeight(weight_.load(), 0, 1.5);
    std::cout << "  已交还，双臂回到机载运控控制。\n";
  }

  bool requireTaken() const {
    if (weight_.load() == 0) {
      std::cout << "  ⚠ 当前权重为 0（未接管），指令不会生效 —— 先输入 t 接管。\n";
      return false;
    }
    return true;
  }

  // -------------------------------------------------------- 目标修改 --
  void setJointDeg(int idx, double deg) {
    if (!requireTaken()) return;
    const ArmJoint& j = kArmJoints[idx];
    double rad = deg2rad(deg);
    if (rad < j.lo || rad > j.hi) {
      const double c = std::min(std::max(rad, j.lo), j.hi);
      std::printf("  ⚠ %s %.1f° 超出限位 [%.1f°, %.1f°] → 夹到 %.1f°\n",
                  j.name, deg, rad2deg(j.lo), rad2deg(j.hi), rad2deg(c));
      rad = c;
    }
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      arm_target_[idx] = rad;
    }
    std::printf("  %s ← %+.2f°\n", j.name, rad2deg(rad));
  }

  void nudgeDeg(int idx, double delta_deg) {
    if (!requireTaken()) return;
    double cur = 0.0;
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      cur = arm_target_[idx];
    }
    setJointDeg(idx, rad2deg(cur) + delta_deg);
  }

  void cmdHome() {
    if (!requireTaken()) return;
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      for (int i = 0; i < kNumArmJoint; ++i) arm_target_[i] = 0.0;
    }
    std::cout << "  双臂目标 ← 零位（按 --vel 限速平滑过去）。\n";
  }

  // ------------------------------------------------------------ 发布 --
  void seedCommand() {
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    const auto& motor = lowstate_->msg_.motor_state();
    publisher_->lock();
    publisher_->msg_.mode_machine(lowstate_->msg_.mode_machine());
    for (int i = 0; i < kNumArmJoint; ++i) {
      const ArmJoint& j = kArmJoints[i];
      auto& c = publisher_->msg_.motor_cmd().at(j.slot);
      c.mode(1);
      c.q(static_cast<float>(motor.at(j.slot).q()));
      c.dq(0.f);
      c.tau(0.f);
      c.kp(static_cast<float>(j.kp));
      c.kd(static_cast<float>(j.kd));
    }
    for (int i = 0; i < kNumHold; ++i) {
      const int s = holdSlot(i);
      auto& c = publisher_->msg_.motor_cmd().at(s);
      c.mode(1);
      c.q(static_cast<float>(motor.at(s).q()));
      c.dq(0.f);
      c.tau(0.f);
      if (i < 2) { c.kp(static_cast<float>(kWaistKp)); c.kd(static_cast<float>(kWaistKd)); }
      else { c.kp(static_cast<float>(kHeadKp)); c.kd(static_cast<float>(kHeadKd)); }
    }
    publisher_->unlock();  // 先不发布，交给发布线程统一下发
  }

  void publishLoop() {
    const auto period = std::chrono::duration<double>(1.0 / cfg_.rate_hz);
    while (run_.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      publishOnce();
      const auto elapsed = std::chrono::steady_clock::now() - t0;
      const auto remain = period - elapsed;
      if (remain > std::chrono::duration<double>(0)) std::this_thread::sleep_for(remain);
    }
  }

  void publishOnce() {
    const double dt = 1.0 / cfg_.rate_hz;

    std::array<double, kNumMotor> meas{};
    std::array<uint32_t, kNumMotor> ms{};
    uint8_t mode_machine = 0;
    snapshot(meas, mode_machine, &ms);

    // 电机故障监控。官方要求：motorstate != 0 时立即停止下发并释放控制权。
    if (cfg_.fault_check && !fault_latched_) {
      for (int i = 0; i < kNumArmJoint; ++i) {
        const int s = kArmJoints[i].slot;
        if (ms[s] != 0) {
          fault_latched_ = true;
          fault_slot_ = s;
          fault_code_ = ms[s];
          break;
        }
      }
      if (fault_latched_) {
        const int fslot = fault_slot_.load();
        const uint32_t fcode = fault_code_.load();
        std::printf("\n  ⚠⚠ 电机故障：%s（槽位 %d）motorstate = 0x%08X → 立即释放控制权\n",
                    nameOfSlot(fslot), fslot, fcode);
        std::cout << "     权重将在约 0.5 s 内降到 0；排查后再输 t 重新接管。\n" << std::endl;
      }
    }
    if (fault_latched_ && weight_.load() > 0) {
      // 逐帧递减，约 0.5 s 降到 0（不能硬切到 0，否则手臂会突然失力）
      const int step =
          std::max(1, static_cast<int>(std::lround(100.0 / (0.5 * cfg_.rate_hz))));
      weight_.store(std::max(0, weight_.load() - step));
    }

    const int w = weight_.load();

    // 未接管时目标持续跟随实测 —— 保证接管瞬间零跳变
    if (w == 0) {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      for (int i = 0; i < kNumArmJoint; ++i) arm_target_[i] = meas[kArmJoints[i].slot];
      for (int i = 0; i < kNumHold; ++i) hold_target_[i] = meas[holdSlot(i)];
    }

    std::array<double, kNumArmJoint> tgt{};
    std::array<double, kNumHold> hold{};
    {
      std::lock_guard<std::mutex> lk(tgt_mtx_);
      tgt = arm_target_;
      hold = hold_target_;
    }

    // ---- 速度限幅：指令轨迹积分（**不是**基于实测的比例推进）-------------------
    // ⚠️ 2026-09-23 真机缺陷与修复（务必别改回去）：
    //   原写法 cmd = 实测 + (目标-实测)*scale（scale = vel_limit*dt/maxd），即每帧最多比
    //   "实测"超前 vel_limit*dt。当这个超前量产生的力矩小于静摩擦/伺服死区时，实测不动
    //   ⇒ 下一帧又从"没动的实测"重新算 ⇒ 指令永远推不动电机。真机表现：权重 100、
    //   目标列在变、手臂纹丝不动。--vel 2 时超前量只有 0.46°(≈0.4 N·m) 必被吃掉；
    //   --vel 30（6.9°、≈6 N·m）则直接能动 —— 这正是 09-23 现场实测到的分界。
    //   （xr_teleoperate 与 r1_arm_controller.h::clipTargets 是同一写法，它们因为默认
    //     arm_vel_limit=30 而侥幸能用，同样的坑在低限速下会复现。）
    //   正确做法（与官方 r1_arm_sdk_dds_example::movej 的绝对轨迹同源）：让指令自己按
    //   速度限幅向前积分，超前量持续累积直到克服死区；kCmdLagMax 兜底，防止电机被
    //   物理卡住时指令跑到远处、一旦解除就暴冲。
    const double allowed = cfg_.vel_limit * dt;
    const bool reset_flag = cmd_reset_.exchange(false);
    const bool reset = (w == 0) || reset_flag;
    std::array<double, kNumArmJoint> cmd{};
    double lagmax = 0.0;
    for (int i = 0; i < kNumArmJoint; ++i) {
      const int s = kArmJoints[i].slot;
      if (reset) cmd_prev_[i] = meas[s];  // 未接管/刚复位 ⇒ 从实测起步，接管零跳变
      const double step = std::clamp(tgt[i] - cmd_prev_[i], -allowed, allowed);
      double c = cmd_prev_[i] + step;
      const double lag = c - meas[s];
      if (std::fabs(lag) > kCmdLagMax) c = meas[s] + std::copysign(kCmdLagMax, lag);
      cmd[i] = c;
      cmd_prev_[i] = c;
      lagmax = std::max(lagmax, std::fabs(c - meas[s]));
    }
    cmd_lag_max_.store(lagmax);

    frames_.fetch_add(1);

    if (cfg_.dry_run) {
      // 本地一阶跟随，模拟真实关节的限速响应
      std::lock_guard<std::mutex> lk(sim_mtx_);
      for (int i = 0; i < kNumArmJoint; ++i) {
        const int s = kArmJoints[i].slot;
        const double d = cmd[i] - sim_[s];
        sim_[s] += std::max(-allowed, std::min(allowed, d));
      }
      return;
    }

    publisher_->lock();
    publisher_->msg_.mode_machine(mode_machine);
    publisher_->msg_.mode_pr(static_cast<uint8_t>(w));
    for (int i = 0; i < kNumArmJoint; ++i) {
      auto& c = publisher_->msg_.motor_cmd().at(kArmJoints[i].slot);
      c.mode(1);
      c.q(static_cast<float>(cmd[i]));
      c.dq(0.f);
      c.tau(0.f);
    }
    for (int i = 0; i < kNumHold; ++i) {
      auto& c = publisher_->msg_.motor_cmd().at(holdSlot(i));
      c.mode(1);
      c.q(static_cast<float>(hold[i]));
      c.dq(0.f);
      c.tau(0.f);
    }
    publisher_->unlockAndPublish();
  }

  // ------------------------------------------------------------ 提示 --
  static void printSafetyBanner() {
    std::cout <<
      "\n================= R1 双臂手动关节调试 =================\n"
      " 通道: rt/arm_sdk，mode_pr = 接管权重(0..100)；下肢由机载 ai_sport 照常控制\n"
      " 臂关节: 左 15-19 / 右 22-26（a5，每臂 5 自由度）\n"
      "\n 运行前四项检查（顺序别换）:\n"
      "   1) 机器人已吊挂或有可靠支撑，臂展范围内无人无物\n"
      "   2) sudo systemctl stop r1-custom-head-remote   ← 否则与头/腰槽位抢控制权\n"
      "   3) echo $CYCLONEDDS_URI 必须为空（非空就 env -u CYCLONEDDS_URI 再跑）\n"
      "      （tmux 里若出现 `ros:foxy(1) noetic(2) ?`：按回车或 Ctrl+C 跳过，**千万别按 1**\n"
      "         —— 按 1 会导出绑定 eth0 的配置，本机是 eth10 ⇒ DDS 静默超时）\n"
      "   4) FSM 用 loco.sh 切（第1步=4 锁定站立 / 第2步=811 走跑运控），本程序不切 FSM\n"
      "\n ⚠ 退出请用 q 或 Ctrl+C（会自动把权重降到 0 交还）。**绝不要 kill -9** ——\n"
      "   权重停在 100 且不再发布指令时，按官方说明**手臂会冻结在最后一帧姿态**，不会回落到运控。\n"
      " ⚠ 本接口不做安全限位与碰撞检测，目标位置由本程序负责（已按 URDF 限位 + 速度限幅）。\n"
      "=======================================================\n" << std::endl;
  }

  // ------------------------------------------------------------ 成员 --
  Config cfg_;

  std::shared_ptr<unitree::robot::g1::subscription::LowState> lowstate_;
  std::unique_ptr<CrcArmSdkPublisher> publisher_;
  std::thread thread_;

  std::atomic<bool> run_{false};
  std::atomic<bool> shutdown_done_{false};
  std::atomic<int> weight_{0};  // 0..100，直接写进 mode_pr
  std::atomic<unsigned long long> frames_{0};

  // 电机故障闩锁（只有发布线程写，REPL 线程读；用 atomic 免锁）
  std::atomic<bool> fault_latched_{false};
  std::atomic<int> fault_slot_{-1};
  std::atomic<uint32_t> fault_code_{0};

  mutable std::mutex tgt_mtx_;
  std::array<double, kNumArmJoint> arm_target_{};
  std::array<double, kNumHold> hold_target_{};

  // 指令轨迹积分器：cmd_prev_ 只有发布线程访问；cmd_reset_ 由 REPL 线程置位
  std::array<double, kNumArmJoint> cmd_prev_{};
  std::atomic<bool> cmd_reset_{true};
  // 最近一帧"指令 - 实测"的最大绝对值。它就是真正加在电机上的偏差：
  //   ≈0 且不动       → 指令没推进（限幅太小或目标没变）
  //   一直贴着上限    → 电机被卡住/死区过大（指令在推但实测不跟）
  //   正常应在小幅波动
  std::atomic<double> cmd_lag_max_{0.0};

  // dry-run 用
  mutable std::mutex sim_mtx_;
  std::array<double, kNumMotor> sim_{};
  std::array<uint32_t, kNumMotor> sim_mstate_{};  // dry-run 故障注入

  std::string last_;
};

// ---------------------------------------------------------------- 用法 ----
void printUsage(const char* prog) {
  std::cout <<
    "用法: " << prog << " <网卡> [选项]        （背包上网卡是 eth10）\n"
    "      " << prog << " --dry-run [选项]      （离线自测，不连 DDS）\n"
    "\n选项:\n"
    "  --dry-run        不连接 DDS、不下发任何指令，仅本地模拟\n"
    "  --auto-take      启动后直接接管（默认需手动输入 t）\n"
    "  --no-fault-check 关闭 motorstate 故障监控（⚠ 只有确认该字段语义与本机不同才用）\n"
    "  --rate <hz>      rt/arm_sdk 发布频率（默认 250；官方建议 100，可写 --rate 100）\n"
    "  --vel <rad/s>    关节速度限幅（默认 2.0）。⚠ 限幅是【指令轨迹】的速度上限，\n"
    "                   不是【能推动电机的下限】；太小则每帧给电机的偏差力矩可能低于\n"
    "                   静摩擦死区（09-23 实测：--vel 2 ⇒ 0.46°/帧 ⇒ 完全不动；30 ⇒ 能动）。\n"
    "                   已改为指令积分，2.0 现在可用；仍不动就加大到 10~30。\n"
    "  -h, --help       显示本帮助\n"
    "\n交互命令（靠 stdin 收键 ⇒ 必须在带 TTY 的交互式终端里跑；\n"
    "  推荐 tmux：会话可脱离/重连，SSH 断了程序也不会被一起带走）:\n"
    "  l 关节表   t 接管   r 交还   c 目标←当前   h 回零位   p 状态\n"
    "  s <关节> <deg> | m <L|R> d0..d4 | + <关节> [deg] | - <关节> [deg]\n"
    "  a <deg> | w <0..100> | ? 帮助 | q 交还并退出 | 空行 = 重复上一条\n"
    "  f <关节> [0|1]   仅 --dry-run：注入/清除模拟电机故障\n"
    "\n示例:\n"
    "  " << prog << " eth10 --vel 2\n"
    "  " << prog << " --dry-run          # 本机就能跑，验证命令与限位逻辑\n"
    "\n⚠ 本程序只接管双臂。跑之前先停 r1-custom-head-remote、确认 $CYCLONEDDS_URI 为空、\n"
    "  机器人有支撑，并按第 1 步(FSM 4 锁定站立)/第 2 步(FSM 811 走跑运控)推进。\n"
    "⚠ 退出用 q 或 Ctrl+C（会自动降权交还）；**绝不要 kill -9** —— 权重停在 100 且不再\n"
    "  发布时，手臂会冻结在最后一帧姿态（官方说明），不会自动回落给内置运控。\n"
    "  SSH 掉线/关窗口（SIGHUP）已接住，同样会走 1 s 线性交还。\n";
}

}  // namespace

int main(int argc, char** argv) {
  ManualArmTool::Config cfg;
  std::string iface;
  bool want_help = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--dry-run") cfg.dry_run = true;
    else if (a == "--auto-take") cfg.auto_take = true;
    else if (a == "--no-fault-check") cfg.fault_check = false;
    else if (a == "--rate" && i + 1 < argc) cfg.rate_hz = std::atof(argv[++i]);
    else if (a == "--vel" && i + 1 < argc) cfg.vel_limit = std::atof(argv[++i]);
    else if (a == "-h" || a == "--help") want_help = true;
    else if (!a.empty() && a[0] == '-') {
      std::cerr << "[main] 未知选项: " << a << std::endl;
      return 1;
    } else if (iface.empty()) {
      iface = a;
    }
  }

  if (want_help) { printUsage(argv[0]); return 0; }

  if (cfg.rate_hz <= 0.0 || cfg.vel_limit <= 0.0) {
    std::cerr << "[main] --rate / --vel 必须为正数" << std::endl;
    return 1;
  }

  if (!cfg.dry_run) {
    if (iface.empty()) {
      std::cerr << "[main] 缺少网卡名。背包上一般是 eth10。" << std::endl;
      printUsage(argv[0]);
      return 1;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  // SSH 掉线 / 终端窗口被关时，内核给前台进程组发的是 SIGHUP；不接住的话
  // 默认动作会立刻终止进程 ⇒ 权重停在 100 且不再发布 ⇒ 手臂冻结在最后一帧
  // 姿态（官方明确警告的失效模式）。接住它，走与 Ctrl+C 相同的交还路径。
  std::signal(SIGHUP, onSignal);

  ManualArmTool tool(cfg);
  tool.start();

  struct pollfd pfd{};
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN;

  std::string line;
  while (!g_quit.load()) {
    const int r = ::poll(&pfd, 1, 100);
    if (r < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (r == 0) continue;
    if (!std::getline(std::cin, line)) break;  // EOF（管道输入结束 / Ctrl+D）
    if (!tool.handleLine(line)) break;
  }

  if (g_quit.load()) std::cout << "\n收到中断信号，正在安全退出..." << std::endl;
  tool.shutdown();
  std::cout << "退出。" << std::endl;
  return 0;
}
