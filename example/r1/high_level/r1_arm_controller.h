#pragma once
// ============================================================================
// r1_arm_controller.h
// R1 双臂控制器（高层 rt/arm_sdk 覆盖模式；可选 rt/lowcmd 全关节模式）。
//
// 移植自 xr_teleoperate：teleop/robot_control/robot_arm.py 的 R1_A5_ArmController
// 关键行为（与原版对齐）：
//   1) 订阅 rt/lowstate（unitree::robot::g1::subscription::LowState），
//      以当前关节位置给所有受控槽位做种子值（mode=1 使能 + kp/kd）。
//   2) 启动时把头部(29/30)与腰部(12/13)线性回零（3s，逐帧发布）。
//   3) 独立发布线程按 250 Hz 发送 LowCmd_：
//        - motion_mode(rt/arm_sdk)：只写双臂槽位（A5:15-19/22-26, A7:15-21/22-28），
//          头部/腰部保持回零后的目标；下肢由机载 ai_sport 继续控制。
//          mode_pr 作为接管权重（0..100），100 = 双臂完全交给 SDK。
//        - 否则(rt/lowcmd)：35 槽全量写（腿 200/3、踝弱 50/2、臂 50/40/30、头 15），
//          使用前必须通过 b2::MotionSwitcherClient 释放机载运动服务
//          （见 example/r1/low_level/r1A_wrist_swing_example.cpp）。
//   4) 每帧对目标做关节速度限幅：|d(q_target)|/dt <= arm_vel_limit（默认 30 rad/s，
//      与 xr_teleoperate set_arm_velocity_limit 默认值一致）。
//   5) releaseArmSdk()：把权重从当前值线性降到 0（默认 2s），平滑交还 ai_sport。
//
// 关节槽位（LowCmd_ 35 槽，与 xr_teleoperate R1_A5/A7_JointIndex 一致）：
//   腿 0-11，腰 12(未用/腰滚) 13(腰偏航)，14 未用，
//   左臂 15-19(A5) / 15-21(A7)，右臂 22-26(A5) / 22-28(A7)，头 29/30，31-34 未用。
// ============================================================================

#include "unitree/dds_wrapper/robots/r1/r1.h"          // r1::publisher::ArmSdk
#include "unitree/dds_wrapper/robots/g1/g1.h"          // g1::subscription::LowState
#include "unitree/dds_wrapper/common/crc.h"            // crc32_core
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace r1skeleton {

// ---------------------------------------------------------------------------
// 发布器：在 ArmSdk(RealTimePublisher) 基础上，于每次真正发送前写入 CRC。
// （官方 Python 侧每次 Write 前都算 CRC；C++ 侧用 pre_communication 钩子补齐。）
// ---------------------------------------------------------------------------
class R1ArmSdkPublisher : public unitree::robot::r1::publisher::ArmSdk {
 public:
  using unitree::robot::r1::publisher::ArmSdk::ArmSdk;

 protected:
  void pre_communication() override {
    msg_.crc(crc32_core(reinterpret_cast<uint32_t*>(&msg_), (sizeof(msg_) >> 2) - 1));
  }
};

// ---------------------------------------------------------------------------
// R1 双臂控制器
// ---------------------------------------------------------------------------
class R1ArmController {
 public:
  enum class Variant { R1_A5, R1_A7 };

  struct Params {
    bool motion_mode = true;       // true : 发布到 rt/arm_sdk（行走时并行接管双臂，推荐）
                                   // false: 发布到 rt/lowcmd（全关节，需释放机载运动服务）
    double publish_hz = 250.0;     // 官方 250 Hz
    double arm_vel_limit = 30.0;   // rad/s，官方 set_arm_velocity_limit 默认值
    // 增益（与 xr_teleoperate 当前 head 提交一致）
    double kp_shoulder = 50.0, kd_shoulder = 2.0;   // 肩 pitch/roll
    double kp_elbow    = 40.0, kd_elbow    = 2.0;   // 肩 yaw / 肘
    double kp_wrist    = 30.0, kd_wrist    = 2.0;   // 腕
    double kp_head     = 15.0, kd_head     = 1.0;   // 头
    double kp_weak     = 50.0, kd_weak     = 2.0;   // 踝 pitch（lowcmd 模式）
    double kp_high     = 200.0, kd_high    = 3.0;   // 腿（lowcmd 模式）
    double waist_kp    = 50.0, waist_kd    = 3.0;   // 腰（lowcmd/arm_sdk 保持）
  };

  explicit R1ArmController(Variant variant, const Params& params)
      : variant_(variant), params_(params) {
    if (variant_ == Variant::R1_A7) {
      // xr_teleoperate 明确 R1_A7 暂不支持 motion_mode(rt/arm_sdk)。
      // 仍保留该路径，接入前请确认你的 R1_A7 固件是否已开放 arm_sdk 覆盖。
      std::cout << "[arm] R1_A7: rt/arm_sdk(motion_mode) 支持取决于固件版本，"
                << "xr_teleoperate 中默认走 rt/lowcmd 全关节。" << std::endl;
    }
  }

  // 便捷重载：不能写成 `const Params& params = {}`，因为 Params 是嵌套类，
  // 其默认成员初始化器在封闭类未完成时不可用（GCC/Clang 均报错）。
  explicit R1ArmController(Variant variant) : R1ArmController(variant, Params{}) {}

  ~R1ArmController() { stop(); }

  // ---- 生命周期 ----------------------------------------------------------
  void start() {
    if (started_.exchange(true)) return;

    // 1) 订阅低层状态
    lowstate_ = std::make_shared<unitree::robot::g1::subscription::LowState>();
    lowstate_->wait_for_connection();
    std::cout << "[arm] lowstate connected." << std::endl;

    // 2) 发布器（必须在 ChannelFactory Init 之后构造）
    const std::string topic = params_.motion_mode ? "rt/arm_sdk" : "rt/lowcmd";
    publisher_ = std::make_unique<R1ArmSdkPublisher>(topic);
    std::cout << "[arm] publishing to " << topic << " @ " << params_.publish_hz << " Hz" << std::endl;

    // 3) 种子命令
    seedCommands();

    // 4) 臂 SDK 接管权重（必须在回零之前设置）。
    //    arm_sdk 模式下 mode_pr 是接管权重，权重为 0 时头/腰指令会被 ai_sport 忽略；
    //    若先回零再设权重，权重生效瞬间头/腰会从当前位直接跳到 0（且不被限幅）。
    //    对齐官方 R1_A5_ArmController：init 时就 mode_pr=100，之后才 go_home。
    setWeight(params_.motion_mode ? 1.0 : 0.0);

    // 5) 头部/腰部回零（阻塞 ~3s，逐帧发布；此时权重已生效）
    homeHeadAndWaist(3.0);

    // 6) 目标默认值 = 当前关节角（避免发布线程拿到空向量）
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      q_target_ = currentArmQ();
      tau_ff_ = Eigen::VectorXd::Zero(armDof());
    }

    // 7) 启动 250Hz 发布线程
    run_.store(true);
    publish_thread_ = std::thread(&R1ArmController::publishLoop, this);
    ready_.store(true);
    std::cout << "[arm] controller ready." << std::endl;
  }

  void stop() {
    if (!run_.exchange(false)) return;
    if (publish_thread_.joinable()) publish_thread_.join();
  }

  bool ready() const { return ready_.load(); }

  // ---- 数据接口 ----------------------------------------------------------
  int armDof() const { return (variant_ == Variant::R1_A7) ? 14 : 10; }
  int dofPerArm() const { return armDof() / 2; }

  /// 当前双臂关节角 [左 n, 右 n]（单位 rad，顺序 = 链顺序，与 IK 一致）。
  Eigen::VectorXd currentArmQ() const {
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    Eigen::VectorXd q(armDof());
    for (int i = 0; i < armDof(); ++i) {
      q[i] = lowstate_->msg_.motor_state().at(armSlot(i)).q();
    }
    return q;
  }

  Eigen::VectorXd currentArmDq() const {
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    Eigen::VectorXd dq(armDof());
    for (int i = 0; i < armDof(); ++i) {
      dq[i] = lowstate_->msg_.motor_state().at(armSlot(i)).dq();
    }
    return dq;
  }

  /// 设目标：q 与 tau_ff 长度均为 armDof()（[左 n, 右 n]）。
  void setTargets(const Eigen::VectorXd& q, const Eigen::VectorXd& tau_ff) {
    if (q.size() != armDof() || (tau_ff.size() != armDof() && tau_ff.size() != 0)) {
      std::cerr << "[arm] setTargets size mismatch: q=" << q.size()
                << " tau=" << tau_ff.size() << " want=" << armDof() << std::endl;
      return;
    }
    std::lock_guard<std::mutex> lock(target_mutex_);
    q_target_ = q;
    tau_ff_ = tau_ff.size() == armDof() ? tau_ff : Eigen::VectorXd::Zero(armDof());
  }

  /// 平滑释放 arm_sdk（阻塞）。仅 motion_mode 有意义。
  void releaseArmSdk(double duration_sec = 2.0) {
    if (!params_.motion_mode) return;
    if (!publisher_) return;
    const double dt = 1.0 / params_.publish_hz;
    const int steps = std::max(1, static_cast<int>(duration_sec / dt));
    const double w0 = weight();
    for (int i = 0; i <= steps; ++i) {
      const double w = w0 * (1.0 - static_cast<double>(i) / steps);
      setWeightAndPublish(w);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    setWeightAndPublish(0.0);
    std::cout << "[arm] arm_sdk released (weight -> 0)." << std::endl;
  }

  /// 双臂回零 + 释放：target=0 直到 |q|<tol（最多 max_attempts*5s），然后释放。
  void goHomeAndRelease(double tol = 0.05, int max_attempts = 100,
                        double release_dur = 2.0) {
    std::cout << "[arm] going home..." << std::endl;
    setTargets(Eigen::VectorXd::Zero(armDof()), Eigen::VectorXd::Zero(armDof()));
    for (int i = 0; i < max_attempts; ++i) {
      Eigen::VectorXd q = currentArmQ();
      if (q.cwiseAbs().maxCoeff() < tol) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    releaseArmSdk(release_dur);
  }

  // ---- 权重 --------------------------------------------------------------
  double weight() const { return weight_.load(); }

  void setWeight(double w) {
    const double c = std::clamp(w, 0.0, 1.0);
    weight_.store(c);
    publisher_->lock();
    publisher_->msg_.mode_pr(static_cast<uint8_t>(std::lround(c * 100.0)));
    publisher_->unlock();
  }

 private:
  // ---- 关节槽位表 --------------------------------------------------------
  int armSlot(int i) const {
    // i: 0..n-1 左臂，n..2n-1 右臂（n = dofPerArm）
    const bool a7 = (variant_ == Variant::R1_A7);
    const int n = dofPerArm();
    static constexpr std::array<int, 5> left_a5 = {15, 16, 17, 18, 19};
    static constexpr std::array<int, 7> left_a7 = {15, 16, 17, 18, 19, 20, 21};
    static constexpr std::array<int, 5> right_a5 = {22, 23, 24, 25, 26};
    static constexpr std::array<int, 7> right_a7 = {22, 23, 24, 25, 26, 27, 28};
    if (i < n) return a7 ? left_a7[i] : left_a5[i];
    return a7 ? right_a7[i - n] : right_a5[i - n];
  }

  static constexpr std::array<int, 2> kHeadSlots = {29, 30};
  static constexpr std::array<int, 2> kWaistSlots = {12, 13};

  // 槽位分类（用于增益选择，移植 _Is_weak/_Is_medium/_Is_wrist）
  bool isMediumArm(int slot) const {
    static const std::array<int, 4> m = {17, 18, 24, 25};  // 肩yaw、肘（左右）
    return std::find(m.begin(), m.end(), slot) != m.end();
  }
  bool isWrist(int slot) const {
    const bool a7 = (variant_ == Variant::R1_A7);
    if (a7) {
      static const std::array<int, 6> w = {19, 20, 21, 26, 27, 28};
      return std::find(w.begin(), w.end(), slot) != w.end();
    }
    static const std::array<int, 2> w = {19, 26};
    return std::find(w.begin(), w.end(), slot) != w.end();
  }
  bool isWeakLow(int slot) const { return slot == 4 || slot == 10; }  // 踝 pitch

  // ---- 初始化 ------------------------------------------------------------
  void seedCommands() {
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    const auto& motor = lowstate_->msg_.motor_state();
    publisher_->lock();  // 修改 msg_ 前取得发布器锁（RealTimePublisher 内部线程会读 msg_）
    publisher_->msg_.mode_machine(lowstate_->msg_.mode_machine());  // 透传机型/模式标识

    if (params_.motion_mode) {
      // arm_sdk 覆盖：只初始化 双臂 + 头部 + 腰部槽位
      for (int i = 0; i < armDof(); ++i) {
        const int s = armSlot(i);
        auto& c = publisher_->msg_.motor_cmd().at(s);
        c.mode(1);
        c.q(motor.at(s).q());
        c.dq(0.f); c.tau(0.f);
        gainsFor(s, c);
      }
      for (int s : kHeadSlots) {
        auto& c = publisher_->msg_.motor_cmd().at(s);
        c.mode(1); c.q(motor.at(s).q()); c.dq(0.f); c.tau(0.f);
        c.kp(static_cast<float>(params_.kp_head));
        c.kd(static_cast<float>(params_.kd_head));
      }
      for (int s : kWaistSlots) {
        auto& c = publisher_->msg_.motor_cmd().at(s);
        c.mode(1); c.q(motor.at(s).q()); c.dq(0.f); c.tau(0.f);
        c.kp(static_cast<float>(params_.waist_kp));
        c.kd(static_cast<float>(params_.waist_kd));
      }
    } else {
      // rt/lowcmd 全量 35 槽：腿 200/3，踝弱 50/2，臂 50/40/30，头 15（对齐 teleop）
      for (int s = 0; s < 35; ++s) {
        auto& c = publisher_->msg_.motor_cmd().at(s);
        c.mode(1);
        c.q(motor.at(s).q());
        c.dq(0.f); c.tau(0.f);
        if (s < 12) {
          if (isWeakLow(s)) { c.kp(static_cast<float>(params_.kp_weak)); c.kd(static_cast<float>(params_.kd_weak)); }
          else { c.kp(static_cast<float>(params_.kp_high)); c.kd(static_cast<float>(params_.kd_high)); }
        } else if (s == 12 || s == 13) {
          c.kp(static_cast<float>(params_.waist_kp)); c.kd(static_cast<float>(params_.waist_kd));
        } else if (s == 29 || s == 30) {
          c.kp(static_cast<float>(params_.kp_head)); c.kd(static_cast<float>(params_.kd_head));
        } else {
          gainsFor(s, c);
        }
      }
    }
    // 发布一次种子
    publisher_->unlockAndPublish();
  }

  void gainsFor(int slot, unitree_hg::msg::dds_::MotorCmd_& c) const {
    if (isWrist(slot)) {
      c.kp(static_cast<float>(params_.kp_wrist));
      c.kd(static_cast<float>(params_.kd_wrist));
    } else if (isMediumArm(slot)) {
      c.kp(static_cast<float>(params_.kp_elbow));
      c.kd(static_cast<float>(params_.kd_elbow));
    } else {
      c.kp(static_cast<float>(params_.kp_shoulder));
      c.kd(static_cast<float>(params_.kd_shoulder));
    }
  }

  void homeHeadAndWaist(double duration_sec) {
    std::cout << "[arm] head/waist going home (" << duration_sec << " s)..." << std::endl;
    const double dt = 1.0 / params_.publish_hz;
    const int steps = std::max(1, static_cast<int>(duration_sec / dt));
    const Eigen::VectorXd h0 = currentHeadQ();
    std::array<double, 2> w0{};
    {
      std::lock_guard<std::mutex> lock(lowstate_->mutex_);
      w0[0] = lowstate_->msg_.motor_state().at(kWaistSlots[0]).q();
      w0[1] = lowstate_->msg_.motor_state().at(kWaistSlots[1]).q();
    }
    for (int step = 1; step <= steps; ++step) {
      const double scale = 1.0 - static_cast<double>(step) / steps;
      publisher_->lock();
      for (int i = 0; i < 2; ++i) {
        publisher_->msg_.motor_cmd().at(kHeadSlots[i]).q(static_cast<float>(h0[i] * scale));
        publisher_->msg_.motor_cmd().at(kWaistSlots[i]).q(static_cast<float>(w0[i] * scale));
      }
      publisher_->unlockAndPublish();
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    std::cout << "[arm] head/waist home done." << std::endl;
  }

  Eigen::VectorXd currentHeadQ() const {
    std::lock_guard<std::mutex> lock(lowstate_->mutex_);
    Eigen::VectorXd q(2);
    for (int i = 0; i < 2; ++i) q[i] = lowstate_->msg_.motor_state().at(kHeadSlots[i]).q();
    return q;
  }

  // ---- 发布循环（250Hz）--------------------------------------------------
  void publishLoop() {
    const auto period = std::chrono::duration<double>(1.0 / params_.publish_hz);
    while (run_.load()) {
      const auto t0 = std::chrono::steady_clock::now();
      Eigen::VectorXd q, tau;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        q = q_target_;
        tau = tau_ff_;
      }
      if (q.size() != armDof()) {
        // 目标尚未就绪，跳过本帧（start 后目标会立即填充）
        std::this_thread::sleep_for(period);
        continue;
      }
      const Eigen::VectorXd q_clip = clipTargets(q);

      publisher_->lock();
      for (int i = 0; i < armDof(); ++i) {
        const int s = armSlot(i);
        auto& c = publisher_->msg_.motor_cmd().at(s);
        c.mode(1);
        c.q(static_cast<float>(q_clip[i]));
        c.dq(0.f);
        c.tau(static_cast<float>(tau[i]));
      }
      publisher_->unlockAndPublish();

      const auto elapsed = std::chrono::steady_clock::now() - t0;
      const auto remain = period - elapsed;
      if (remain > std::chrono::duration<double>(0)) {
        std::this_thread::sleep_for(remain);
      }
    }
  }

  /// 关节速度限幅（对齐 xr_teleoperate clip_arm_q_target）。
  Eigen::VectorXd clipTargets(const Eigen::VectorXd& q) const {
    if (q.size() != armDof()) return q;
    const Eigen::VectorXd cur = currentArmQ();
    const Eigen::VectorXd delta = q - cur;
    const double maxd = delta.cwiseAbs().maxCoeff();
    const double dt = 1.0 / params_.publish_hz;
    const double scale = maxd / (params_.arm_vel_limit * dt);
    if (scale <= 1.0) return q;
    return cur + delta / scale;
  }

  void setWeightAndPublish(double w) {
    const double c = std::clamp(w, 0.0, 1.0);
    weight_.store(c);
    publisher_->lock();
    publisher_->msg_.mode_pr(static_cast<uint8_t>(std::lround(c * 100.0)));
    publisher_->unlockAndPublish();
  }

  // ---- 成员 --------------------------------------------------------------
  Variant variant_;
  Params params_;
  std::atomic<bool> started_{false};
  std::atomic<bool> run_{false};
  std::atomic<bool> ready_{false};
  std::atomic<double> weight_{0.0};

  std::shared_ptr<unitree::robot::g1::subscription::LowState> lowstate_;
  std::unique_ptr<R1ArmSdkPublisher> publisher_;
  std::thread publish_thread_;

  mutable std::mutex target_mutex_;
  Eigen::VectorXd q_target_;
  Eigen::VectorXd tau_ff_;
};

}  // namespace r1skeleton
