#pragma once
// ============================================================================
// r1_hand_interface.h
// 灵巧手驱动接口（预留占位）。
//
// 说明：
//  - 你的灵巧手暂时自行实现，不经过 Unitree DDS 通道。
//  - 本接口把"每帧手部动作"抽象成 HandAction + HandState，
//    由主控制循环（见 r1_dual_arm_loco.cpp 的 arm-thread）以固定频率调用。
//  - 接入你自己的手时：继承 HandDriver 实现 init/update/stop，
//    在 update() 里写你的串口/CAN/总线逻辑即可。
//
// 官方灵巧手 DDS 通道（当你后续想改用宇树官方手时可参考）：
//   Unitree Dex3  : rt/dex3/left/cmd  rt/dex3/right/cmd
//                   (unitree_hg::msg::dds_::HandCmd_，motor_cmd 为 vector<MotorCmd_>)
//   Unitree Dex1  : rt/dex1/left|right/cmd|state（夹爪）
//   Inspire DFX   : rt/inspire/cmd  rt/inspire/state
//   Inspire FTP   : rt/inspire_hand/ctrl/l|r   rt/inspire_hand/state/l|r
//   BrainCo       : rt/brainco/left|right/cmd|state
// 这些 topic 必须在 ChannelFactory::Instance()->Init() 之后创建。
// ============================================================================

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

namespace r1skeleton {

/// 每帧手部动作（抽象层，字段可按你的手自由扩展）。
struct HandAction {
  enum class Mode : int {
    kIdle = 0,      // 不动
    kOpen,          // 张开
    kClose,         // 握拳
    kPose,          // 指定各手指目标（0=张开, 1=握紧）
  };

  Mode mode = Mode::kIdle;

  // —— 原样透传的手关节位置命令（PICOHandLink robot_control.hands，0..10000）——
  // 每一路对应你灵巧手上的一个关节目标位置。left_raw_valid/right_raw_valid 为 true
  // 表示本帧携带了 6 路原始命令；"0..10000 → 各关节实际运动"的换算由 HandDriver 实现
  // （当前骨架为 NullHandDriver，只打印验证；具体映射见 r1_hand_interface.h 顶部说明）。
  std::array<double, 6> left_joint_raw  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> right_joint_raw = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool left_raw_valid = false;
  bool right_raw_valid = false;

  // —— 归一化手指目标（0=张开, 1=握紧；供简易驱动参考，非权威）——
  std::array<double, 5> left_finger  = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 5> right_finger = {0.0, 0.0, 0.0, 0.0, 0.0};
  double left_grip  = 0.0;   // 自定义附加量（如整手开合比例 0..1）
  double right_grip = 0.0;
  bool left_enabled  = false;   // 左手是否已安装/启用
  bool right_enabled = false;
};

/// 手部状态反馈（可选，默认全 -1 表示无反馈）。
struct HandState {
  std::array<double, 5> left_finger  = {-1.0, -1.0, -1.0, -1.0, -1.0};
  std::array<double, 5> right_finger = {-1.0, -1.0, -1.0, -1.0, -1.0};
  bool present = false;

  bool hasFeedback() const { return present; }
};

/// 手部驱动接口：接你的灵巧手时替换 / 实现它。
class HandDriver {
 public:
  virtual ~HandDriver() = default;

  /// 驱动名称（日志用）。
  virtual std::string name() const = 0;

  /// 初始化（打开串口/CAN 等）。失败返回 false，主循环将跳过 update()。
  virtual bool init() = 0;

  /// 每帧调用。dt 为距上次调用的秒数；action 为上层策略给出的动作，
  /// state 为读回的状态（没有反馈则保持默认值）。
  virtual bool update(const HandAction& action, HandState& state, double dt) = 0;

  /// 停机释放（断电前调用，如回中位）。
  virtual void stop() = 0;
};

/// 空实现：只打日志，不连任何硬件。作为骨架默认驱动。
class NullHandDriver : public HandDriver {
 public:
  std::string name() const override { return "null_hand"; }

  bool init() override {
    std::cout << "[hand] NullHandDriver initialized (no hardware)." << std::endl;
    return true;
  }

  bool update(const HandAction& action, HandState& state, double dt) override {
    (void)dt;
    // 1 Hz 打印一次，方便观察主循环确实在驱动手部接口。
    auto now = std::chrono::steady_clock::now();
    if (now - last_log_ > std::chrono::seconds(1)) {
      last_log_ = now;
      std::cout << "[hand] mode=" << static_cast<int>(action.mode)
                << " L=" << (action.left_enabled ? "on" : "off")
                << " R=" << (action.right_enabled ? "on" : "off");
      if (action.left_raw_valid || action.right_raw_valid) {
        const auto fmt = [](const std::array<double, 6>& v) {
          std::string s;
          char buf[32];
          for (int i = 0; i < 6; ++i) {
            snprintf(buf, sizeof(buf), "%s%.0f", i ? "," : "", v[i]);
            s += buf;
          }
          return s;
        };
        std::cout << " | rawL[" << (action.left_raw_valid ? fmt(action.left_joint_raw) : "-")
                  << "] rawR[" << (action.right_raw_valid ? fmt(action.right_joint_raw) : "-") << "]";
      }
      std::cout << " (NullHandDriver: raw->关节运动映射等你实现 HandDriver 后生效)"
                << std::endl;
    }
    state.present = false;  // 无硬件，无反馈
    return true;
  }

  void stop() override {
    std::cout << "[hand] NullHandDriver stopped." << std::endl;
  }

 private:
  std::chrono::steady_clock::time_point last_log_{};
};

// ----------------------------------------------------------------------------
// 参考：如果你之后要驱动 Unitree Dex3（官方灵巧手），骨架如下
// （本 repo 的 include/unitree/idl/hg/HandCmd_.hpp 已提供类型）。
// 注意 HandCmd_.motor_cmd() 是 vector<MotorCmd_>，每个元素与 Dex3 电机一一对应。
// ----------------------------------------------------------------------------
// #include <unitree/robot/channel/channel_publisher.hpp>
// #include <unitree/idl/hg/HandCmd_.hpp>
//
// class Dex3HandDriver : public HandDriver {
//  public:
//   bool init() override {
//     left_pub_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>("rt/dex3/left/cmd");
//     right_pub_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>("rt/dex3/right/cmd");
//     left_pub_->InitChannel(); right_pub_->InitChannel();
//     return true;
//   }
//   bool update(const HandAction& a, HandState& s, double dt) override {
//     (void)dt;
//     if (a.left_enabled) {
//       unitree_hg::msg::dds_::HandCmd_ msg;
//       msg.motor_cmd().resize(6);  // Dex3 每手 6 自由度，按你的手型配置
//       for (size_t i = 0; i < msg.motor_cmd().size(); ++i) {
//         msg.motor_cmd()[i].mode(1);
//         msg.motor_cmd()[i].q(a.left_finger[i] * kMaxQRad_);  // 归一化 -> 弧度
//         msg.motor_cmd()[i].dq(0.f);
//         msg.motor_cmd()[i].kp(kKp_); msg.motor_cmd()[i].kd(kKd_);
//         msg.motor_cmd()[i].tau(0.f);
//       }
//       left_pub_->Write(msg);
//     }
//     return true;
//   }
//   ...
// };

}  // namespace r1skeleton
