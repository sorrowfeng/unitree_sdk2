#pragma once
// ============================================================================
// r1_hand_dds.h
// 灵巧手驱动（DDS 版）—— 只做「映射 + 发 DDS」，**不碰串口**。
//
// ── 为什么这样分层（2026-09-23 在 R1-EDU 算力背包 PC2 上实测的结论）──────────
//   宇树工厂里已经躺着**四家**灵巧手的接入目录：
//       ~/brainco_hand_service   （另有 systemd 服务 brainco_hand.service）
//       ~/linker_hand_service
//       ~/h1_inspire_service
//       ~/stark-serialport-example
//   它们的结构**完全同构**，都是「串口 ↔ DDS 桥」：
//       串口(Modbus RTU / 私有协议)  <── 桥进程 ──>  DDS
//   对外只暴露一对 topic，且**三家用的是同一个消息类型**：
//       订阅  rt/<厂商>/<side>/cmd     unitree_go::msg::dds_::MotorCmds_
//       发布  rt/<厂商>/<side>/state   unitree_go::msg::dds_::MotorStates_
//
//   ⇒ 遥操主程序**不应该**直接驱动串口/总线。它只需要把「目标关节位置」按厂商的
//     指序与方向约定换算好，发到对应 topic 即可。串口协议留在厂商桥进程里。
//   ⇒ 好处：加一款新手 = 加一条 HandProfile + 跑厂商的桥进程，**遥操代码零改动**；
//           也不会让主程序因为某个桥挂了而卡在串口 read 上。
//
// ── 三家实测参数（2026-09-23，背包 PC2）────────────────────────────────────
//   ┌─────────┬────────────────────────────────────┬──────┬──────────────────────────────┬───────────────┐
//   │ 厂商    │ topic                              │ 元素 │ 指序（文档定义）             │ q 语义        │
//   ├─────────┼────────────────────────────────────┼──────┼──────────────────────────────┼───────────────┤
//   │ BrainCo │ rt/brainco/{left,right}/{cmd,state}│ 6/手 │ 拇指,拇指副,食,中,无名,小指  │ [0,1] ×1000   │
//   │ Linker  │ rt/linker/{left,right}/{cmd,state} │ 6/手 │ 同上                         │ 0=张开 1=握紧 │
//   │ Inspire │ rt/inspire/{cmd,state}             │ 12   │ 小指,无名,中,食,拇指弯,拇指转 │ 0=握紧 1=张开 │
//   └─────────┴────────────────────────────────────┴──────┴──────────────────────────────┴───────────────┘
//   证据来源：三家 README_zh-CN.md / README.md + 各自源码
//     · BrainCo  main.cpp:24      baud=460800, slave 0x7e(L)/0x7f(R), Modbus RTU, /dev/ttyHand0
//                 main.cpp:64     modbus_set_finger_unit_mode(FINGER_UNIT_MODE_NORMALIZED)
//                 main.cpp:98     positions = clamp(q,0,1) * 1000
//                 main.cpp:127,131 topic rt/brainco/<ns>/{cmd,state}
//     · Linker   hand_o6.hpp:44   baud=4000000, slave 0x27, /dev/ttyCH343USB*
//                 hand_dds_service.cpp:38   angles = normalizeToByte(1.0f - q)
//                 ↑ 注释原文「Upper layer: 0 (open) -> 1 (closed); Device: 255 (open) -> 0 (closed)」
//     · Inspire  param.h       默认 ns="inspire"，`-s /dev/ttyUSB0`，B115200
//                 inspire_ctrl.cpp:23,26  topic rt/inspire/{cmd,state}，resize(12)
//                 h1_hand_example.cpp    ctrl(): cmds()[i]=right, cmds()[i+6]=left
//                                        注释「The angles should be in the range [0, 1]. 0: close 1: open」
//
//   ⚠️ 三个最容易踩的坑：
//     1. **Inspire 的 12 元素是左右合一**：[0..5]=右手、[6..11]=左手（例程 ctrl() 就这么填）。
//        BrainCo/Linker 则是左右各一对 topic、每 topic 6 元素。布局不同，不能混。
//     2. **Inspire 的指序与 Linker/BrainCo 相反**（它从"小指"数起），**q 方向也相反**
//        （Inspire 0=握紧，Linker 0=张开）。换手时这两处最容易错，且错了表现为"手反向乱抓"。
//     3. **BrainCo 的 q 方向在代码里看不出来**（只知 [0,1]→×1000，NORMALIZED 模式）。
//        本文件把它作为可翻转字段暴露，**必须实物标定后定死**，不要照抄本文件默认值上机。
//
// ── 源侧语义（我们自己的约定，不是某款手的关节序）─────────────────────────
//   HandAction.<side>_joint_raw[6]  来自 PICOHandLink robot_control.hands，0..10000，
//   r1_pico_udp.h:137 注释的布局是 [固定, trigger, trigger, grip, grip, grip]。
//   HandAction.<side>_finger[5]     是主程序填好的归一化量，**0 = 张开，1 = 握紧**。
//   ⚠️ 这 6 路**不是**灵巧手的关节序，所以「源 → 手关节」必须显式映射（见 JointMap），
//      按位置直通是错的。
//
// ── 用法 ──────────────────────────────────────────────────────────────────
//   // 在 main 里（ChannelFactory::Instance()->Init(0, iface) 之后，手部 init 才有效）
//   r1skeleton::DdsHandDriver hand(r1skeleton::hand_profile_brainco());
//   //  或 hand_profile_linker() / hand_profile_inspire() / 自己填一份
//   hand.init();
//   ... hand.update(action, state, dt); ... hand.stop();
//
//   然后把厂商的桥进程跑起来（例如 brainco_hand.service），手就动了。
//   完整选型与上机步骤见 R1_BACKPACK_ARCHITECTURE.md 第 4 节。
// ============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "r1_hand_interface.h"

namespace r1skeleton {

// ---------------------------------------------------------------------------
// 源通道：每个输出关节从哪来
// ---------------------------------------------------------------------------
enum class HandSrc : std::uint8_t {
  /// HandAction.<side>_joint_raw[src_idx] / 10000 → [0,1]（0=张开 1=握紧，与 finger 同向）
  kRaw = 0,
  /// HandAction.<side>_finger[src_idx] → 已是 [0,1]（0=张开 1=握紧）
  kFinger = 1,
  /// 常数：src_idx / 1000.0（调试/固定关节用，例如"拇指副指常开"）
  kConst = 2,
};

/// 一个输出关节的映射：q_out = clamp(offset + scale * src, 0, 1)
/// 方向用 scale 表达：+1 与源同向，-1 反向（Inspire 这种"0=握紧"的手就用 -1）。
struct JointMap {
  HandSrc src = HandSrc::kFinger;
  std::uint8_t src_idx = 0;
  float scale = 1.0f;
  float offset = 0.0f;
};

/// 一款手的完整描述。**加新手 = 加一份这个结构体**，不用改驱动代码。
struct HandProfile {
  const char* name = "unset";

  /// DDS topic 命名空间：rt/<ns>/...
  const char* ns = "";

  /// 布局：
  ///   false —— 左右各一对 topic：rt/<ns>/left/cmd + rt/<ns>/right/cmd，每个 dof 个元素
  ///   true  —— 单个 topic 对：rt/<ns>/cmd，2*dof 个元素，[0..dof)=右、[dof..2*dof)=左
  bool combined = false;

  /// 每只手自由度数（= 单个 topic 的元素数 / combined 时的一半）
  std::uint8_t dof = 6;

  /// joints[i] 描述**该厂商文档里第 i 个关节**从哪个源通道来。
  /// 索引次序必须与该厂商文档一致 —— 这是换手时最容易写错的地方。
  std::array<JointMap, 6> joints{};

  /// 速度通道常数。三家的桥都按 [0,1] 归一化消费 dq（BrainCo ×1000、Linker ×255）。
  float dq = 1.0f;

  /// 是否订阅 state topic 读回手指位置（桥没跑时收不到，不影响下发）。
  bool state_feedback = true;

  /// 标定备注，启动时打印，提醒哪些值是"待标定"的。
  const char* note = "";
};

// ---------------------------------------------------------------------------
// 预设
// ---------------------------------------------------------------------------

/// BrainCo Revo2：rt/brainco/{left,right}/{cmd,state}，6 元素，
/// 指序 [拇指, 拇指副指, 食指, 中指, 无名指, 小指]，q∈[0,1]。
/// ⚠️ q 方向无代码证据，默认取"与源同向"（0=张开），**上机前必须标定**；
///    若实测反向，把前 5 个关节的 scale 改成 -1.0f、offset 改成 1.0f 即可。
inline HandProfile hand_profile_brainco() {
  HandProfile p;
  p.name = "brainco";
  p.ns = "brainco";
  p.combined = false;
  p.dof = 6;
  p.dq = 1.0f;
  p.state_feedback = true;
  p.joints = {{
      {HandSrc::kFinger, 0, 1.0f, 0.0f},  // 拇指      ← finger[0]
      {HandSrc::kFinger, 0, 1.0f, 0.0f},  // 拇指副指  ← 跟随拇指（PICO 无独立通道）
      {HandSrc::kFinger, 1, 1.0f, 0.0f},  // 食指      ← finger[1]
      {HandSrc::kFinger, 2, 1.0f, 0.0f},  // 中指      ← finger[2]
      {HandSrc::kFinger, 3, 1.0f, 0.0f},  // 无名指    ← finger[3]
      {HandSrc::kFinger, 4, 1.0f, 0.0f},  // 小指      ← finger[4]
  }};
  p.note =
      "q 方向未标定（代码里看不出 0 是张开还是握紧）；指序=文档序(拇指..小指)。";
  return p;
}

/// Linker O6：rt/linker/{left,right}/{cmd,state}，6 元素，
/// 指序与 BrainCo 相同；**q=0 张开、q=1 握紧**（hand_dds_service.cpp 有明确注释）。
inline HandProfile hand_profile_linker() {
  HandProfile p;
  p.name = "linker";
  p.ns = "linker";
  p.combined = false;
  p.dof = 6;
  p.dq = 1.0f;
  p.state_feedback = true;
  p.joints = {{
      {HandSrc::kFinger, 0, 1.0f, 0.0f},  // 拇指
      {HandSrc::kFinger, 0, 1.0f, 0.0f},  // 拇指副指 ← 跟随拇指
      {HandSrc::kFinger, 1, 1.0f, 0.0f},  // 食指
      {HandSrc::kFinger, 2, 1.0f, 0.0f},  // 中指
      {HandSrc::kFinger, 3, 1.0f, 0.0f},  // 无名指
      {HandSrc::kFinger, 4, 1.0f, 0.0f},  // 小指
  }};
  p.note = "q 方向有注释证据：0=张开 1=握紧（与源同向）。";
  return p;
}

/// Inspire（H1 版）：rt/inspire/{cmd,state}，**12 元素左右合一**，[0..5]=右、[6..11]=左；
/// 指序 **[小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]**（与上面两家相反）；
/// **q=0 握紧、q=1 张开**（与上面两家相反）⇒ 用 scale=-1, offset=1 翻转。
inline HandProfile hand_profile_inspire() {
  HandProfile p;
  p.name = "inspire";
  p.ns = "inspire";
  p.combined = true;
  p.dof = 6;
  p.dq = 1.0f;
  p.state_feedback = true;
  // 指序按厂商文档 = [小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转]；
  // 方向统一用 scale=-1 / offset=1 把"0=握紧"翻成"0=张开"，便于与另外两家对照。
  p.joints = {{
      {HandSrc::kFinger, 4, -1.0f, 1.0f},  // 小指       ← finger[4]
      {HandSrc::kFinger, 3, -1.0f, 1.0f},  // 无名指     ← finger[3]
      {HandSrc::kFinger, 2, -1.0f, 1.0f},  // 中指       ← finger[2]
      {HandSrc::kFinger, 1, -1.0f, 1.0f},  // 食指       ← finger[1]
      {HandSrc::kFinger, 0, -1.0f, 1.0f},  // 拇指弯曲   ← finger[0]
      {HandSrc::kFinger, 0, -1.0f, 1.0f},  // 拇指旋转   ← 跟随拇指
  }};
  p.note =
      "12 元素左右合一(右0-5/左6-11)；指序倒序(小指..拇指)；q 方向与源相反(已 scale=-1)。";
  return p;
}

// ---------------------------------------------------------------------------
// 驱动
// ---------------------------------------------------------------------------
class DdsHandDriver : public HandDriver {
 public:
  explicit DdsHandDriver(HandProfile profile) : p_(std::move(profile)) {}

  std::string name() const override { return std::string("dds_hand[") + p_.name + "]"; }

  bool init() override {
    if (p_.combined) {
      left_pub_ = std::make_shared<unitree::robot::ChannelPublisher<MotorCmds>>(cmdTopic(""));
      left_pub_->InitChannel();
      if (p_.state_feedback) {
        right_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MotorStates>>(
            stateTopic(""));
        // combined 布局：一条 state 消息里同时含左右（右 [0..dof)、左 [dof..2dof)），
        // 所以只订一次，在一次回调里把两侧都填上。
        right_sub_->InitChannel(
            [this](const void* m) {
              onState(m, 0, /*is_left=*/false);
              onState(m, p_.dof, /*is_left=*/true);
            },
            1);
      }
    } else {
      left_pub_ = std::make_shared<unitree::robot::ChannelPublisher<MotorCmds>>(cmdTopic("left"));
      right_pub_ = std::make_shared<unitree::robot::ChannelPublisher<MotorCmds>>(cmdTopic("right"));
      left_pub_->InitChannel();
      right_pub_->InitChannel();
      if (p_.state_feedback) {
        left_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MotorStates>>(
            stateTopic("left"));
        left_sub_->InitChannel([this](const void* m) { onState(m, 0, true); }, 1);
        right_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<MotorStates>>(
            stateTopic("right"));
        right_sub_->InitChannel([this](const void* m) { onState(m, 0, false); }, 1);
      }
    }
    std::cout << "[hand] " << name() << " ready | cmd topic="
              << (p_.combined ? cmdTopic("") : cmdTopic("left") + " + " + cmdTopic("right"))
              << " | dof=" << static_cast<int>(p_.dof)
              << (p_.combined ? " (L/R in one topic)" : " (per-side)") << std::endl;
    if (p_.note && *p_.note) std::cout << "[hand] ⚠️ " << p_.note << std::endl;
    std::cout << "[hand] 注意：本驱动只发 DDS，串口侧需厂商桥进程在跑（如 brainco_hand.service）"
              << std::endl;
    return true;
  }

  bool update(const HandAction& action, HandState& state, double dt) override {
    (void)dt;
    if (!action.left_enabled && !action.right_enabled) {
      // 两侧都不在线：不动作，保持上一次目标（桥进程仍会维持当前位置）
      state.present = have_state_;
      fillState(state);
      return true;
    }

    if (p_.combined) {
      MotorCmds msg;
      msg.cmds().resize(static_cast<std::size_t>(2 * p_.dof));
      buildOneSide(msg, 0, action, /*is_left=*/false);
      buildOneSide(msg, p_.dof, action, /*is_left=*/true);
      if (left_pub_) left_pub_->Write(msg);
    } else {
      MotorCmds lmsg, rmsg;
      lmsg.cmds().resize(static_cast<std::size_t>(p_.dof));
      rmsg.cmds().resize(static_cast<std::size_t>(p_.dof));
      buildOneSide(lmsg, 0, action, /*is_left=*/true);
      buildOneSide(rmsg, 0, action, /*is_left=*/false);
      if (left_pub_ && action.left_enabled) left_pub_->Write(lmsg);
      if (right_pub_ && action.right_enabled) right_pub_->Write(rmsg);
    }

    state.present = have_state_;
    fillState(state);
    return true;
  }

  void stop() override {
    // 只关通道，不发"松手"指令 —— 松手属安全策略，应由上层（主循环的 safe 分支）显式决定。
    left_pub_.reset();
    right_pub_.reset();
    left_sub_.reset();
    right_sub_.reset();
    std::cout << "[hand] " << name() << " stopped." << std::endl;
  }

 private:
  using MotorCmds = unitree_go::msg::dds_::MotorCmds_;
  using MotorStates = unitree_go::msg::dds_::MotorStates_;

  /// combined 布局下 side 无意义（左右共用一个 topic），传空串即可。
  std::string cmdTopic(const std::string& side) const {
    if (p_.combined) return std::string("rt/") + p_.ns + "/cmd";
    return std::string("rt/") + p_.ns + "/" + side + "/cmd";
  }
  std::string stateTopic(const std::string& side) const {
    if (p_.combined) return std::string("rt/") + p_.ns + "/state";
    return std::string("rt/") + p_.ns + "/" + side + "/state";
  }

  /// 取源通道归一化值
  static float source(const JointMap& jm, const HandAction& a, bool is_left) {
    const std::array<double, 6>& raw = is_left ? a.left_joint_raw : a.right_joint_raw;
    const std::array<double, 5>& fing = is_left ? a.left_finger : a.right_finger;
    const bool raw_valid = is_left ? a.left_raw_valid : a.right_raw_valid;
    switch (jm.src) {
      case HandSrc::kRaw: {
        if (!raw_valid) return 0.0f;  // 无原始数据 → 当"张开"处理，避免误抓
        const std::size_t i = jm.src_idx < 6 ? jm.src_idx : 0;
        return static_cast<float>(std::clamp(raw[i] / 10000.0, 0.0, 1.0));
      }
      case HandSrc::kFinger: {
        const std::size_t i = jm.src_idx < 5 ? jm.src_idx : 4;
        return static_cast<float>(std::clamp(fing[i], 0.0, 1.0));
      }
      case HandSrc::kConst:
      default:
        return static_cast<float>(jm.src_idx) / 1000.0f;
    }
  }

  void buildOneSide(MotorCmds& msg, std::size_t base, const HandAction& a, bool is_left) const {
    for (std::size_t i = 0; i < p_.dof; ++i) {
      const JointMap& jm = p_.joints[i];

      // 先把源归一化成"手指开合语义"：0 = 张开，1 = 握紧
      float src = 0.0f;  // 张开
      switch (a.mode) {
        case HandAction::Mode::kIdle:
          src = 0.0f;
          break;
        case HandAction::Mode::kOpen:
          src = 0.0f;
          break;
        case HandAction::Mode::kClose:
          src = 1.0f;
          break;
        case HandAction::Mode::kPose:
        default:
          src = source(jm, a, is_left);
          break;
      }
      // 再按该手的方向约定换算成它自己的 q
      const float q = std::clamp(jm.offset + jm.scale * src, 0.0f, 1.0f);

      auto& c = msg.cmds()[base + i];
      c.mode(1);  // 位置模式（三家桥都只消费 q/dq，mode 仅作兼容）
      c.q(q);
      c.dq(p_.dq);
      c.tau(0.0f);
      c.kp(0.0f);
      c.kd(0.0f);
    }
  }

  /// 收到状态。
  /// @param offset  该侧在 MotorStates.states() 里的起始下标
  ///                （combined 布局：右 0 / 左 dof；分侧布局：两侧都是 0）
  /// @param is_left true = 这批数据属于左手
  void onState(const void* message, std::size_t offset, bool is_left) {
    const auto* m = static_cast<const MotorStates*>(message);
    if (!m) return;
    const auto& st = m->states();
    if (st.size() < offset + p_.dof) return;
    std::array<double, 5>& dst = is_left ? left_q_ : right_q_;
    for (std::size_t i = 0; i < p_.dof && i < 5; ++i) {
      const float q = st[offset + i].q();
      // 反算回"0=张开 1=握紧"的展示语义（与 JointMap 的 scale/offset 逆运算）
      const JointMap& jm = p_.joints[i];
      const float v = (jm.scale != 0.0f) ? (q - jm.offset) / jm.scale : q;
      dst[i] = std::clamp(v, 0.0f, 1.0f);
    }
    have_state_ = true;
  }

  void fillState(HandState& s) const {
    if (!have_state_) return;
    s.left_finger = left_q_;
    s.right_finger = right_q_;
  }

  HandProfile p_;
  std::shared_ptr<unitree::robot::ChannelPublisher<MotorCmds>> left_pub_;
  std::shared_ptr<unitree::robot::ChannelPublisher<MotorCmds>> right_pub_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<MotorStates>> left_sub_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<MotorStates>> right_sub_;
  std::array<double, 5> left_q_ = {0, 0, 0, 0, 0};
  std::array<double, 5> right_q_ = {0, 0, 0, 0, 0};
  bool have_state_ = false;
};

}  // namespace r1skeleton
