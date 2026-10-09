#pragma once
// ============================================================================
// r1_hand_canfd_bridge.h
// 灵巧手 CANFD 驱动 —— C++ 侧「桥」实现。
//
// 为什么不直接在 C++ 里发 CANFD
// ------------------------------
// 这套手用 gs_usb 用户态协议（USB-CANFD 适配器 `a8fa:8598`）。C++ 侧要用就得在
// 背包上编译 `gsusb-canfd` 的 C++ 库；而 Python 那套（canfd_lib + gsusb_canfd +
// pyusb）**已真机验证**、且离线部署完成。所以这里只做"把位置转发给 Python 桥"，
// 由 `scripts/hand_bridge.py` 独占适配器。
//
// 架构
// ----
//     r1_dual_arm_loco.cpp ──UDP 127.0.0.1:9998──> hand_bridge.py ──CANFD──> 两只手
//
//     Python 桥在**启动时**走完初始化（使能 → 回零 → 位置模式 → 速度 → 电流），
//     之后只做「遥操位置 → set_position」的直接映射。
//
// 安全语义（重要）
// ----------------
//   主循环只在**遥操分支**传 `mode = kPose`；急停 / 掉包 / 保持 / 回零都传 `kIdle`。
//   本驱动**只在 kPose 时发送**，kIdle 时什么都不发 ⇒ 手保持在最后位置不动
//   （与双臂"保持"的语义一致，掉包或急停时手不会自行乱动）。
//
// 报文格式（一行文本，便于抓包调试）
// ----------------------------------
//     "POS l0 l1 l2 l3 l4 l5 r0 r1 r2 r3 r4 r5 mask\n"
//     mask bit0 = 左手有效, bit1 = 右手有效；数值 0..10000
//     "BYE\n"  —— stop() 时发一次
// ============================================================================

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include "r1_hand_interface.h"

namespace r1skeleton {

/// 把遥操手部目标经 UDP 转发给 Python CANFD 桥（`scripts/hand_bridge.py`）。
class CanfdBridgeHandDriver : public HandDriver {
 public:
  explicit CanfdBridgeHandDriver(uint16_t port = 9998, double send_hz = 50.0,
                                 std::string host = "127.0.0.1")
      : host_(std::move(host)), port_(port),
        min_interval_s_(send_hz > 0 ? 1.0 / send_hz : 0.0) {}

  std::string name() const override { return "canfd_bridge"; }

  bool init() override {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      std::cout << "[hand] ✗ socket() 失败，CANFD 桥不可用" << std::endl;
      return false;
    }
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &dst_.sin_addr) != 1) {
      std::cout << "[hand] ✗ 目标地址非法: " << host_ << std::endl;
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    std::cout << "[hand] CANFD 桥驱动就绪 → udp://" << host_ << ":" << port_
              << "（位置由 hand_bridge.py 落到 CAN；kIdle 时不发送）" << std::endl;
    return true;
  }

  bool update(const HandAction& action, HandState& state, double dt) override {
    (void)dt;
    state.present = false;  // 反馈在 Python 侧，这里不读回

    // 只在遥操（kPose）时下发；急停/掉包/保持/回零都传 kIdle ⇒ 不发，手保持不动
    if (action.mode != HandAction::Mode::kPose) return true;
    if (!action.left_raw_valid && !action.right_raw_valid) return true;

    // 限速：主循环 30 Hz，这里默认 50 Hz 上限，避免无谓刷总线
    const auto now = std::chrono::steady_clock::now();
    if (min_interval_s_ > 0.0 &&
        std::chrono::duration<double>(now - last_send_).count() < min_interval_s_) {
      return true;
    }
    last_send_ = now;

    const int mask = (action.left_raw_valid ? 1 : 0) | (action.right_raw_valid ? 2 : 0);
    char buf[192];
    const auto iv = [](double v) {
      // 0..10000，四舍五入并夹紧（PICO 给的就是这个量纲）
      if (v < 0.0) v = 0.0;
      if (v > 10000.0) v = 10000.0;
      return static_cast<int>(v + 0.5);
    };
    std::snprintf(buf, sizeof(buf),
                  "POS %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                  iv(action.left_joint_raw[0]), iv(action.left_joint_raw[1]),
                  iv(action.left_joint_raw[2]), iv(action.left_joint_raw[3]),
                  iv(action.left_joint_raw[4]), iv(action.left_joint_raw[5]),
                  iv(action.right_joint_raw[0]), iv(action.right_joint_raw[1]),
                  iv(action.right_joint_raw[2]), iv(action.right_joint_raw[3]),
                  iv(action.right_joint_raw[4]), iv(action.right_joint_raw[5]),
                  mask);
    sendRaw(buf);
    return true;
  }

  void stop() override {
    if (fd_ >= 0) {
      sendRaw("BYE\n");
      ::close(fd_);
      fd_ = -1;
      std::cout << "[hand] CANFD 桥驱动已停止（发送 " << sent_ << " 帧, 失败 "
                << failed_ << " 次）" << std::endl;
    }
  }

 private:
  void sendRaw(const char* buf) {
    const ssize_t n = ::sendto(fd_, buf, std::strlen(buf), 0,
                               reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
    if (n < 0) {
      if (++failed_ == 1) {
        // 只报一次，避免 30 Hz 刷屏（桥没起来时是常态）
        std::cout << "[hand] ⚠ 发送失败（hand_bridge.py 没在跑？）—— 后续不再重复报"
                  << std::endl;
      }
    } else {
      ++sent_;
    }
  }

  std::string host_;
  uint16_t port_;
  double min_interval_s_ = 0.02;
  int fd_ = -1;
  sockaddr_in dst_{};
  std::chrono::steady_clock::time_point last_send_{};
  unsigned long sent_ = 0;
  unsigned long failed_ = 0;
};

}  // namespace r1skeleton
