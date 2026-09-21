#pragma once
// ============================================================================
// r1_pico_udp.h
// PICOHandLink UDP-JSON v3 接收与解析（机器人端）。
//
// 数据源：D:\Project\PICOProject\PICOHandLink（PICO 端 OpenXR App）
//   协议文档：docs/UDP_JSON_V3_zh.md
//   编码实现：openxr-app/src/main/cpp/HandLinkProtocol.cpp
//
// 要点（与协议文档一致）：
//   - 顶层固定字段 + safety / teleop / hmd / controllers / robot_control 等可选模块。
//   - 运动执行唯一许可：emergency_stop_latched == false 且 safety.safe_to_execute == true
//     且 operator_mode == active_stream（见 safeToExecute()）。
//   - operator_mode 语义：active_stream 连续遥操；return_zero 一键回零
//     （不要求 active_stream，见 returnZeroRequested()）；stop_signal 停止包。
//   - 急停闩锁优先级最高，压过包括 return_zero 在内的一切判据。
//   - pose.position 单位 m；orientation 为 pitch/yaw/roll（deg），同时携带
//     orientation_quat{x,y,z,w}（OpenXR 原始单位四元数，w 为实部）。
//     解析端优先用四元数重建姿态（无万向锁），缺省才回退欧拉 ZYX
//     （R = Rz(yaw) * Ry(pitch) * Rx(roll)，与 PICO 端 EulerDegToQuaternion 互逆）。
//   - 本文件只消费当前实际发送的字段；未知字段忽略。
//
// 提供：
//   1) parsePicoPacket()  —— 字符串 JSON -> PicoTeleopPacket
//   2) PicoUdpReceiver    —— POSIX UDP 接收（可配超时）
//   3) 位姿重建 toOpenXrPose()（欧拉角 -> 4x4 齐次，OpenXR 世界系）
// ============================================================================

#include "unitree/common/json/json.hpp"
#include "unitree/common/any.hpp"

#include <eigen3/Eigen/Dense>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace r1skeleton {
namespace pico {

// ---------------------------------------------------------------------------
// 数据结构（只含当前协议实际发送的字段）
// ---------------------------------------------------------------------------

struct PicoPose3 {
  double x = 0.0, y = 0.0, z = 0.0;          // 单位 m（OpenXR 世界系）
  double pitch_deg = 0.0, yaw_deg = 0.0, roll_deg = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  // OpenXR 原始单位四元数（w 为实部）
  bool has_quat = false;                     // JSON 是否携带 orientation_quat
  bool valid = false;

  /// 重建 OpenXR 世界系 4x4 齐次位姿。
  /// 优先用原始四元数（无万向锁）；缺省回退欧拉(deg, ZYX)：
  ///   R = Rz(yaw)Ry(pitch)Rx(roll)，与 PICO 端 EulerDegToQuaternion 互逆。
  Eigen::Isometry3d toOpenXrPose() const {
    const double d2r = M_PI / 180.0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    if (has_quat) {
      Eigen::Quaterniond q(qw, qx, qy, qz);   // Eigen 顺序 (w,x,y,z)，与 XrQuaternionf(x,y,z,w) 对应
      if (q.squaredNorm() > 1e-8) {
        q.normalize();
        T.linear() = q.toRotationMatrix();
        return T;
      }
      // 非法四元数：回退欧拉
    }
    T.linear() = (Eigen::AngleAxisd(yaw_deg * d2r, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(pitch_deg * d2r, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(roll_deg * d2r, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return T;
  }
};

struct PicoInput {
  double trigger = 0.0;          // 0..1 模拟量
  double grip = 0.0;             // 0..1 模拟量
  double thumb_x = 0.0, thumb_y = 0.0;
  bool primary_pressed = false;  // 左 X / 右 A
  bool secondary_pressed = false; // 左 Y / 右 B
};

struct PicoSide {
  bool valid = false;                // quality == "live"
  bool calibration_applied = false;
  PicoPose3 pose;                    // 应用校准后的 grip pose（遥操作端点）
  PicoPose3 aim_pose;
  PicoInput input;
};

struct PicoControllers {
  PicoSide left, right;      // 原始 grip/aim pose（未校准，OpenXR app space）
  bool output_valid = false;
};

struct PicoSafety {
  bool operator_active = false;
  bool teleop_armed = false;
  bool emergency_stop_latched = false;
  bool session_running = false;
  bool session_focused = false;
  bool calibration_ready = false;
  bool tracking_valid = false;
  bool sample_fresh = false;
  bool safe_to_execute = false;      // 唯一运动执行许可
  double sample_age_ms = 0.0;
  double max_sample_age_ms = 250.0;
  std::string reason;
};

struct PicoRobotControl {
  std::string model;                 // f1 / h1（R1 接入时仅作日志，不用其关节序）
  bool has_arms = false;
  bool has_hands = false;
  bool has_head = false;
  bool has_waist = false;
  bool has_lift = false;
  bool has_base = false;
  std::vector<double> left_arm;      // 7 关节（PICO 端已 IK，仅参考/调试）
  std::vector<double> right_arm;
  std::vector<double> left_hand;     // 0..10000, [固定, trigger, trigger, grip, grip, grip]
  std::vector<double> right_hand;
  double head_yaw_deg = 0.0, head_pitch_deg = 0.0;
  double base_linear_x = 0.0, base_linear_y = 0.0, base_angular_z = 0.0;  // m/s, m/s, rad/s
};

struct PicoTeleopPacket {
  uint64_t sequence = 0;
  double timestamp_ms = 0.0;
  double age_ms = 0.0;
  std::string operator_mode;         // active_stream / diagnostic_capture / return_zero / stop_signal
  std::string sdk;
  bool calibrated = false;
  PicoSafety safety;
  bool teleop_valid = false;
  PicoSide left, right;         // teleop（校准后端点）
  PicoControllers ctrl;         // controllers（原始手柄数据，与 Unitree 摇操同源）
  PicoPose3 hmd_pose;
  bool hmd_live = false;
  PicoRobotControl robot;

  bool hasData() const { return sequence > 0; }

  /// 急停闩锁：优先级最高，压过其余一切判据（含 return_zero）。
  bool emergencyStopLatched() const { return safety.emergency_stop_latched; }

  /// 一键回零请求：未闩锁急停，且 operator_mode == "return_zero"。
  ///
  /// 注意这里**不要求** safe_to_execute / active_stream：回零期间 PICO 端本就
  /// 把 operator_mode 置为 return_zero（不是 active_stream），若用 safeToExecute()
  /// 兜底则该分支永远不可达。回零是操作者的主动请求，只再叠加"包新鲜"即可执行。
  bool returnZeroRequested() const {
    return !emergencyStopLatched() && operator_mode == "return_zero";
  }

  /// 唯一运动执行许可：未闩锁急停 + safe_to_execute + 处于连续发送状态。
  ///
  /// 急停闩锁必须在这里挡住：PICO 端可能在置 emergency_stop_latched=true 的
  /// 同时保持 safe_to_execute=true（两者是两个独立字段），此时旧实现会放行运动。
  bool safeToExecute() const {
    return !emergencyStopLatched() && safety.safe_to_execute &&
           operator_mode == "active_stream";
  }
};

// ---------------------------------------------------------------------------
// JSON 解析（基于 unitree::common::json）
// ---------------------------------------------------------------------------
namespace detail {

using unitree::common::Any;
using unitree::common::AnyCast;
using unitree::common::AnyNumberCast;
using unitree::common::IsBool;
using unitree::common::IsJsonArray;
using unitree::common::IsJsonMap;
using unitree::common::JsonArray;
using unitree::common::JsonMap;
using unitree::common::ToString;

inline const Any* find(const JsonMap& m, const char* key) {
  auto it = m.find(key);
  return it == m.end() ? nullptr : &(it->second);
}

inline double asNumber(const Any* a, double def = 0.0) {
  if (a == nullptr || a->Empty()) return def;
  try {
    return AnyNumberCast<double>(*a);
  } catch (...) {
    return def;
  }
}

inline bool asBool(const Any* a, bool def = false) {
  if (a == nullptr || a->Empty()) return def;
  try {
    if (IsBool(*a)) return AnyCast<bool>(*a);
    return asNumber(a, def ? 1.0 : 0.0) != 0.0;
  } catch (...) {
    return def;
  }
}

inline std::string asString(const Any* a, const std::string& def = "") {
  if (a == nullptr || a->Empty()) return def;
  try {
    return ToString(*a);
  } catch (...) {
    return def;
  }
}

inline const JsonMap* asMap(const Any* a) {
  if (a == nullptr || a->Empty() || !IsJsonMap(*a)) return nullptr;
  try {
    return &AnyCast<JsonMap>(*a);
  } catch (...) {
    return nullptr;
  }
}

inline const JsonArray* asArray(const Any* a) {
  if (a == nullptr || a->Empty() || !IsJsonArray(*a)) return nullptr;
  try {
    return &AnyCast<JsonArray>(*a);
  } catch (...) {
    return nullptr;
  }
}

inline PicoPose3 parsePose(const JsonMap& m) {
  PicoPose3 p;
  const JsonMap* pos = asMap(find(m, "position"));
  const JsonMap* ori = asMap(find(m, "orientation"));
  if (pos) {
    p.x = asNumber(find(*pos, "x"));
    p.y = asNumber(find(*pos, "y"));
    p.z = asNumber(find(*pos, "z"));
  }
  if (ori) {
    p.pitch_deg = asNumber(find(*ori, "pitch"));
    p.yaw_deg = asNumber(find(*ori, "yaw"));
    p.roll_deg = asNumber(find(*ori, "roll"));
  }
  // 原始单位四元数（OpenXR 原样，w 为实部）——与欧拉角同源但无万向锁
  const JsonMap* quat = asMap(find(m, "orientation_quat"));
  if (quat) {
    p.qx = asNumber(find(*quat, "x"));
    p.qy = asNumber(find(*quat, "y"));
    p.qz = asNumber(find(*quat, "z"));
    p.qw = asNumber(find(*quat, "w"), 1.0);
    p.has_quat = true;
  }
  p.valid = true;
  return p;
}

inline PicoInput parseInput(const JsonMap& m) {
  PicoInput in;
  in.trigger = asNumber(find(m, "trigger"));
  in.grip = asNumber(find(m, "grip"));
  const JsonMap* ts = asMap(find(m, "thumbstick"));
  if (ts) {
    in.thumb_x = asNumber(find(*ts, "x"));
    in.thumb_y = asNumber(find(*ts, "y"));
  }
  in.primary_pressed = asBool(find(m, "primary_pressed"));
  in.secondary_pressed = asBool(find(m, "secondary_pressed"));
  return in;
}

inline PicoSide parseSide(const JsonMap& m) {
  PicoSide s;
  const std::string quality = asString(find(m, "quality"));
  s.valid = (quality == "live") || asBool(find(m, "valid"));
  s.calibration_applied = asBool(find(m, "calibration_applied"));
  const JsonMap* pose = asMap(find(m, "pose"));
  if (pose) s.pose = parsePose(*pose);
  const JsonMap* aim = asMap(find(m, "aim_pose"));
  if (aim) s.aim_pose = parsePose(*aim);
  const JsonMap* in = asMap(find(m, "input"));
  if (in) s.input = parseInput(*in);
  return s;
}

inline bool parseVec(const Any* a, std::vector<double>& out) {
  const JsonArray* arr = asArray(a);
  if (!arr) return false;
  out.clear();
  out.reserve(arr->size());
  for (const Any& v : *arr) out.push_back(asNumber(&v, 0.0));
  return true;
}

}  // namespace detail

/// 解析完整 UDP JSON 报文。失败返回 false（包不合法，机器人端应保持停止）。
inline bool parsePicoPacket(const std::string& json_text, PicoTeleopPacket& out) {
  using namespace detail;
  try {
    unitree::common::Any root = unitree::common::FromJsonString(json_text);
    if (root.Empty() || !IsJsonMap(root)) return false;
    const JsonMap& m = AnyCast<JsonMap>(root);

    out.sequence = static_cast<uint64_t>(asNumber(find(m, "sequence")));
    out.timestamp_ms = asNumber(find(m, "timestamp"));
    out.age_ms = asNumber(find(m, "age_ms"));
    out.operator_mode = asString(find(m, "operator_mode"), "stop_signal");
    out.sdk = asString(find(m, "sdk"));
    out.calibrated = asBool(find(m, "calibrated"));

    const JsonMap* safety = asMap(find(m, "safety"));
    if (safety) {
      PicoSafety& s = out.safety;
      s.operator_active = asBool(find(*safety, "operator_active"));
      s.teleop_armed = asBool(find(*safety, "teleop_armed"));
      s.emergency_stop_latched = asBool(find(*safety, "emergency_stop_latched"));
      s.session_running = asBool(find(*safety, "session_running"));
      s.session_focused = asBool(find(*safety, "session_focused"));
      s.calibration_ready = asBool(find(*safety, "calibration_ready"));
      s.tracking_valid = asBool(find(*safety, "tracking_valid"));
      s.sample_fresh = asBool(find(*safety, "sample_fresh"));
      s.safe_to_execute = asBool(find(*safety, "safe_to_execute"));
      s.sample_age_ms = asNumber(find(*safety, "sample_age_ms"));
      s.max_sample_age_ms = asNumber(find(*safety, "max_sample_age_ms"), 250.0);
      s.reason = asString(find(*safety, "reason"));
    }

    const JsonMap* teleop = asMap(find(m, "teleop"));
    if (teleop) {
      out.teleop_valid = asBool(find(*teleop, "output_valid"));
      const JsonMap* l = asMap(find(*teleop, "left"));
      const JsonMap* r = asMap(find(*teleop, "right"));
      if (l) out.left = parseSide(*l);
      if (r) out.right = parseSide(*r);
    }

    const JsonMap* ctrls = asMap(find(m, "controllers"));
    if (ctrls) {
      const JsonMap* cl = asMap(find(*ctrls, "left"));
      const JsonMap* cr = asMap(find(*ctrls, "right"));
      if (cl) out.ctrl.left = parseSide(*cl);
      if (cr) out.ctrl.right = parseSide(*cr);
      out.ctrl.output_valid = out.ctrl.left.valid && out.ctrl.right.valid;
    }

    const JsonMap* hmd = asMap(find(m, "hmd"));
    if (hmd) {
      const std::string quality = asString(find(*hmd, "quality"));
      out.hmd_live = (quality == "live");
      const JsonMap* pose = asMap(find(*hmd, "pose"));
      if (pose) out.hmd_pose = parsePose(*pose);
    }

    // 可选：controllers（原始姿态，暂供调试用，这里不展开消费）
    // const JsonMap* ctrls = asMap(find(m, "controllers")); ...

    const JsonMap* rc = asMap(find(m, "robot_control"));
    if (rc) {
      PicoRobotControl& r = out.robot;
      r.model = asString(find(*rc, "model"));
      const JsonMap* arms = asMap(find(*rc, "arms"));
      if (arms) {
        r.has_arms = true;
        parseVec(find(*arms, "left"), r.left_arm);
        parseVec(find(*arms, "right"), r.right_arm);
      }
      const JsonMap* hands = asMap(find(*rc, "hands"));
      if (hands) {
        r.has_hands = true;
        parseVec(find(*hands, "left"), r.left_hand);
        parseVec(find(*hands, "right"), r.right_hand);
      }
      const JsonMap* head = asMap(find(*rc, "head"));
      if (head) {
        r.has_head = true;
        r.head_yaw_deg = asNumber(find(*head, "yaw"));
        r.head_pitch_deg = asNumber(find(*head, "pitch"));
      }
      const JsonMap* body = asMap(find(*rc, "body"));
      if (body) {
        if (asMap(find(*body, "waist"))) r.has_waist = true;
        if (asMap(find(*body, "lift"))) r.has_lift = true;
      }
      const JsonMap* base = asMap(find(*rc, "base"));
      if (base) {
        r.has_base = true;
        r.base_linear_x = asNumber(find(*base, "linear_x"));
        r.base_linear_y = asNumber(find(*base, "linear_y"));
        r.base_angular_z = asNumber(find(*base, "angular_z"));
      }
    }

    if (out.sequence == 0) return false;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[pico] JSON parse error: " << e.what() << std::endl;
    return false;
  }
}

// ---------------------------------------------------------------------------
// UDP 接收器（POSIX）
// ---------------------------------------------------------------------------
class PicoUdpReceiver {
 public:
  explicit PicoUdpReceiver(uint16_t port, const std::string& host = "0.0.0.0")
      : port_(port), host_(host), sock_(-1) {}

  ~PicoUdpReceiver() { close(); }

  PicoUdpReceiver(const PicoUdpReceiver&) = delete;
  PicoUdpReceiver& operator=(const PicoUdpReceiver&) = delete;

  bool bind() {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      std::cerr << "[pico] socket() failed: " << strerror(errno) << std::endl;
      return false;
    }
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (host_ != "0.0.0.0") {
      if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "[pico] invalid bind host: " << host_ << std::endl;
        close();
        return false;
      }
    }
    if (::bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      std::cerr << "[pico] bind udp://" << host_ << ":" << port_
                << " failed: " << strerror(errno) << std::endl;
      close();
      return false;
    }
    std::cout << "[pico] UDP receiver listening on " << host_ << ":" << port_ << std::endl;
    return true;
  }

  /// 阻塞最多 timeout_ms 毫秒接收并解析一个包。超时返回 false（不解析错误日志）。
  bool recv(PicoTeleopPacket& out, int timeout_ms = 1000) {
    if (sock_ < 0) return false;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock_, &fds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int r = select(sock_ + 1, &fds, nullptr, nullptr, &tv);
    if (r <= 0) return false;  // 超时/信号中断

    char buf[128 * 1024];
    const ssize_t n = ::recvfrom(sock_, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
    if (n <= 0) return false;
    buf[n] = '\0';

    PicoTeleopPacket pkt;
    if (!parsePicoPacket(std::string(buf, n), pkt)) return false;
    out = pkt;
    return true;
  }

  void close() {
    if (sock_ >= 0) {
      ::close(sock_);
      sock_ = -1;
    }
  }

 private:
  uint16_t port_;
  std::string host_;
  int sock_;
};

// ---------------------------------------------------------------------------
// 后台接收线程：只保留"最新一包"，供控制循环随时取用。
// ---------------------------------------------------------------------------
class PicoUdpThread {
 public:
  PicoUdpThread(uint16_t port, const std::string& host = "0.0.0.0") : receiver_(port, host) {}

  bool start() {
    if (!receiver_.bind()) return false;
    run_ = true;
    thread_ = std::thread([this]() {
      while (run_.load()) {
        PicoTeleopPacket pkt;
        if (receiver_.recv(pkt, 500)) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = pkt;
          last_rx_ms_ = nowMs();
        }
      }
    });
    return true;
  }

  void stop() {
    run_ = false;
    if (thread_.joinable()) thread_.join();
    receiver_.close();
  }

  /// 取最新包 + 距上次收包时间（ms）。没有收到过任何包返回 false。
  bool tryGet(PicoTeleopPacket& out, double& rx_age_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_.hasData()) return false;
    out = latest_;
    rx_age_ms = nowMs() - last_rx_ms_;
    return true;
  }

 private:
  static double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
  }

  PicoUdpReceiver receiver_;
  std::thread thread_;
  std::atomic<bool> run_{false};
  std::mutex mutex_;
  PicoTeleopPacket latest_;
  double last_rx_ms_ = 0.0;
};

}  // namespace pico
}  // namespace r1skeleton
