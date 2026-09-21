#pragma once
// ============================================================================
// r1_pico_safety_policy.h
// PICO 单帧报文的「处置决策」——把判据优先级集中到一处，可脱离 DDS 单测。
//
// 为什么单独抽出来：
//   原先各判据直接写在主循环的 if/else 链里，优先级靠语句顺序隐式表达，
//   一旦顺序写错（例如 return_zero 分支排在 `if (!safe) continue;` 之后），
//   整个分支变成死代码，而编译、单测都不会报错。抽成纯函数后可由
//   tests/test_pico_parse.cpp 直接覆盖优先级。
//
// 优先级（高 -> 低）：
//   1) 急停闩锁  kEmergencyStop  停移动 + 阻尼，双臂保持，不自动复位
//   2) 一键回零  kReturnZero     双臂目标归零，底盘停止
//   3) 保持      kHold           未安全 / 掉包 / 来源无效：停移动，双臂保持
//   4) 正常遥操  kTeleop         位姿 -> 对齐 -> IK -> 臂控
// ============================================================================

#include "r1_pico_udp.h"

namespace r1skeleton {
namespace pico {

/// 收包超过此时限视为掉包（与主程序 r1_dual_arm_loco.cpp 保持一致）。
constexpr double kDefaultStaleMs = 600.0;

/// 单帧报文的处置决策。
enum class Disposition {
  kEmergencyStop,
  kReturnZero,
  kHold,
  kTeleop,
};

inline const char* dispositionName(Disposition d) {
  switch (d) {
    case Disposition::kEmergencyStop: return "emergency_stop";
    case Disposition::kReturnZero:    return "return_zero";
    case Disposition::kHold:          return "hold";
    case Disposition::kTeleop:        return "teleop";
  }
  return "?";
}

/// 判据优先级：急停 > 回零 > 保持 > 遥操。
///
/// - 急停只看闩锁位：与包新鲜度、safe_to_execute、operator_mode 均无关，
///   因为「已闩锁」本身就是最高优先级的停止请求。
/// - 回零只要求包新鲜：回零期间 operator_mode 不是 active_stream，
///   若用 safeToExecute() 兜底则该分支永不可达（历史 bug）。
///   与 src_valid 无关——目标是零位，不依赖手柄位姿。
/// - 遥操要求三条件齐备：safe_to_execute（已含 !急停）+ 新鲜 + 来源有效。
inline Disposition decideDisposition(const PicoTeleopPacket& pkt,
                                     double rx_age_ms,
                                     bool src_valid,
                                     double stale_ms = kDefaultStaleMs) {
  if (pkt.emergencyStopLatched()) return Disposition::kEmergencyStop;

  const bool fresh = rx_age_ms < stale_ms;
  if (pkt.returnZeroRequested() && fresh) return Disposition::kReturnZero;

  if (pkt.safeToExecute() && fresh && src_valid) return Disposition::kTeleop;

  return Disposition::kHold;
}

/// kHold 是否应触发阻尼：掉包（不新鲜）或显式 stop_signal 包。
/// 其余 hold 场景（如 safe_to_execute=false、手柄无效）只停移动、保持站立。
inline bool holdTriggersDamp(const PicoTeleopPacket& pkt,
                             double rx_age_ms,
                             double stale_ms = kDefaultStaleMs) {
  return rx_age_ms >= stale_ms || pkt.operator_mode == "stop_signal";
}

}  // namespace pico
}  // namespace r1skeleton
