// 不接机器人的解析/对齐冒烟测试：验证 r1_pico_udp.h + r1_xr_pose_alignment.h
//
// 编译运行见同目录上级 scripts/run_tests.sh，或：
//   g++ -std=c++17 -O2 \
//     -I<repo>/include -I<repo>/thirdparty/include \
//     -I<repo>/thirdparty/include/ddscxx -I<repo>/thirdparty/include/ddsc \
//     -I<repo>/example/r1/high_level \
//     test_pico_parse.cpp -o /tmp/test_pico_parse \
//     -L<repo>/thirdparty/lib/$(uname -m) -Wl,-rpath,<repo>/thirdparty/lib/$(uname -m) \
//     <repo>/lib/$(uname -m)/libunitree_sdk2.a -lddsc -lddscxx -lpthread -ldl
#include <iostream>

#include "r1_pico_udp.h"
#include "r1_pico_safety_policy.h"
#include "r1_xr_pose_alignment.h"

using namespace r1skeleton;

static int g_fail = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (cond) {                                                           \
      std::cout << "[ OK ] " << #cond << "\n";                            \
    } else {                                                              \
      std::cout << "[FAIL] " << #cond << "\n";                            \
      ++g_fail;                                                           \
    }                                                                     \
  } while (0)

int main() {
  const char* js_active = R"JSON({
    "sequence": 42,
    "timestamp": 123456.7,
    "operator_mode": "active_stream",
    "sdk": "PICOHandLink",
    "safety": { "safe_to_execute": true, "tracking_valid": true,
                "emergency_stop_latched": false, "sample_age_ms": 12.0,
                "max_sample_age_ms": 250.0 },
    "teleop": {
      "output_valid": true,
      "left":  { "quality": "live",
                 "pose": { "position": {"x":0.10,"y":1.20,"z":-0.30},
                           "orientation": {"pitch":10,"yaw":20,"roll":5},
                           "orientation_quat": {"x":0.0,"y":0.0,"z":0.087,"w":0.996} } },
      "right": { "quality": "live",
                 "pose": { "position": {"x":-0.10,"y":1.20,"z":-0.30},
                           "orientation": {"pitch":-10,"yaw":-20,"roll":-5} } }
    },
    "controllers": {
      "left":  { "quality": "live",
                 "pose": { "position": {"x":0.20,"y":1.30,"z":-0.40},
                           "orientation": {"pitch":0,"yaw":0,"roll":0} } },
      "right": { "quality": "live",
                 "pose": { "position": {"x":-0.20,"y":1.30,"z":-0.40},
                           "orientation": {"pitch":0,"yaw":0,"roll":0} } }
    },
    "hmd": { "quality": "live",
             "pose": { "position": {"x":0.0,"y":1.60,"z":0.10},
                       "orientation": {"pitch":0,"yaw":0,"roll":0} } },
    "robot_control": {
      "model": "f1",
      "hands": { "left":  [0,1000,2000,3000,4000,5000],
                 "right": [6000,7000,8000,9000,10000,500] },
      "base":  { "linear_x": 0.10, "linear_y": 0.00, "angular_z": 0.20 }
    }
  })JSON";

  std::cout << "=== 1) active_stream 完整包 ===\n";
  pico::PicoTeleopPacket pkt;
  CHECK(pico::parsePicoPacket(js_active, pkt));
  CHECK(pkt.sequence == 42);
  CHECK(pkt.operator_mode == "active_stream");
  CHECK(pkt.safeToExecute() == true);
  CHECK(pkt.safety.tracking_valid == true);
  CHECK(pkt.teleop_valid == true);
  CHECK(pkt.ctrl.output_valid == true);
  CHECK(pkt.hmd_live == true);
  CHECK(pkt.robot.has_hands == true);
  CHECK(pkt.robot.left_hand.size() == 6);
  CHECK(pkt.robot.left_hand[5] == 5000.0);
  CHECK(pkt.robot.right_hand[0] == 6000.0);
  CHECK(pkt.robot.has_base == true);
  CHECK(pkt.robot.base_linear_x == 0.10);
  CHECK(pkt.robot.base_angular_z == 0.20);

  // 四元数优先路径
  CHECK(pkt.left.pose.has_quat == true);
  Eigen::Isometry3d Lquat = pkt.left.pose.toOpenXrPose();
  CHECK(std::abs(Lquat.translation().x() - 0.10) < 1e-9);

  // 无四元数 -> 欧拉回退路径
  CHECK(pkt.right.pose.has_quat == false);
  Eigen::Isometry3d Reuler = pkt.right.pose.toOpenXrPose();
  CHECK(std::abs(Reuler.translation().y() - 1.20) < 1e-9);

  // 对齐：head_yaw / head_trans 参考系
  Eigen::Isometry3d head = pkt.hmd_pose.toOpenXrPose();
  Eigen::Isometry3d Lrobot =
      xralign::alignWristToRobot(Lquat, head, xralign::RefMode::kHeadYaw);
  std::cout << "  Lrobot pos = " << Lrobot.translation().transpose() << "\n";
  CHECK(Lrobot.matrix().allFinite());
  Eigen::Isometry3d Rrobot =
      xralign::alignWristToRobot(Reuler, head, xralign::RefMode::kHeadTranslation);
  CHECK(Rrobot.matrix().allFinite());

  std::cout << "\n=== 2) safe_to_execute=false（不可执行）===\n";
  std::string js = js_active;
  const std::string key = "\"safe_to_execute\": true";
  js.replace(js.find(key), key.size(), "\"safe_to_execute\": false");
  pico::PicoTeleopPacket pkt2;
  CHECK(pico::parsePicoPacket(js, pkt2));
  CHECK(pkt2.safeToExecute() == false);

  std::cout << "\n=== 3) stop_signal 包（不可执行）===\n";
  std::string js3 = js_active;
  const std::string mode = "\"active_stream\"";
  js3.replace(js3.find(mode), mode.size(), "\"stop_signal\"");
  pico::PicoTeleopPacket pkt3;
  CHECK(pico::parsePicoPacket(js3, pkt3));
  CHECK(pkt3.operator_mode == "stop_signal");
  CHECK(pkt3.safeToExecute() == false);

  std::cout << "\n=== 4) 急停闩锁（须不可执行，即使 safe_to_execute=true）===\n";
  const std::string estop_key = "\"emergency_stop_latched\": false";
  std::string js5 = js_active;
  js5.replace(js5.find(estop_key), estop_key.size(),
              "\"emergency_stop_latched\": true");
  pico::PicoTeleopPacket pkt5;
  CHECK(pico::parsePicoPacket(js5, pkt5));
  CHECK(pkt5.emergencyStopLatched() == true);
  CHECK(pkt5.safety.safe_to_execute == true);  // 两字段独立，PICO 端可能只置其一
  CHECK(pkt5.safeToExecute() == false);        // 修复前此处会误判为 true
  CHECK(pkt5.returnZeroRequested() == false);  // 急停优先级高于回零

  std::cout << "\n=== 5) return_zero（回零请求成立，但不等于遥操许可）===\n";
  std::string js6 = js_active;
  js6.replace(js6.find(mode), mode.size(), "\"return_zero\"");
  pico::PicoTeleopPacket pkt6;
  CHECK(pico::parsePicoPacket(js6, pkt6));
  CHECK(pkt6.operator_mode == "return_zero");
  CHECK(pkt6.safeToExecute() == false);       // 非 active_stream
  CHECK(pkt6.returnZeroRequested() == true);  // 修复前被 !safe 短路，分支不可达

  std::cout << "\n=== 6) return_zero + 急停闩锁（急停优先）===\n";
  std::string js7 = js6;
  js7.replace(js7.find(estop_key), estop_key.size(),
              "\"emergency_stop_latched\": true");
  pico::PicoTeleopPacket pkt7;
  CHECK(pico::parsePicoPacket(js7, pkt7));
  CHECK(pkt7.returnZeroRequested() == false);
  CHECK(pkt7.safeToExecute() == false);

  std::cout << "\n=== 7) 判据优先级（decideDisposition：急停 > 回零 > 保持 > 遥操）===\n";
  using pico::Disposition;
  // 正常遥操：三条件齐备
  CHECK(pico::decideDisposition(pkt, 10.0, true) == Disposition::kTeleop);
  // 掉包（>600ms）与来源无效 -> 保持
  CHECK(pico::decideDisposition(pkt, 700.0, true) == Disposition::kHold);
  CHECK(pico::decideDisposition(pkt, 10.0, false) == Disposition::kHold);
  // 回零：不要求 active_stream，也不要求 src_valid（目标为零位）
  CHECK(pico::decideDisposition(pkt6, 10.0, false) == Disposition::kReturnZero);
  // 陈旧的回零包不再执行
  CHECK(pico::decideDisposition(pkt6, 700.0, true) == Disposition::kHold);
  // 回零 + 急停：急停优先
  CHECK(pico::decideDisposition(pkt7, 10.0, false) == Disposition::kEmergencyStop);
  // 急停：包陈旧 / 来源无效 / safe_to_execute=true 都不影响
  CHECK(pico::decideDisposition(pkt5, 700.0, false) == Disposition::kEmergencyStop);
  // 阻尼触发条件
  CHECK(pico::holdTriggersDamp(pkt6, 10.0) == false);  // 回零包：非 stop_signal
  CHECK(pico::holdTriggersDamp(pkt2, 10.0) == false);  // safe=false：只停移动
  CHECK(pico::holdTriggersDamp(pkt, 700.0) == true);   // 掉包 -> 阻尼
  CHECK(pico::holdTriggersDamp(pkt3, 10.0) == true);   // stop_signal -> 阻尼

  std::cout << "\n=== 8) 畸形 JSON / sequence=0（必须拒绝）===\n";
  pico::PicoTeleopPacket pkt4;
  CHECK(pico::parsePicoPacket("{not json", pkt4) == false);
  CHECK(pico::parsePicoPacket("{\"sequence\":0}", pkt4) == false);

  std::cout << "\n" << (g_fail == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED")
            << " (failures=" << g_fail << ")\n";
  return g_fail == 0 ? 0 : 1;
}
