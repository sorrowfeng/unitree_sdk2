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

  std::cout << "\n=== 4) 畸形 JSON / sequence=0（必须拒绝）===\n";
  pico::PicoTeleopPacket pkt4;
  CHECK(pico::parsePicoPacket("{not json", pkt4) == false);
  CHECK(pico::parsePicoPacket("{\"sequence\":0}", pkt4) == false);

  std::cout << "\n" << (g_fail == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED")
            << " (failures=" << g_fail << ")\n";
  return g_fail == 0 ? 0 : 1;
}
