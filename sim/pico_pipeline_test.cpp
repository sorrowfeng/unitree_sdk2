// ============================================================================
// pico_pipeline_test.cpp
// 本机 PICO 全链路测试：UDP JSON 报文 -> 解析 -> 安全判据 -> 对齐 -> IK -> 关节角。
//
// 与 r1_dual_arm_loco.cpp 的 --pico 分支同源：同一组头文件、同一套默认参数
// （--pico-source controllers / --pico-frame head_yaw），但不涉及 DDS 与机器人，
// 因此可以在 macOS 本机运行，用于在没有 PICO 与机器人的情况下验证整条链路。
//
// 用法（stdin 每行一个完整 JSON 报文）：
//   ./build/pico_pipeline_test --variant a5 --source controllers --frame head_yaw < packets.jsonl
//   ./build/pico_pipeline_test --variant a5 --raw < packets.jsonl    # 跳过 WMA 平滑
//
// 输出每行：
//   ok=<0|1> seq=<n> safe=<0|1> q=<2n 个关节角，rad>
// ok=0 表示报文被拒绝（畸形 JSON / sequence=0），此时不输出 q。
// safe=0 表示安全判据未通过（与主程序一致），此时臂目标不更新；
// ============================================================================

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "r1_arm_ik.h"
#include "r1_pico_udp.h"
#include "r1_xr_pose_alignment.h"

namespace {

std::string argValue(int argc, char** argv, const char* key, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) return argv[i + 1];
  }
  return def;
}

bool hasFlag(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], key) == 0) return true;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string variant = argValue(argc, argv, "--variant", "a5");
  const std::string source = argValue(argc, argv, "--source", "controllers");
  const std::string frame = argValue(argc, argv, "--frame", "head_yaw");
  const bool dump_pose = hasFlag(argc, argv, "--dump-pose");
  // --raw：跳过 WMA 输出平滑，得到"求解器裸精度"。
  // 默认走 ik.solve（含平滑），与 r1_dual_arm_loco.cpp 完全一致。
  const bool raw = hasFlag(argc, argv, "--raw");

  if (variant != "a5" && variant != "a7") {
    std::cerr << "[pipeline] unknown --variant: " << variant << " (expect a5|a7)\n";
    return 1;
  }
  if (source != "controllers" && source != "teleop") {
    std::cerr << "[pipeline] unknown --source: " << source << "\n";
    return 1;
  }

  const bool use_a7 = (variant == "a7");
  const int n = use_a7 ? 7 : 5;
  r1skeleton::R1DualArmIk ik(use_a7 ? r1skeleton::R1ArmKinematics::R1_A7
                                    : r1skeleton::R1ArmKinematics::R1_A5);
  r1skeleton::xralign::RefMode ref_mode = r1skeleton::xralign::RefMode::kHeadYaw;
  if (frame == "head_trans") ref_mode = r1skeleton::xralign::RefMode::kHeadTranslation;

  std::cerr << "[pipeline] variant=" << variant << " source=" << source
            << " frame=" << frame << " n=" << n << "\n";

  // warm start：与主程序一致，用上一次的解作为下一次 IK 的初值
  Eigen::VectorXd q = Eigen::VectorXd::Zero(2 * n);

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;

    r1skeleton::pico::PicoTeleopPacket pkt;
    if (!r1skeleton::pico::parsePicoPacket(line, pkt)) {
      std::cout << "ok=0 seq=0 safe=0\n";
      continue;
    }

    // 与 r1_dual_arm_loco.cpp 的判据保持一致
    const r1skeleton::pico::PicoSide& leftS =
        (source == "teleop") ? pkt.left : pkt.ctrl.left;
    const r1skeleton::pico::PicoSide& rightS =
        (source == "teleop") ? pkt.right : pkt.ctrl.right;
    const bool src_valid = (source == "teleop")
                               ? (pkt.teleop_valid && pkt.left.valid && pkt.right.valid)
                               : pkt.ctrl.output_valid;
    const bool safe = pkt.safeToExecute() && src_valid;

    std::ostringstream os;
    os << "ok=1 seq=" << pkt.sequence << " safe=" << (safe ? 1 : 0);
    if (safe) {
      if (pkt.operator_mode == "return_zero") {
        q.setZero();
      } else if (leftS.pose.valid && rightS.pose.valid) {
        const Eigen::Isometry3d Lxr = leftS.pose.toOpenXrPose();
        const Eigen::Isometry3d Rxr = rightS.pose.toOpenXrPose();
        const Eigen::Isometry3d head = pkt.hmd_live
                                           ? pkt.hmd_pose.toOpenXrPose()
                                           : r1skeleton::xralign::defaultHeadPose();
        Eigen::Isometry3d Lrobot, Rrobot;
        if (frame == "basis") {
          Lrobot = r1skeleton::xralign::basisToRobot(Lxr);
          Rrobot = r1skeleton::xralign::basisToRobot(Rxr);
        } else {
          Lrobot = r1skeleton::xralign::alignWristToRobot(Lxr, head, ref_mode);
          Rrobot = r1skeleton::xralign::alignWristToRobot(Rxr, head, ref_mode);
        }
        if (raw) {
          // 与 solve() 完全相同的两步单臂求解，但不经过 WMA 平滑，
          // 用于分离"求解器精度"与"平滑滞后"两种误差来源。
          q.head(n) = ik.solveArm(Lrobot, q.head(n), true);
          q.tail(n) = ik.solveArm(Rrobot, q.tail(n), false);
        } else {
          q = ik.solve(Lrobot, Rrobot, q);
        }
        if (dump_pose) {
          std::cerr << "[pose] L=" << Lrobot.translation().transpose()
                    << " R=" << Rrobot.translation().transpose() << "\n";
        }
      } else {
        os << " (pose_invalid)";
      }
      os << " q=";
      for (int i = 0; i < 2 * n; ++i) {
        os << (i ? " " : "") << q[i];
      }
    }
    std::cout << os.str() << "\n";
  }
  return 0;
}
