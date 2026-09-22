// 导出 r1_arm_ik.h 里实际使用的关节链参数，供与官方 URDF 逐项机检比对。
// 编译：clang++ -std=c++17 -O2 -I sim/build/inc -I example/r1/high_level sim/dump_chain.cpp -o sim/build/dump_chain
#include <cstdio>
#include <string>

#include "r1_arm_ik.h"

int main(int argc, char** argv) {
  const std::string which = (argc > 1) ? argv[1] : "a5";
  const auto v = (which == "a7") ? r1skeleton::R1ArmKinematics::R1_A7
                                 : r1skeleton::R1ArmKinematics::R1_A5;
  r1skeleton::R1ArmKinematics kin(v);
  const char* namesL[7] = {"shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
                           "elbow_joint", "wrist_roll_joint", "wrist_pitch_joint", "wrist_yaw_joint"};
  for (int side = 0; side < 2; ++side) {
    const bool left = (side == 0);
    const auto& ch = kin.chain(left);
    for (int i = 0; i < static_cast<int>(ch.size()); ++i) {
      const auto& j = ch[i];
      std::printf("%s|%s|%.9g %.9g %.9g|%.9g %.9g %.9g|%.9g %.9g %.9g|%.6f %.6f\n",
                  left ? "left" : "right", namesL[i],
                  j.origin.x(), j.origin.y(), j.origin.z(),
                  j.rpy.x(), j.rpy.y(), j.rpy.z(),
                  j.axis.x(), j.axis.y(), j.axis.z(),
                  j.lower, j.upper);
    }
  }
  return 0;
}
