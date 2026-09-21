// ============================================================================
// ik_probe.cpp —— 双臂运动学 / IK 探针（仅依赖 Eigen，可在 macOS 本机编译）
//
// 目的：把 example/r1/high_level/r1_arm_ik.h 的 FK 与 IK 暴露成行式文本接口，
//       交给 sim/eval_ik.py 用 MuJoCo 加载同一份 URDF 做**独立**正运动学校验，
//       避免"用自家 FK 验证自家 IK"的循环论证。
//
// 用法：
//   ik_probe --variant a5|a7 --mode fk|ik|ik-raw
//
// 输入（stdin，逐行；以 # 开头或空行忽略；数值以空格分隔）：
//   fk     : q[2n]                                    n = 5(A5) / 7(A7)
//   ik     : L(p3 q4) R(p3 q4) qcur[2n]   → 走 R1DualArmIk::solve（含 WMA 平滑）
//   ik-raw : L(p3 q4) R(p3 q4) qcur[2n]   → 走 R1DualArmIk::solveArm（无平滑，逐臂）
//
// 输出（stdout，每行对应一行输入，精度 12 位）：
//   fk     : L(px py pz qx qy qz qw) R(px py pz qx qy qz qw)
//   ik*    : q[2n]
//
// 坐标约定：骨盆系（= URDF 的 waist_yaw_link 系，waist_yaw 锁 0，与官方一致）；
//           EE 已含官方手安装偏移（A5 +0.20m x，A7 +0.05m x）；四元数顺序 x y z w。
// ============================================================================

#include "r1_arm_ik.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using r1skeleton::Iso;
using r1skeleton::R1ArmKinematics;
using r1skeleton::R1DualArmIk;

static void printIso(const Iso& T) {
  const Eigen::Quaterniond q(T.linear());
  std::cout << T.translation().x() << ' ' << T.translation().y() << ' '
            << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z()
            << ' ' << q.w();
}

static Iso isoFrom(const std::vector<double>& v, int base) {
  Eigen::Quaterniond q(v[base + 6], v[base + 3], v[base + 4], v[base + 5]);  // w,x,y,z
  q.normalize();
  Iso T = Iso::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(v[base + 0], v[base + 1], v[base + 2]);
  return T;
}

int main(int argc, char** argv) {
  std::string variant = "a5";
  std::string mode = "ik";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--variant" && i + 1 < argc) variant = argv[++i];
    else if (a == "--mode" && i + 1 < argc) mode = argv[++i];
    else {
      std::cerr << "用法: ik_probe --variant a5|a7 --mode fk|ik|ik-raw\n";
      return 2;
    }
  }
  if (variant != "a5" && variant != "a7") {
    std::cerr << "[ik_probe] --variant 只支持 a5 / a7\n";
    return 2;
  }

  const R1ArmKinematics::Variant v =
      (variant == "a7") ? R1ArmKinematics::R1_A7 : R1ArmKinematics::R1_A5;
  R1ArmKinematics kin(v);
  R1DualArmIk ik(v);
  const int n = kin.dofPerArm();

  std::cout << std::setprecision(12);
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::vector<double> val;
    double x = 0.0;
    while (ss >> x) val.push_back(x);

    if (mode == "fk") {
      if (static_cast<int>(val.size()) != 2 * n) {
        std::cerr << "[ik_probe] fk 需要 " << 2 * n << " 个数，收到 " << val.size()
                  << "\n";
        return 3;
      }
      Eigen::VectorXd qL(n), qR(n);
      for (int i = 0; i < n; ++i) qL[i] = val[i];
      for (int i = 0; i < n; ++i) qR[i] = val[n + i];
      printIso(kin.forward(qL, true));
      std::cout << ' ';
      printIso(kin.forward(qR, false));
      std::cout << '\n';
    } else {
      if (static_cast<int>(val.size()) != 14 + 2 * n) {
        std::cerr << "[ik_probe] " << mode << " 需要 " << 14 + 2 * n
                  << " 个数，收到 " << val.size() << "\n";
        return 3;
      }
      const Iso L = isoFrom(val, 0);
      const Iso R = isoFrom(val, 7);
      Eigen::VectorXd qcur(2 * n);
      for (int i = 0; i < 2 * n; ++i) qcur[i] = val[14 + i];

      Eigen::VectorXd q;
      if (mode == "ik-raw") {
        Eigen::VectorXd qL = ik.solveArm(L, qcur.head(n), true);
        Eigen::VectorXd qR = ik.solveArm(R, qcur.tail(n), false);
        q.resize(2 * n);
        q << qL, qR;
      } else {
        q = ik.solve(L, R, qcur);
      }
      for (int i = 0; i < 2 * n; ++i) {
        std::cout << q[i] << (i + 1 == 2 * n ? '\n' : ' ');
      }
    }
  }
  return 0;
}
