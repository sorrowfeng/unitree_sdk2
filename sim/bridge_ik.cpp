// ============================================================================
// bridge_ik.cpp
// 把 r1_arm_ik.h 的 FK / 单臂 IK 暴露成一个行协议，供 Python 侧（官方 pinocchio +
// casadi 求解器）驱动，用来做「我方实现 vs 官方实现」的同输入对照。
//
// 行协议（stdin -> stdout，每行一问一答）：
//   FK <variant> <q0..q_{2n-1}>
//        -> OK <左: x y z qx qy qz qw> <右: x y z qx qy qz qw>
//   IKL <variant> <tx> <ty> <tz> <q0..q_{n-1}>
//        -> OK <q0..q_{n-1}> <位置误差mm> <姿态残差deg>
//        目标姿态 = 单位阵；warm start = 给的 q
//   IKL2 <variant> <tx> <ty> <tz> <r00..r22> <q0..q_{n-1}>
//        -> 同上，但目标姿态由 3x3 旋转矩阵给定（行主序）
//
// 编译：clang++ -std=c++17 -O2 -I sim/build/inc -I example/r1/high_level sim/bridge_ik.cpp -o sim/build/bridge_ik
// ============================================================================

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "r1_arm_ik.h"

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using r1skeleton::R1ArmKinematics;
using r1skeleton::R1DualArmIk;

namespace {

constexpr double kDeg = 180.0 / M_PI;

R1ArmKinematics::Variant parseVariant(const std::string& s) {
  return (s == "a7") ? R1ArmKinematics::R1_A7 : R1ArmKinematics::R1_A5;
}

void printQuat(const Matrix3d& R, std::ostream& os) {
  Eigen::Quaterniond q(R);
  q.normalize();
  os << ' ' << std::setprecision(10) << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w();
}

}  // namespace

int main() {
  std::string line;
  std::cout << std::setprecision(10);
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::istringstream is(line);
    std::string cmd;
    is >> cmd;

    if (cmd == "FK") {
      std::string v;
      is >> v;
      R1ArmKinematics kin(parseVariant(v));
      const int n = kin.dofPerArm();
      VectorXd q(2 * n);
      for (int i = 0; i < 2 * n; ++i) {
        if (!(is >> q[i])) { std::cout << "ERR fk-parse\n"; break; }
      }
      const Isometry3d L = kin.forward(q.head(n), true);
      const Isometry3d R = kin.forward(q.tail(n), false);
      std::cout << "OK " << L.translation().x() << ' ' << L.translation().y() << ' '
                << L.translation().z();
      printQuat(L.linear(), std::cout);
      std::cout << ' ' << R.translation().x() << ' ' << R.translation().y() << ' '
                << R.translation().z();
      printQuat(R.linear(), std::cout);
      std::cout << '\n';
    } else if (cmd == "IKL" || cmd == "IKL2") {
      std::string v;
      is >> v;
      R1ArmKinematics kin(parseVariant(v));
      const int n = kin.dofPerArm();
      double tx = 0, ty = 0, tz = 0;
      is >> tx >> ty >> tz;
      Matrix3d Rt = Matrix3d::Identity();
      if (cmd == "IKL2") {
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c) is >> Rt(r, c);
      }
      VectorXd q(n);
      for (int i = 0; i < n; ++i) {
        if (!(is >> q[i])) { std::cout << "ERR ik-parse\n"; break; }
      }
      Isometry3d T = Isometry3d::Identity();
      T.translation() = Vector3d(tx, ty, tz);
      T.linear() = Rt;

      R1DualArmIk ik(parseVariant(v));
      const VectorXd qs = ik.solveArm(T, q, true);
      const Isometry3d Tf = kin.forward(qs, true);
      const double pos_err = (Tf.translation() - T.translation()).norm() * 1000.0;
      const Matrix3d dR = Tf.linear() * Rt.transpose();
      const double rot_err =
          std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0))) * kDeg;
      std::cout << "OK";
      for (int i = 0; i < n; ++i) std::cout << ' ' << qs[i];
      std::cout << ' ' << pos_err << ' ' << rot_err << '\n';
    } else if (cmd == "IKEOBJ") {
      // 忠实最小化官方目标函数的 DLS 变体，供参数扫描实验（不参与构建链路）：
      //   r = [ √w_pos (p_t − p) ; −√w_rot e_rot ; √w_reg q ; √w_smooth (q − q_last) ]
      //   J 同步扩成 (6+2n) × n
      // 用法：IKEOBJ <variant> <tx> <ty> <tz> <w_pos> <w_rot> <w_reg> <w_smooth>
      //        <max_iter> <lambda> <q_last...n> <q0...n>
      std::string v;
      is >> v;
      R1ArmKinematics kin(parseVariant(v));
      const int n = kin.dofPerArm();
      double tx, ty, tz, w_pos, w_rot, w_reg, w_smooth, lam;
      int max_iter;
      is >> tx >> ty >> tz >> w_pos >> w_rot >> w_reg >> w_smooth >> max_iter >> lam;
      VectorXd q_last(n), q(n);
      for (int i = 0; i < n; ++i) is >> q_last[i];
      for (int i = 0; i < n; ++i) is >> q[i];
      kin.clamp(q, true);

      const Vector3d p_t(tx, ty, tz);
      const Matrix3d R_t = Matrix3d::Identity();
      const double swp = std::sqrt(w_pos), swr = std::sqrt(w_rot);
      const double swg = std::sqrt(w_reg), sws = std::sqrt(w_smooth);

      for (int iter = 0; iter < max_iter; ++iter) {
        const Isometry3d T = kin.forward(q, true);
        const Vector3d e_pos = p_t - T.translation();
        const Eigen::AngleAxisd aa(T.linear() * R_t.transpose());
        const Vector3d e_rot = aa.axis() * aa.angle();

        VectorXd r(6 + 2 * n);
        r << swp * e_pos, -swr * e_rot, swg * q, sws * (q - q_last);
        if (r.norm() < 1e-6) break;

        MatrixXd J = kin.jacobian(q, true);           // 6 x n
        MatrixXd Jw(6 + 2 * n, n);
        Jw.topRows(6) = J;
        for (int i = 0; i < 3; ++i) Jw.row(i) *= swp;
        for (int i = 3; i < 6; ++i) Jw.row(i) *= swr;
        Jw.block(n, 0, n, n).setZero();
        Jw.block(6, 0, n, n).setZero();
        for (int i = 0; i < n; ++i) {
          // 正则/平滑项的残差是 +swg*q、+sws*(q−q_last)，其雅可比 ∂r/∂q = +sw·I，
          // 解 A dq = −r 后写成 J' dq = r 的形式得 J' = −A = −sw·I（负号不能漏，
          // 写正号会把解推向关节限位）。
          Jw(6 + i, i) = -swg;
          Jw(6 + n + i, i) = -sws;
        }
        // 最小化 ||Jw dq − r||²；注意 Jw 各行已含权重
        // 但位置/姿态行的 J 需要乘权重（上面已乘），单位阵块已带权重
        VectorXd dq = (Jw.transpose() * Jw + lam * MatrixXd::Identity(n, n))
                          .ldlt()
                          .solve(Jw.transpose() * r);
        if (!dq.allFinite()) break;
        const double step = dq.norm();
        if (step > 0.3) dq *= 0.3 / step;
        q += dq;
        kin.clamp(q, true);
      }
      const Isometry3d Tf = kin.forward(q, true);
      const double pos_err = (Tf.translation() - p_t).norm() * 1000.0;
      const Matrix3d dR = Tf.linear() * R_t.transpose();
      const double rot_err =
          std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0))) * kDeg;
      std::cout << "OK";
      for (int i = 0; i < n; ++i) std::cout << ' ' << q[i];
      std::cout << ' ' << pos_err << ' ' << rot_err << '\n';
    } else {
      std::cout << "ERR unknown-cmd\n";
    }
    std::cout.flush();
  }
  return 0;
}
