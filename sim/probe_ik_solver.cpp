// ============================================================================
// probe_ik_solver.cpp
// 诊断 R1DualArmIk 在「手往下放」时的表现，定位「肘一直弯着、手放不下去」的成因。
//
// 背景：
//   可达点云（sim/probe_reach.py 的采样）显示左腕位置在工作空间内是可以放到
//   z = −0.30 m（腰部系）的，且几何上肘角应随目标下降**增大**（−42° → +86°，
//   手臂越来越伸展）。但完整管线实测把肘钉在 −55.9°（URDF 下限）不动。
//
// 待验假设：
//   A5 只有 5 个关节，而 IK 同时约束位置(3) + 姿态(3) = 6 个量 → 超定。
//   当目标姿态在该位置不可行时，可用的自由度被拿去迁就姿态，位置就做不好。
//
// 本程序只调用 r1_arm_ik.h（仅依赖 Eigen），可 macOS 本机编译，不是链路的一部分。
//
// 构建/运行：见 sim/README.md
// ============================================================================

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
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

const double X = 0.25;   // 左腕目标 x（腰部系）
const double Y = 0.15;   // 左腕目标 y
const int N = 5;

double posErrMm(const R1ArmKinematics& kin, const VectorXd& q, const Isometry3d& T) {
  return (kin.forward(q, true).translation() - T.translation()).norm() * 1000.0;
}
double rotErrDeg(const R1ArmKinematics& kin, const VectorXd& q, const Isometry3d& T) {
  const Matrix3d dR = kin.forward(q, true).linear() * T.linear().transpose();
  return std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) / 2.0))) * 180.0 / M_PI;
}
double deg(double r) { return r * 180.0 / M_PI; }

std::string hitLimit(const R1ArmKinematics& kin, double q) {
  const double lo = kin.chain(true)[3].lower, hi = kin.chain(true)[3].upper;
  if (q - lo < 1e-3) return "下限";
  if (hi - q < 1e-3) return "上限";
  return "    ";
}

/// 复刻 R1DualArmIk::solveArm 的 DLS 逻辑，但把姿态权重变成可调参数。
/// 仅用于诊断「把 c_rot 降到多少，位置才跟得上」——不参与构建。
VectorXd solveArmW(const R1ArmKinematics& kin, const Isometry3d& target, const VectorXd& q0,
                   double w_pos, double w_rot, double lambda = 1e-3, int max_iter = 15) {
  const int n = kin.dofPerArm();
  VectorXd q = q0;
  kin.clamp(q, true);
  const Vector3d p_t = target.translation();
  const Matrix3d R_t = target.linear();
  for (int iter = 0; iter < max_iter; ++iter) {
    const Isometry3d T = kin.forward(q, true);
    const Vector3d e_pos = p_t - T.translation();
    const Eigen::AngleAxisd aa(T.linear() * R_t.transpose());
    const Vector3d e_rot = aa.axis() * aa.angle();
    VectorXd e(6);
    e << std::sqrt(w_pos) * e_pos, -std::sqrt(w_rot) * e_rot;
    if (e.norm() < 1e-4) break;

    MatrixXd J = kin.jacobian(q, true);
    const double sw_pos = std::sqrt(w_pos), sw_rot = std::sqrt(w_rot);
    for (int r = 0; r < 3; ++r) J.row(r) *= sw_pos;
    for (int r = 3; r < 6; ++r) J.row(r) *= sw_rot;
    const MatrixXd A = J.transpose() * J + lambda * MatrixXd::Identity(n, n);
    VectorXd dq = A.ldlt().solve(J.transpose() * e);
    if (!dq.allFinite()) break;
    const double step = dq.norm();
    if (step > 0.3) dq *= 0.3 / step;
    q += dq;
    kin.clamp(q, true);
    if ((p_t - kin.forward(q, true).translation()).norm() < 5e-4) break;
  }
  return q;
}

}  // namespace

int main() {
  R1DualArmIk ik(R1ArmKinematics::R1_A5);
  const R1ArmKinematics& kin = ik.kinematics();

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "左腕目标 x=" << X << " y=" << Y << "（腰部系）。肘关节限位 "
            << std::setprecision(1) << deg(kin.chain(true)[3].lower) << "° .. "
            << deg(kin.chain(true)[3].upper) << "°\n";
  std::cout << std::setprecision(2);

  // ---------------------------------------------------------------------
  // 实验 1：位置追踪对照 —— 「姿态自由」把姿态这个自由度让出来，
  //         每帧用上一帧解出的可达姿态作目标；「姿态固定」则钉死单位朝向。
  //         两者都用 warm start = 上一帧解（与真实链路一致），目标 z 由高到低。
  // ---------------------------------------------------------------------
  std::cout << "\n[实验 1] 位置追踪：目标 z 由高到低连续下降（warm start 接力）\n";
  std::cout << "  目标z  | 姿态自由（让出姿态）        | 姿态固定=单位朝向\n";
  std::cout << "         | 位置误差mm  肘角°  顶限位    | 位置误差mm  肘角°  顶限位\n";
  std::cout << std::string(80, '-') << "\n";

  VectorXd q_free = VectorXd::Zero(N), q_fix = VectorXd::Zero(N);
  for (double z = 0.60; z > -0.46; z -= 0.05) {
    Isometry3d T;
    T.translation() = Vector3d(X, Y, z);

    T.linear() = kin.forward(q_free, true).linear();   // 姿态自由：沿用当前可达姿态
    q_free = ik.solveArm(T, q_free, true);
    const double ef = posErrMm(kin, q_free, T);

    T.linear() = Matrix3d::Identity();                 // 姿态固定：目标朝向 = 单位阵
    q_fix = ik.solveArm(T, q_fix, true);
    const double ex = posErrMm(kin, q_fix, T);

    std::cout << std::setw(8) << z << " | " << std::setw(9) << ef << " " << std::setw(8)
              << deg(q_free[3]) << "  " << hitLimit(kin, q_free[3]) << "    | " << std::setw(9) << ex
              << " " << std::setw(8) << deg(q_fix[3]) << "  " << hitLimit(kin, q_fix[3]) << "\n";
  }

  // ---------------------------------------------------------------------
  // 实验 2：随机初值扫描 —— 每个目标点独立地从 300 个随机初值求解，
  //         看位置误差的**最小值**。若最小值都很大，说明该点在该姿态约束下
  //         根本不可精确到达（而不是求解器没找对）。
  // ---------------------------------------------------------------------
  std::cout << "\n[实验 2] 每点 300 个随机初值，看位置误差能达到的最小值\n";
  std::cout << "  目标z | 位置误差 min / 中位 / max (mm)   | 肘角范围(°)        | 姿态残差中位(°)\n";
  std::cout << std::string(84, '-') << "\n";

  std::mt19937 rng(0);
  for (double z : {0.40, 0.30, 0.20, 0.10, 0.00, -0.10, -0.20, -0.28}) {
    Isometry3d T = Isometry3d::Identity();
    T.translation() = Vector3d(X, Y, z);

    std::vector<double> errs, elbows, rots;
    for (int k = 0; k < 300; ++k) {
      VectorXd q0(N);
      for (int i = 0; i < N; ++i) {
        const double lo = kin.chain(true)[i].lower, hi = kin.chain(true)[i].upper;
        q0[i] = lo + (hi - lo) * (rng() / double(std::mt19937::max()));
      }
      const VectorXd q = ik.solveArm(T, q0, true);
      errs.push_back(posErrMm(kin, q, T));
      elbows.push_back(deg(q[3]));
      rots.push_back(rotErrDeg(kin, q, T));
    }
    std::sort(errs.begin(), errs.end());
    std::sort(elbows.begin(), elbows.end());
    std::sort(rots.begin(), rots.end());

    std::cout << std::setw(8) << z << " | " << std::setw(9) << errs.front() << " / " << std::setw(9)
              << errs[errs.size() / 2] << " / " << std::setw(9) << errs.back() << " | "
              << std::setw(8) << elbows.front() << " .. " << std::setw(8) << elbows.back() << " | "
              << std::setw(9) << rots[rots.size() / 2] << "\n";
  }

  // ---------------------------------------------------------------------
  // 实验 3：姿态代价 —— 同一位置，把目标姿态逐步从「单位朝向」转到「手朝下」，
  //         看位置误差怎么变。用于判断"姿态不匹配"是不是位置精度损失的主因。
  // ---------------------------------------------------------------------
  std::cout << "\n[实验 3] z=0.00 处，目标姿态绕 X 轴逐步旋转，看位置误差代价\n";
  std::cout << "  目标姿态 | 位置误差mm | 肘角°   | 实际姿态残差°\n";
  std::cout << std::string(58, '-') << "\n";
  Isometry3d T3;
  T3.translation() = Vector3d(X, Y, 0.0);
  for (int a = 0; a <= 90; a += 15) {
    T3.linear() = Eigen::AngleAxisd(a * M_PI / 180.0, Vector3d::UnitX()).toRotationMatrix();
    const VectorXd q = ik.solveArm(T3, VectorXd::Zero(N), true);
    std::cout << std::setw(8) << a << "° | " << std::setw(10) << posErrMm(kin, q, T3) << " | "
              << std::setw(7) << deg(q[3]) << " | " << std::setw(15) << rotErrDeg(kin, q, T3) << "\n";
  }

  // ---------------------------------------------------------------------
  // 实验 4：姿态权重的代价 —— 目标姿态钉死为最坏情况（单位朝向），
  //         逐步降低 w_rot（官方 A5 = 0.5），看位置误差何时全线收敛。
  //         给出「该调到多少」的定量依据。
  // ---------------------------------------------------------------------
  std::cout << "\n[实验 4] 降低姿态权重 w_rot 后的位置误差（姿态固定=单位朝向，warm start 接力）\n";
  std::cout << "  w_rot |     z=+0.30   z=+0.10   z=+0.00   z=-0.20   z=-0.35  | 全程最大误差\n";
  std::cout << std::string(88, '-') << "\n";
  const std::vector<double> probe_z = {0.30, 0.10, 0.00, -0.20, -0.35};
  for (double wr : {0.5, 0.2, 0.1, 0.05, 0.02, 0.0}) {
    VectorXd q = VectorXd::Zero(N);
    double worst = 0.0;
    std::vector<double> picked;
    for (double z = 0.60; z > -0.46; z -= 0.02) {
      Isometry3d T = Isometry3d::Identity();
      T.translation() = Vector3d(X, Y, z);
      q = solveArmW(kin, T, q, 50.0, wr);
      const double e = posErrMm(kin, q, T);
      worst = std::max(worst, e);
      for (size_t i = 0; i < probe_z.size(); ++i) {
        if (std::abs(z - probe_z[i]) < 1e-9) picked.push_back(e);
      }
    }
    std::cout << std::setw(7) << wr << " |";
    for (double e : picked) std::cout << std::setw(10) << e;
    std::cout << "  |" << std::setw(10) << worst << "\n";
  }
  return 0;
}
