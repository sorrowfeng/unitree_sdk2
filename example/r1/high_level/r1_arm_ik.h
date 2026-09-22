#pragma once
// ============================================================================
// r1_arm_ik.h
// R1 双臂运动学 + 数值 IK（C++ 移植版）。
//
// 算法来源：从 unitreerobotics/xr_teleoperate 提取
//   - teleop/robot_control/robot_arm_ik.py 里的 R1_A5_ArmIK / R1_A7_ArmIK
//   - assets/r1/r1_a5.urdf / r1_a7.urdf（关节 origin/axis/limit 全部按 URDF 原值）
//
// 官方公式（CasADi + IPOPT 非线性最小二乘）：
//   min  50*||p_ee(q) - p_target||^2 + c_rot*||log3(R_ee(q)*R_target^T)||^2
//        + 0.02*||q||^2 + 0.1*||q - q_last||^2
//   s.t. joint limits
//   - A5: c_rot = 0.5，EE = 腕roll关节 + 0.20m（x 方向，手安装偏移）
//   - A7: c_rot = 1.0，EE = 腕yaw关节 + 0.05m（x 方向，手安装偏移）
//   - 求解器 warm start 于当前关节角；IPOPT max_iter=30；输出再过
//     WMA[0.4,0.3,0.2,0.1] 平滑；失败回退为当前关节角。
//
// 本文件是无第三方依赖（仅 Eigen）的 C++ 移植：用阻尼最小二乘（DLS）迭代
// 最小化与官方**完全相同**的四项目标函数（含正则项与平滑项），并叠加关节限位。
// 追求最高保真度时，可直接在 Python 侧调用 xr_teleoperate 的 R1_A5/A7_ArmIK，
// 或引入 Pinocchio/CasADi 把官方求解器原样搬到 C++。
//
// ⚠️ 历史坑（2026-09-22 修复）：早期版本只实现了前两项（位置 + 姿态），
//    丢掉了 `0.02*||q||^2` 与 `0.1*||q - q_last||^2`。在 A5 这种 5-DOF 超定臂上，
//    姿态残差很大时那两项缺失会让 DLS 把解推向关节限位（肘顶死 −55.9°、
//    位置误差 649 mm）；补齐后 52.5 mm，与官方 IPOPT 的 52.4 mm 一致。
//    两项里**平滑项是决定性的**（只加它 → 50.8 mm），正则项次之（只加它 → 105 mm）。
// ============================================================================

#include <eigen3/Eigen/Dense>

#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace r1skeleton {

using Mat4 = Eigen::Matrix4d;
using Iso = Eigen::Isometry3d;   // 齐次变换：R(t) 到目标变换，R 为旋转、t 为平移

/// 一个旋转关节描述（取自 URDF：origin 是子坐标系相对于父坐标系的位置，
/// axis 是关节轴线（子坐标系下），lower/upper 为弧度限位）。
struct ArmJoint {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();  // URDF origin 的固定旋转（本项目只有 x 分量）
  double lower = -M_PI;
  double upper = M_PI;
};

/// URDF rpy（固定轴 XYZ 顺序：R = Rz*Ry*Rx）。
inline Eigen::Matrix3d rotFromRpy(const Eigen::Vector3d& rpy) {
  return Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
}

/// R1 双臂运动学模型（root = 骨盆/躯干坐标系，waist_yaw 锁在 0，与官方一致）。
class R1ArmKinematics {
 public:
  enum Variant { R1_A5 = 5, R1_A7 = 7 };

  explicit R1ArmKinematics(Variant v) : variant_(v) { buildChains(); }

  /// 单臂自由度：A5=5，A7=7。
  int dofPerArm() const { return static_cast<int>(variant_); }
  int dofTotal() const { return dofPerArm() * 2; }

  /// 返回一只手臂的关节链（从左肩到腕，顺序与 LowCmd 槽位一致，
  /// 也即 xr_teleoperate 的 R1_A5_JointArmIndex / R1_A7_JointArmIndex）。
  const std::vector<ArmJoint>& chain(bool left) const { return left ? left_ : right_; }

  /// 正运动学：输入单臂关节角 q（n 维，链顺序），输出腕部 EE 坐标系（骨盆系）。
  /// EE 已包含官方手安装偏移（A5: +0.20m x，A7: +0.05m x）。
  Iso forward(const Eigen::VectorXd& q, bool left) const {
    const auto& joints = chain(left);
    Iso T = Iso::Identity();
    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
      const ArmJoint& j = joints[i];
      Iso Tj = Iso::Identity();
      Tj.translate(j.origin);
      Tj.rotate(rotFromRpy(j.rpy));                       // 关节放置（子系相对父系）
      Tj.rotate(Eigen::AngleAxisd(q[i], j.axis.normalized()));  // 关节转动（子系轴线）
      T = T * Tj;
    }
    // 手安装偏移（官方 IK 的 EE frame 定义）
    T.translate(Eigen::Vector3d(eeOffsetM(), 0.0, 0.0));
    return T;
  }

  /// 数值雅可比（6 x n）：[位置(3); 姿态(3)]，姿态用旋转向量误差。
  Eigen::MatrixXd jacobian(const Eigen::VectorXd& q, bool left) const {
    const int n = dofPerArm();
    const double h = 1e-6;
    Eigen::MatrixXd J(6, n);
    const Iso T0 = forward(q, left);
    Eigen::Matrix3d R0 = T0.linear();
    Eigen::Vector3d p0 = T0.translation();
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd qp = q;
      qp[i] += h;
      Iso Tp = forward(qp, left);
      // 位置雅可比
      J.block<3, 1>(0, i) = (Tp.translation() - p0) / h;
      // 姿态雅可比：旋转向量误差 / h
      Eigen::Matrix3d dR = Tp.linear() * R0.transpose();
      J.block<3, 1>(3, i) = rotationVector(dR) / h;
    }
    return J;
  }

  /// 把关节角限制到 URDF 限位内。
  void clamp(Eigen::VectorXd& q, bool left) const {
    const auto& joints = chain(left);
    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
      q[i] = std::max(joints[i].lower, std::min(joints[i].upper, q[i]));
    }
  }

 private:
  void buildChains() {
    const bool a7 = (variant_ == R1_A7);

    // ---- 左臂 ----
    left_ = {
      // left_shoulder_pitch: origin [0,0.085688,0.19749] rpy 0.26187, axis Y
      {Eigen::Vector3d(0.0, 0.085688, 0.19749), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d(0.26187, 0.0, 0.0), -3.1416, 2.0944},
      // left_shoulder_roll: origin [0.03445,0.047132,-0.025693] rpy -0.26187, axis X
      {Eigen::Vector3d(0.03445, 0.047132, -0.025693), Eigen::Vector3d::UnitX(),
       Eigen::Vector3d(-0.26187, 0.0, 0.0), -0.22689, 2.4784},
      // left_shoulder_yaw: origin [-0.03445,0.0043,-0.10835], axis Z
      {Eigen::Vector3d(-0.03445, 0.0043, -0.10835), Eigen::Vector3d::UnitZ(),
       Eigen::Vector3d::Zero(), -1.9199, 1.9199},
      // left_elbow: origin [0.016191,0.026461,-0.082858], axis Y
      {Eigen::Vector3d(0.016191, 0.026461, -0.082858), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d::Zero(), -0.97564, 2.1852},
      // left_wrist_roll: origin [0.11218,-0.03002,-0.011702], axis X
      {Eigen::Vector3d(0.11218, -0.03002, -0.011702), Eigen::Vector3d::UnitX(),
       Eigen::Vector3d::Zero(), -1.9199, 1.9199},
    };
    if (a7) {
      // left_wrist_pitch: origin [0.04646,-0.0018006,0.00064554], axis Y
      left_.push_back({Eigen::Vector3d(0.04646, -0.0018006, 0.00064554), Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::Zero(), -1.6144, 1.6144});
      // left_wrist_yaw: origin [0.051,0.00013893,0.02225], axis Z
      left_.push_back({Eigen::Vector3d(0.051, 0.00013893, 0.02225), Eigen::Vector3d::UnitZ(),
                       Eigen::Vector3d::Zero(), -1.6144, 1.6144});
    }

    // ---- 右臂（y 镜像）----
    right_ = {
      {Eigen::Vector3d(0.0, -0.085688, 0.19749), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d(-0.26187, 0.0, 0.0), -3.1416, 2.0944},
      {Eigen::Vector3d(0.03445, -0.047132, -0.025693), Eigen::Vector3d::UnitX(),
       Eigen::Vector3d(0.26187, 0.0, 0.0), -2.47849, 0.2268},
      {Eigen::Vector3d(-0.03445, -0.0043, -0.10835), Eigen::Vector3d::UnitZ(),
       Eigen::Vector3d::Zero(), -1.9199, 1.9199},
      {Eigen::Vector3d(0.016191, -0.026461, -0.082858), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d::Zero(), -0.97564, 2.1852},
      {Eigen::Vector3d(0.11218, 0.03002, -0.011702), Eigen::Vector3d::UnitX(),
       Eigen::Vector3d::Zero(), -1.9199, 1.9199},
    };
    if (a7) {
      right_.push_back({Eigen::Vector3d(0.04646, 0.0018084, -0.00062321), Eigen::Vector3d::UnitY(),
                         Eigen::Vector3d::Zero(), -1.6144, 1.6144});
      right_.push_back({Eigen::Vector3d(0.051235, 0.00013294, -0.021703), Eigen::Vector3d::UnitZ(),
                        Eigen::Vector3d::Zero(), -1.6144, 1.6144});
    }
  }

  /// EE 手安装偏移（官方 IK 的 EE frame 定义）。
  double eeOffsetM() const { return (variant_ == R1_A7) ? 0.05 : 0.20; }

  /// R 的旋转向量（log map）。R 接近单位阵时返回零向量。
  static Eigen::Vector3d rotationVector(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    const double angle = aa.angle();
    return aa.axis() * angle;
  }

  Variant variant_;
  std::vector<ArmJoint> left_;
  std::vector<ArmJoint> right_;
};

/// 双臂 IK（完整最小化官方四项目标 + 关节限位 + 输出平滑），移植自官方
/// R1_A5/A7_ArmIK 的 `opti.minimize(...)` 与其后的 WeightedMovingFilter。
class R1DualArmIk {
 public:
  explicit R1DualArmIk(R1ArmKinematics::Variant v)
      : kin_(v),
        w_pos_(50.0),                                       // 官方位置权重
        w_rot_((v == R1ArmKinematics::R1_A7) ? 1.0 : 0.5),  // 官方姿态权重
        w_reg_(0.02),                                       // 官方正则项 0.02*||q||^2
        w_smooth_(0.1),                                     // 官方平滑项 0.1*||q-q_last||^2
        lambda_(1e-3),
        max_iter_(30),                                      // 对齐官方 ipopt.max_iter=30
        filtered_(Eigen::VectorXd::Zero(2 * kin_.dofPerArm())) {}

  /// 输入：左右 EE 目标（骨盆系）、当前双臂关节角 q_cur = [左 n, 右 n]。
  /// 输出：求解后的双臂关节角（长度 2n）。
  Eigen::VectorXd solve(const Iso& L_target, const Iso& R_target,
                        const Eigen::VectorXd& q_cur) {
    const int n = kin_.dofPerArm();
    if (q_cur.size() != 2 * n) {
      std::cerr << "[ik] q_cur size " << q_cur.size() << " != " << 2 * n << std::endl;
      return q_cur;
    }
    Eigen::VectorXd qL = solveArm(L_target, q_cur.head(n), true);
    Eigen::VectorXd qR = solveArm(R_target, q_cur.tail(n), false);
    Eigen::VectorXd q(2 * n);
    q << qL, qR;
    return smooth(q);
  }

  /// 单臂求解（DLS），最小化官方四项目标：
  ///   min  50*||p - p_t||^2 + c_rot*||log3(R*R_t^T)||^2
  ///        + 0.02*||q||^2 + 0.1*||q - q_last||^2     s.t. 关节限位
  /// 做法：把四项折成一个加权最小二乘 ||Jw*dq - r||，
  ///   r  = [ √w_pos*(p_t-p) ; -√w_rot*e_rot ; √w_reg*q ; √w_smooth*(q-q_last) ]
  ///   Jw = [ √w_pos*J_p ; √w_rot*J_w ; -√w_reg*I ; -√w_smooth*I ]
  ///
  /// `q_last` 的语义：官方 solve_ik() 把 `var_q_last`（平滑项参考）与 warm start
  /// 设成**同一个量** —— robot_arm.py:2419 传入的 `current_lr_arm_motor_q`
  /// （当前实测关节角）。所以 q_last 就是本帧的起始 q0，不是上一帧的解。
  Eigen::VectorXd solveArm(const Iso& target, const Eigen::VectorXd& q0, bool left) {
    const int n = kin_.dofPerArm();
    Eigen::VectorXd q = q0;
    kin_.clamp(q, left);
    const Eigen::VectorXd q_last = q;      // = warm start，见上方说明

    const Eigen::Vector3d p_t = target.translation();
    const Eigen::Matrix3d R_t = target.linear();
    const double sw_pos = std::sqrt(w_pos_);
    const double sw_rot = std::sqrt(w_rot_);
    const double sw_reg = std::sqrt(w_reg_);
    const double sw_smooth = std::sqrt(w_smooth_);

    // 残差与雅可比按「6 维任务空间 + n 维正则 + n 维平滑」堆叠
    Eigen::MatrixXd Jw(6 + 2 * n, n);
    Eigen::VectorXd r(6 + 2 * n);
    Jw.block(6, 0, n, n).setZero();
    Jw.block(6 + n, 0, n, n).setZero();
    for (int i = 0; i < n; ++i) {
      // 正则/平滑项残差是 +√w*q、+√w*(q−q_last)，其对 q 的偏导是 +√w*I；
      // 写成 Jw*dq = r 的形式后为 **−√w*I**。
      // ⚠️ 负号不能漏 —— 写正号会把解推向关节限位（实测 0.87 mm → 181 mm）。
      Jw(6 + i, i) = -sw_reg;
      Jw(6 + n + i, i) = -sw_smooth;
    }

    for (int iter = 0; iter < max_iter_; ++iter) {
      Iso T = kin_.forward(q, left);
      Eigen::Vector3d e_pos = p_t - T.translation();
      // 旋转误差：R_cur * R_t^T 的旋转向量（官方 log3(R_ee * R_target^T)）。
      // 取负：线性化后位置项需要 J_p·dq = p_t - p（即 +e_pos），
      // 而姿态项需要 J_w·dq = log(R_t * R_cur^T) = -rotvec(R_cur * R_t^T)（即 -e_rot）。
      // 若姿态项也用 +e_rot，它会与位置项反向对抗，误差稍大即发散到限位。
      Eigen::Vector3d e_rot = rotationVector(T.linear() * R_t.transpose());

      r << sw_pos * e_pos, -sw_rot * e_rot, sw_reg * q, sw_smooth * (q - q_last);

      Eigen::MatrixXd J = kin_.jacobian(q, left);   // 6 x n
      Jw.topRows(6) = J;
      // 权重必须同时作用于雅可比与误差：求解 min ||Jw dq - r||，
      // 即 J_w = √w*J、r = √w*e。若只缩放 r 而不缩放 J，
      // 等效步长会被放大 sqrt(w_pos)≈7.07 倍，第一步即冲过目标并撞限位（实测发散）。
      for (int row = 0; row < 3; ++row) Jw.row(row) *= sw_pos;
      for (int row = 3; row < 6; ++row) Jw.row(row) *= sw_rot;

      Eigen::MatrixXd A = Jw.transpose() * Jw + lambda_ * Eigen::MatrixXd::Identity(n, n);
      Eigen::VectorXd dq = A.ldlt().solve(Jw.transpose() * r);
      if (!dq.allFinite()) break;

      // 单步限幅：防止大姿态误差下一步跨度过大导致绕限位震荡
      const double step = dq.norm();
      if (step < 1e-9) break;               // 已收敛
      const double kMaxStep = 0.3;          // rad/次迭代
      if (step > kMaxStep) dq *= kMaxStep / step;

      q += dq;
      kin_.clamp(q, left);
    }
    return q;
  }

  /// 输出平滑：移植官方 teleop/utils/weighted_moving_filter.py 的 WeightedMovingFilter。
  /// - weights = [0.4,0.3,0.2,0.1]（新样本权重最大，和恒为 1）；
  /// - 窗口（4 帧）未满时：直通最新样本（不做加权）；
  /// - 与上一帧完全相同的解：跳过（防重复计数）。
  Eigen::VectorXd smooth(const Eigen::VectorXd& q) {
    static const std::array<double, 4> w = {0.4, 0.3, 0.2, 0.1};
    if (!hist_.empty()) {
      bool same = true;
      for (int i = 0; i < q.size(); ++i) {
        if (hist_.back()[i] != q[i]) { same = false; break; }
      }
      if (same) return filtered_;           // 官方 add_data 的重复帧跳过
    }
    if (static_cast<int>(hist_.size()) >= 4) hist_.erase(hist_.begin());
    hist_.push_back(q);
    if (hist_.size() < 4) {
      filtered_ = q;                        // 官方窗口未满时直通最新样本
      return filtered_;
    }
    Eigen::VectorXd out = Eigen::VectorXd::Zero(q.size());
    for (int i = 0; i < 4; ++i) out += w[i] * hist_[3 - i];   // 最新*0.4 … 最旧*0.1
    filtered_ = out;
    return filtered_;
  }

  /// 供上层拿初始 EE 位姿（例如用当前关节角算目标起点）。
  const R1ArmKinematics& kinematics() const { return kin_; }

 private:
  static Eigen::Vector3d rotationVector(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
  }

  R1ArmKinematics kin_;
  double w_pos_;
  double w_rot_;
  double w_reg_;      // 官方 regularization_cost 权重 0.02
  double w_smooth_;   // 官方 smooth_cost 权重 0.1
  double lambda_;     // DLS 阻尼（官方 IPOPT 无此项）
  int max_iter_;
  std::vector<Eigen::VectorXd> hist_;
  Eigen::VectorXd filtered_;    // 上一帧平滑输出（官方 WeightedMovingFilter::_filtered_data）
};

}  // namespace r1skeleton
