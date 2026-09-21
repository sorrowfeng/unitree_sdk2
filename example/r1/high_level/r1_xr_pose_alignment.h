#pragma once
// ============================================================================
// r1_xr_pose_alignment.h
// OpenXR 位姿 → 宇树机器人（R1 骨盆/腰部系）对准。
//
// 移植自 unitreerobotics/televuer: src/televuer/tv_wrapper.py
//   - 基线系相似变换：Brobot = T_ROBOT_OPENXR * Bxr * T_OPENXR_ROBOT
//     OpenXR(右手系, Y上, Z后, X右)  →  Robot(Z上, Y左, X前)
//   - world → head → waist：
//       head_yaw（默认）: 只取头部偏航旋转 R_yaw^T*(P - P_head)，忽略俯仰/横滚
//       head_trans     : 只做平移 P - P_head
//     再平移 +0.15m(x) +0.45m(z) 到腰部（R1 IK 原点，即骨架 r1_arm_ik.h 的根）。
//
// 输入输出均为 4x4 齐次 SE(3)（Eigen::Isometry3d，单位米）。
// ============================================================================

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace r1skeleton {
namespace xralign {

/// OpenXR → Robot 基线系（4x4）。
inline Eigen::Matrix4d robotOpenXr() {
  Eigen::Matrix4d m;
  // clang-format off
  m << 0, 0,-1, 0,
      -1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 0, 1;
  // clang-format on
  return m;
}

/// Robot → OpenXR 基线系（4x4），即 robotOpenXr 的逆。
inline Eigen::Matrix4d openXrRobot() {
  Eigen::Matrix4d m;
  // clang-format off
  m << 0,-1, 0, 0,
       0, 0, 1, 0,
      -1, 0, 0, 0,
       0, 0, 0, 1;
  // clang-format on
  return m;
}

/// 相似变换：把 OpenXR 基系下的位姿换成 Robot 基系下的同一位姿。
inline Eigen::Isometry3d basisToRobot(const Eigen::Isometry3d& pose) {
  Eigen::Matrix4d p = pose.matrix();
  Eigen::Matrix4d out = robotOpenXr() * p * openXrRobot();
  Eigen::Isometry3d r;
  r.matrix() = out;
  return r;
}

enum class RefMode { kHeadYaw, kHeadTranslation };

/// 头部只取偏航的旋转（Robot 基系），z 轴竖直。
inline Eigen::Matrix3d headYawRotation(const Eigen::Isometry3d& head) {
  Eigen::Vector3d x = head.linear().col(0);
  x.z() = 0.0;
  const double n = x.norm();
  if (n < 1e-9) return Eigen::Matrix3d::Identity();
  x /= n;
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d y = z.cross(x);       // 保持右手系
  if (y.norm() < 1e-9) return Eigen::Matrix3d::Identity();
  y /= y.norm();
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return R;
}

/// world(已转 Robot 基系) → head(reference) → waist(R1 IK 原点)。
inline Eigen::Isometry3d worldToWaist(const Eigen::Isometry3d& pose, const Eigen::Isometry3d& head, RefMode mode) {
  Eigen::Isometry3d out = pose;
  if (mode == RefMode::kHeadYaw) {
    const Eigen::Matrix3d Ryaw = headYawRotation(head);
    out.linear() = Ryaw.transpose() * out.linear();
    out.translation() = Ryaw.transpose() * (out.translation() - head.translation());
  } else {
    out.translation() -= head.translation();
  }
  // 头部 → 腰部（R1 A5/A7 的 IK 根位于腰部关节附近，offset 取自官方 teleop）
  out.translation().x() += 0.15;
  out.translation().z() += 0.45;
  return out;
}

/// 一键对齐：OpenXR 世界系下手柄/腕部位姿 → R1 IK 可直接消费的骨盆/腰部系位姿。
inline Eigen::Isometry3d alignWristToRobot(const Eigen::Isometry3d& wrist_openxr,
                                           const Eigen::Isometry3d& head_openxr,
                                           RefMode mode = RefMode::kHeadYaw) {
  const Eigen::Isometry3d wrist_robot = basisToRobot(wrist_openxr);
  const Eigen::Isometry3d head_robot = basisToRobot(head_openxr);
  return worldToWaist(wrist_robot, head_robot, mode);
}

/// 兜底头部位姿（OpenXR 世界系，对应官方 CONST_HEAD_POSE）。
inline Eigen::Isometry3d defaultHeadPose() {
  Eigen::Isometry3d h = Eigen::Isometry3d::Identity();
  h.translation() = Eigen::Vector3d(0.0, 1.5, -0.2);
  return h;
}

}  // namespace xralign
}  // namespace r1skeleton
