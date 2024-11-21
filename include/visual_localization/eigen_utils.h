// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_EIGEN_UTILS_H_
#define INCLUDE_VISUAL_LOCALIZATION_EIGEN_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_utils {

constexpr double kEpsilon = 1e-16;

inline Eigen::Vector3d ToScaledAxis(const Eigen::AngleAxisd& angle_axis) {
  return angle_axis.angle() * angle_axis.axis();
}

inline Eigen::AngleAxisd FromScaledAxis(const Eigen::Vector3d& scaled_axis) {
  const double norm = scaled_axis.norm();
  if (norm < kEpsilon) {
    return Eigen::AngleAxisd(Eigen::Quaterniond::Identity());
  }
  const Eigen::Vector3d axis = scaled_axis / norm;
  return {norm, axis};
}

inline Eigen::Isometry3d MakeIsometry3d(const Eigen::AngleAxisd& angle_axis,
                                        const Eigen::Vector3d& translation) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.prerotate(angle_axis);
  transform.pretranslate(translation);
  return transform;
}

inline Eigen::Isometry3d MakeIsometry3d(const Eigen::Quaterniond& quaternion,
                                        const Eigen::Vector3d& translation) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.prerotate(quaternion);
  transform.pretranslate(translation);
  return transform;
}

}  // namespace eigen_utils

#endif  // INCLUDE_VISUAL_LOCALIZATION_EIGEN_UTILS_H_
