// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_TRIANGULATION_H_
#define INCLUDE_VISUAL_LOCALIZATION_TRIANGULATION_H_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <optional>
#include <vector>

constexpr double kEpsilon = 1e-8;

using EigenSolver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>;

inline std::optional<Eigen::Vector3d> LinearTriangulation(
    const Eigen::Isometry3d& transform_c1w,
    const Eigen::Isometry3d& transform_c2w, const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2) {
  const Eigen::Matrix4d& m1 = transform_c1w.matrix();
  const Eigen::Matrix4d& m2 = transform_c2w.matrix();

  Eigen::Matrix4d a;
  a.row(0) = p1(1) * m1.row(2) - m1.row(1);
  a.row(1) = m1.row(0) - p1(0) * m1.row(2);
  a.row(2) = p2(1) * m2.row(2) - m2.row(1);
  a.row(3) = m2.row(0) - p2(0) * m2.row(2);

  const EigenSolver solver(a.transpose() * a);
  const Eigen::Vector4d v = solver.eigenvectors().col(0);

  if (std::abs(v(3)) < kEpsilon) {
    return std::nullopt;
  }

  return v.head(3) / v(3);
}

inline bool IsInFrontOfCamera(const Eigen::Isometry3d& transform_cw,
                              const Eigen::Vector3d& landmark) {
  const Eigen::Vector3d p = transform_cw * landmark;
  return p(2) > 0.0;
}

std::vector<std::optional<Eigen::Vector3d>> LinearTriangulation(
    const Eigen::Isometry3d& transform_c1w,
    const Eigen::Isometry3d& transform_c2w,
    const std::vector<Eigen::Vector2d>& keypoints1,
    const std::vector<Eigen::Vector2d>& keypoints2);

#endif  // INCLUDE_VISUAL_LOCALIZATION_TRIANGULATION_H_
