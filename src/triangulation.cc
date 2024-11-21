// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/triangulation.h"

#include <Eigen/Core>
#include <optional>

std::vector<std::optional<Eigen::Vector3d>> LinearTriangulation(
    const Eigen::Isometry3d& transform_c1w,
    const Eigen::Isometry3d& transform_c2w,
    const std::vector<Eigen::Vector2d>& keypoints1,
    const std::vector<Eigen::Vector2d>& keypoints2) {
  assert(keypoints1.size() == keypoints2.size());
  std::vector<std::optional<Eigen::Vector3d>> landmarks(keypoints1.size());
  for (size_t i = 0; i < keypoints1.size(); i++) {
    const Eigen::Vector2d& p1 = keypoints1[i];
    const Eigen::Vector2d& p2 = keypoints2[i];
    landmarks[i] = LinearTriangulation(transform_c1w, transform_c2w, p1, p2);
  }
  return landmarks;
}
