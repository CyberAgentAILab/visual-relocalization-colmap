// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/projection.h"

#include <iostream>

std::optional<Eigen::Vector2d> Projection(const Eigen::Isometry3d& transform_cw,
                                          const Eigen::Vector3d& p) {
  const Eigen::Vector3d a = transform_cw * p;

  // We don't calculate the projection if the transformed point has zero depth
  // or it is behind the camera.
  if (a(2) <= 0.) {
    return std::nullopt;
  }
  return a.head(2) / a(2);
}
