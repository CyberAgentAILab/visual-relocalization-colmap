// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_PNP_H_
#define INCLUDE_VISUAL_LOCALIZATION_PNP_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <vector>

std::optional<Eigen::Isometry3d> SolvePnp(
    const std::vector<Eigen::Vector3d>& landmarks,
    const std::vector<Eigen::Vector2d>& image_points);

#endif  // INCLUDE_VISUAL_LOCALIZATION_PNP_H_
