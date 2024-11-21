// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_PROJECTION_H_
#define INCLUDE_VISUAL_LOCALIZATION_PROJECTION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

std::optional<Eigen::Vector2d> Projection(const Eigen::Isometry3d& transform_cw,
                                          const Eigen::Vector3d& p);

#endif  // INCLUDE_VISUAL_LOCALIZATION_PROJECTION_H_
