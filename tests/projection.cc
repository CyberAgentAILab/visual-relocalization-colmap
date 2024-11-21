// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/projection.h"

#include <gtest/gtest.h>

#include <numbers>

constexpr double kPi = std::numbers::pi;

TEST(Projection, IdentityTransform) {
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d p(2., 3., 4.);
  const std::optional<Eigen::Vector2d> x = Projection(identity, p);
  ASSERT_TRUE(x.has_value());

  const Eigen::Vector2d expected(0.5, 0.75);
  EXPECT_TRUE(x.value().isApprox(expected));
}

TEST(Projection, SmokeTest) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d transform_wc = Eigen::Isometry3d::Identity();
  transform_wc.prerotate(Eigen::AngleAxis(-kPi * 0.5, y_axis));
  transform_wc.pretranslate(Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const Eigen::Vector3d p(1., 2., 1.);

  const std::optional<Eigen::Vector2d> x = Projection(transform_cw, p);

  ASSERT_TRUE(x.has_value());

  const Eigen::Vector2d expected(-0.5, 1.0);

  EXPECT_TRUE(x.value().isApprox(expected));
}

TEST(Projection, ZeroDepth) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d transform_wc = Eigen::Isometry3d::Identity();
  transform_wc.prerotate(Eigen::AngleAxis(-kPi * 0.5, y_axis));
  transform_wc.pretranslate(Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const Eigen::Vector3d p(3., 2., 1.);

  const std::optional<Eigen::Vector2d> x = Projection(transform_cw, p);

  ASSERT_FALSE(x.has_value());
}

TEST(Projection, PointIsBehindCamera) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d transform_wc = Eigen::Isometry3d::Identity();
  transform_wc.prerotate(Eigen::AngleAxis(-kPi * 0.5, y_axis));
  transform_wc.pretranslate(Eigen::Vector3d(3., 0., 2.));

  const Eigen::Isometry3d transform_cw = transform_wc.inverse();

  const Eigen::Vector3d p(4., 2., 1.);

  const std::optional<Eigen::Vector2d> x = Projection(transform_cw, p);

  ASSERT_FALSE(x.has_value());
}
