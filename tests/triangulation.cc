// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/triangulation.h"

#include <gtest/gtest.h>

#include <numbers>

constexpr double kPi = std::numbers::pi;

TEST(Triangulation, ParallelOpticalAxes) {
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.pretranslate(Eigen::Vector3d(1., 0., 0.));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3., 0., 0.));

  const Eigen::Vector2d p1(0.2, 0.0);
  const Eigen::Vector2d p2(-0.2, 0.0);
  const std::optional<Eigen::Vector3d> maybe_point =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), p1, p2);
  ASSERT_TRUE(maybe_point.has_value());

  const Eigen::Vector3d expected(2.0, 0.0, 5.0);
  EXPECT_TRUE(maybe_point.value().isApprox(expected));
}

TEST(Triangulation, NonParallelOpticalAxes1) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.prerotate(Eigen::AngleAxis(0.5 * kPi, y_axis));
  pose1.pretranslate(Eigen::Vector3d(0.0, 0.0, 4.0));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3.0, 0.0, 0.));

  const Eigen::Vector2d p1(-0.5, 0.5);
  const Eigen::Vector2d p2(-0.2, 0.2);
  const std::optional<Eigen::Vector3d> maybe_point =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), p1, p2);
  ASSERT_TRUE(maybe_point.has_value());

  const Eigen::Vector3d expected(2.0, 1.0, 5.0);
  EXPECT_TRUE(maybe_point.value().isApprox(expected));
}

TEST(Triangulation, NonParallelOpticalAxes2) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.prerotate(Eigen::AngleAxis(0.25 * kPi, y_axis));
  pose1.pretranslate(Eigen::Vector3d(-3.0, 0.0, 0.0));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3.0, 0.0, 0.0));

  const Eigen::Vector2d p1(0.0, -1.0 / (5. * std::sqrt(2.)));
  const Eigen::Vector2d p2(-0.2, -0.2);
  const std::optional<Eigen::Vector3d> maybe_point =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), p1, p2);
  ASSERT_TRUE(maybe_point.has_value());

  const Eigen::Vector3d expected(2.0, -1.0, 5.0);
  EXPECT_TRUE(maybe_point.value().isApprox(expected));
}

TEST(Triangulation, LandmarkIsBehindCamera) {
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.pretranslate(Eigen::Vector3d(-3.0, 0.0, 3.0));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3.0, 0.0, 3.0));

  const Eigen::Vector2d p1(-1.0, 0.0);
  const Eigen::Vector2d p2(1.0, 0.0);
  const std::optional<Eigen::Vector3d> maybe_point =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), p1, p2);

  ASSERT_TRUE(maybe_point.has_value());

  const Eigen::Vector3d expected(0.0, 0.0, 0.0);
  EXPECT_TRUE(maybe_point.value().isApprox(expected));
}

TEST(Triangulation, DegenerateCase) {
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.pretranslate(Eigen::Vector3d(3.0, 0.0, 3.0));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3.0, 0.0, 3.0));

  const Eigen::Vector2d p1(1.0, 0.0);
  const Eigen::Vector2d p2(1.0, 0.0);
  const std::optional<Eigen::Vector3d> maybe_point =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), p1, p2);
  ASSERT_FALSE(maybe_point.has_value());
}

TEST(Triangulation, MultipleLandmarks) {
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.pretranslate(Eigen::Vector3d(-3.0, 0.0, 0.0));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.pretranslate(Eigen::Vector3d(3.0, 0.0, 0.0));

  const std::vector<Eigen::Vector2d> ps1 = {
      Eigen::Vector2d(0.5, 0.0),
      Eigen::Vector2d(1.0, 0.0),
      Eigen::Vector2d(1.5, 0.0),
  };
  const std::vector<Eigen::Vector2d> ps2 = {
      Eigen::Vector2d(-0.5, 0.0),
      Eigen::Vector2d(-1.0, 0.0),
      Eigen::Vector2d(-1.5, 0.0),
  };
  const std::vector<std::optional<Eigen::Vector3d>> maybe_points =
      LinearTriangulation(pose1.inverse(), pose2.inverse(), ps1, ps2);
  ASSERT_EQ(maybe_points.size(), 3);
  ASSERT_TRUE(maybe_points[0].has_value());
  ASSERT_TRUE(maybe_points[1].has_value());
  ASSERT_TRUE(maybe_points[2].has_value());

  const Eigen::Vector3d expected0(0.0, 0.0, 6.0);
  const Eigen::Vector3d expected1(0.0, 0.0, 3.0);
  const Eigen::Vector3d expected2(0.0, 0.0, 2.0);

  EXPECT_TRUE(maybe_points[0].value().isApprox(expected0));
  EXPECT_TRUE(maybe_points[1].value().isApprox(expected1));
  EXPECT_TRUE(maybe_points[2].value().isApprox(expected2));
}

TEST(IsInFrontOfCamera, SmokeTest) {
  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.prerotate(Eigen::AngleAxis(-0.5 * kPi, y_axis));
  pose.pretranslate(Eigen::Vector3d(0., 0., 2.));

  EXPECT_TRUE(IsInFrontOfCamera(pose.inverse(), Eigen::Vector3d(-3., 0., 0.)));
  EXPECT_FALSE(IsInFrontOfCamera(pose.inverse(), Eigen::Vector3d(0.5, 0., 0.)));
  EXPECT_FALSE(IsInFrontOfCamera(pose.inverse(), Eigen::Vector3d(0.0, 0., 0.)));
}
