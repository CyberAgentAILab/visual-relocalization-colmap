// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/eigen_utils.h"

#include <gtest/gtest.h>

namespace eigen_utils {

TEST(FromScaledAxis, SmokeTest) {
  const Eigen::Vector3d v(-3.5, 0.2, -0.9);
  const Eigen::AngleAxisd a = FromScaledAxis(v);
  EXPECT_EQ(a.angle(), v.norm());
  EXPECT_EQ(a.axis(), v / v.norm());
}

TEST(FromScaledAxis, ZeroNorm) {
  const Eigen::Vector3d v = Eigen::Vector3d::Zero();
  const Eigen::AngleAxisd a = FromScaledAxis(v);
  EXPECT_EQ(a.angle(), 0.);
  // We only check that the angle is zero since it corresponds to the identity
  // rotation even if the axis is any.
}

TEST(ToScaledAxis, SmokeTest) {
  const Eigen::Vector3d v(-3.5, 0.2, -0.9);
  const Eigen::AngleAxisd a = FromScaledAxis(v);
  EXPECT_TRUE(v.isApprox(ToScaledAxis(a)));
}

TEST(MakeIsometry3d, FromAngleAxisdAndVector3d) {
  const double pi = std::numbers::pi;

  const Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
  const Eigen::AngleAxisd angle_axis(0.5 * pi, x_axis);
  const Eigen::Vector3d translation(0.5, 3.0, 2.0);

  const Eigen::Isometry3d transform = MakeIsometry3d(angle_axis, translation);
  EXPECT_EQ(transform.rotation(), angle_axis.toRotationMatrix());
  EXPECT_EQ(transform.translation(), translation);
}

TEST(MakeIsometry3d, FromQuaterniondAndVector3d) {
  const double pi = std::numbers::pi;

  const Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(0.5 * pi, x_axis));
  const Eigen::Vector3d translation(0.5, 3.0, 2.0);

  const Eigen::Isometry3d transform = MakeIsometry3d(quaternion, translation);
  EXPECT_EQ(transform.rotation(), quaternion.toRotationMatrix());
  EXPECT_EQ(transform.translation(), translation);
}

}  // namespace eigen_utils
