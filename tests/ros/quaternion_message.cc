// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/quaternion_message.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(QuaternionMessage, SmokeTest) {
  const double w = std::sqrt(0.4);
  const double x = std::sqrt(0.1);
  const double y = std::sqrt(0.3);
  const double z = std::sqrt(0.2);
  const Eigen::Quaterniond q(w, x, y, z);

  const geometry_msgs::msg::Quaternion message = MakeQuaternionMessage(q);
  EXPECT_EQ(message.w, w);
  EXPECT_EQ(message.x, x);
  EXPECT_EQ(message.y, y);
  EXPECT_EQ(message.z, z);
}
