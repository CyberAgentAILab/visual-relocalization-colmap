// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/point_message.h"

#include <gtest/gtest.h>

TEST(PointMessage, SmokeTest) {
  const double x = 15.;
  const double y = 25.;
  const double z = 35.;
  const Eigen::Vector3d t(x, y, z);
  const geometry_msgs::msg::Point message = MakePointMessage(t);
  EXPECT_EQ(message.x, x);
  EXPECT_EQ(message.y, y);
  EXPECT_EQ(message.z, z);
}
