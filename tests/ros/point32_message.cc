// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/point32_message.h"

#include <gtest/gtest.h>

TEST(MakePoint32Message, SmokeTest) {
  const Eigen::Vector3d p(10., 20., 30.);
  const auto message = MakePoint32Message(p);
  EXPECT_EQ(message.x, 10.);
  EXPECT_EQ(message.y, 20.);
  EXPECT_EQ(message.z, 30.);
}
