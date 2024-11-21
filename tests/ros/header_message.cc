// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/header_message.h"

#include <gtest/gtest.h>

TEST(MakeHeaderMessage, SmokeTest) {
  builtin_interfaces::msg::Time time;
  time.sec = 100;
  time.nanosec = 80000;
  const std_msgs::msg::Header header = MakeHeaderMessage("map", time);
  EXPECT_EQ(header.frame_id, "map");
  EXPECT_EQ(header.stamp, time);
}
