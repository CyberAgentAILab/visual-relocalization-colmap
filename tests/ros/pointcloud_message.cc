// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/pointcloud_message.h"

#include <gtest/gtest.h>

TEST(MakePointCloudMessage, SmokeTest) {
  const std::vector<Eigen::Vector3d> landmarks = {
      Eigen::Vector3d(10., 20., 30.), Eigen::Vector3d(11., 21., 31.),
      Eigen::Vector3d(12., 22., 32.), Eigen::Vector3d(13., 23., 33.)};

  std_msgs::msg::Header header;
  header.frame_id = "map";
  const auto message = MakePointCloudMessage(landmarks, header);

  EXPECT_EQ(message.points.size(), 4);
  EXPECT_EQ(message.points[0].x, 10.);
  EXPECT_EQ(message.points[0].y, 20.);
  EXPECT_EQ(message.points[0].z, 30.);
  EXPECT_EQ(message.points[1].x, 11.);
  EXPECT_EQ(message.points[1].y, 21.);
  EXPECT_EQ(message.points[1].z, 31.);
  EXPECT_EQ(message.points[2].x, 12.);
  EXPECT_EQ(message.points[2].y, 22.);
  EXPECT_EQ(message.points[2].z, 32.);
  EXPECT_EQ(message.points[3].x, 13.);
  EXPECT_EQ(message.points[3].y, 23.);
  EXPECT_EQ(message.points[3].z, 33.);
  EXPECT_STREQ(message.header.frame_id.c_str(), "map");
}
