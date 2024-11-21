// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/pose_stamped_message.h"

#include <gtest/gtest.h>

TEST(PoseStampedMessage, SmokeTest) {
  const double qw = std::sqrt(0.2);
  const double qx = std::sqrt(0.1);
  const double qy = std::sqrt(0.3);
  const double qz = std::sqrt(0.4);
  const Eigen::Quaterniond q(qw, qx, qy, qz);

  const double tx = 15.;
  const double ty = 25.;
  const double tz = 35.;
  const Eigen::Vector3d t(tx, ty, tz);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.prerotate(q);
  pose.pretranslate(t);

  std_msgs::msg::Header header;
  header.frame_id = "map";
  const auto message = MakePoseStampedMessage(pose, header);

  const double epsilon = 1e-16;
  EXPECT_TRUE(std::abs(message.pose.orientation.w - qw) < epsilon);
  EXPECT_TRUE(std::abs(message.pose.orientation.x - qx) < epsilon);
  EXPECT_TRUE(std::abs(message.pose.orientation.y - qy) < epsilon);
  EXPECT_TRUE(std::abs(message.pose.orientation.z - qz) < epsilon);
  EXPECT_EQ(message.pose.position.x, tx);
  EXPECT_EQ(message.pose.position.y, ty);
  EXPECT_EQ(message.pose.position.z, tz);
  EXPECT_STREQ(message.header.frame_id.c_str(), "map");
}
