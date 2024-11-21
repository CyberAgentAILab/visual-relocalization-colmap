// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/ros/image_message.h"

#include <gtest/gtest.h>

TEST(MakeImageMessage, SmokeTest) {
  const cv::Mat image = cv::imread("../assets/tracking/00.jpg");
  ASSERT_FALSE(image.empty());

  std_msgs::msg::Header header;
  header.frame_id = "camera_frame";
  const sensor_msgs::msg::Image::SharedPtr message =
      MakeImageMessage(image, header);
  EXPECT_STREQ(message->header.frame_id.c_str(), "camera_frame");
  EXPECT_EQ(message->height, image.rows);
  EXPECT_EQ(message->width, image.cols);
}
