// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/conversion.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

TEST(Vector2dToKeypoints, SmokeTest) {
  const std::vector<Eigen::Vector2d> points = {
      Eigen::Vector2d(0.8, 10.1),
      Eigen::Vector2d(0.7, -0.2),
      Eigen::Vector2d(-0.5, 0.3),
      Eigen::Vector2d(1.1, 10.5),
  };

  const auto keypoints = Vector2dToKeypoints(points);
  EXPECT_EQ(keypoints[0].pt.x, 0.8F);
  EXPECT_EQ(keypoints[0].pt.y, 10.1F);
  EXPECT_EQ(keypoints[1].pt.x, 0.7F);
  EXPECT_EQ(keypoints[1].pt.y, -0.2F);
  EXPECT_EQ(keypoints[2].pt.x, -0.5F);
  EXPECT_EQ(keypoints[2].pt.y, 0.3F);
  EXPECT_EQ(keypoints[3].pt.x, 1.1F);
  EXPECT_EQ(keypoints[3].pt.y, 10.5F);
}

TEST(Point2fToKeypoints, SmokeTest) {
  const std::vector<cv::Point2f> points = {
      cv::Point2f(0.8, 10.1),
      cv::Point2f(0.7, -0.2),
      cv::Point2f(-0.5, 0.3),
      cv::Point2f(1.1, 10.5),
  };

  const auto keypoints = Point2fToKeypoints(points);
  EXPECT_EQ(keypoints[0].pt.x, 0.8F);
  EXPECT_EQ(keypoints[0].pt.y, 10.1F);
  EXPECT_EQ(keypoints[1].pt.x, 0.7F);
  EXPECT_EQ(keypoints[1].pt.y, -0.2F);
  EXPECT_EQ(keypoints[2].pt.x, -0.5F);
  EXPECT_EQ(keypoints[2].pt.y, 0.3F);
  EXPECT_EQ(keypoints[3].pt.x, 1.1F);
  EXPECT_EQ(keypoints[3].pt.y, 10.5F);
}
