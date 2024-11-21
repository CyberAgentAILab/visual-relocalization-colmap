// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/cv_eigen.h"

#include <gtest/gtest.h>

TEST(CvPointToEigen, 2D) {
  const std::vector<cv::Point2d> cvpoints = {
      cv::Point2d(0.5, 1.4), cv::Point2d(0.4, 3.8), cv::Point2d(4.3, 1.2)};
  const std::vector<Eigen::Vector2d> xs = CvPointToEigen(cvpoints);
  ASSERT_EQ(xs.size(), 3);
  EXPECT_EQ(xs[0], Eigen::Vector2d(0.5, 1.4));
  EXPECT_EQ(xs[1], Eigen::Vector2d(0.4, 3.8));
  EXPECT_EQ(xs[2], Eigen::Vector2d(4.3, 1.2));
}

TEST(CvPointToEigen, 3D) {
  const std::vector<cv::Point3d> cvpoints = {
      cv::Point3d(0.5, 1.4, 0.8), cv::Point3d(0.4, 3.8, 0.9),
      cv::Point3d(4.3, 1.2, 0.1), cv::Point3d(5.1, 6.4, 5.2)};
  const std::vector<Eigen::Vector3d> xs = CvPointToEigen(cvpoints);
  ASSERT_EQ(xs.size(), 4);
  EXPECT_EQ(xs[0], Eigen::Vector3d(0.5, 1.4, 0.8));
  EXPECT_EQ(xs[1], Eigen::Vector3d(0.4, 3.8, 0.9));
  EXPECT_EQ(xs[2], Eigen::Vector3d(4.3, 1.2, 0.1));
  EXPECT_EQ(xs[3], Eigen::Vector3d(5.1, 6.4, 5.2));
}

TEST(EigenToCvPoint, 2D) {
  const std::vector<Eigen::Vector2d> xs = {Eigen::Vector2d(0.5, 1.4),
                                           Eigen::Vector2d(0.4, 3.8),
                                           Eigen::Vector2d(4.3, 1.2)};
  const std::vector<cv::Point2d> cvpoints = EigenToCvPoint<double>(xs);
  ASSERT_EQ(cvpoints.size(), 3);
  EXPECT_EQ(cvpoints[0], cv::Point2d(0.5, 1.4));
  EXPECT_EQ(cvpoints[1], cv::Point2d(0.4, 3.8));
  EXPECT_EQ(cvpoints[2], cv::Point2d(4.3, 1.2));
}

TEST(EigenToCvPoint, 3D) {
  const std::vector<Eigen::Vector3d> ps = {
      Eigen::Vector3d(0.5, 1.4, 0.8), Eigen::Vector3d(0.4, 3.8, 0.9),
      Eigen::Vector3d(4.3, 1.2, 0.1), Eigen::Vector3d(5.1, 6.4, 5.2)};
  const std::vector<cv::Point3d> cvpoints = EigenToCvPoint<double>(ps);
  ASSERT_EQ(cvpoints.size(), 4);
  EXPECT_EQ(cvpoints[0], cv::Point3d(0.5, 1.4, 0.8));
  EXPECT_EQ(cvpoints[1], cv::Point3d(0.4, 3.8, 0.9));
  EXPECT_EQ(cvpoints[2], cv::Point3d(4.3, 1.2, 0.1));
  EXPECT_EQ(cvpoints[3], cv::Point3d(5.1, 6.4, 5.2));
}
