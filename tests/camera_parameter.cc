// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/camera_parameter.h"

#include <gtest/gtest.h>

#include <numbers>

TEST(CameraParameter, Matrix) {
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};
  const CameraParameter camera_parameter(639., 638., 640., 360., dist_coeffs);

  const cv::Mat k = camera_parameter.Matrix();
  EXPECT_EQ(k.at<double>(0, 0), 639.);
  EXPECT_EQ(k.at<double>(0, 1), 0.);
  EXPECT_EQ(k.at<double>(0, 2), 640.);
  EXPECT_EQ(k.at<double>(1, 0), 0.);
  EXPECT_EQ(k.at<double>(1, 1), 638.);
  EXPECT_EQ(k.at<double>(1, 2), 360.);
  EXPECT_EQ(k.at<double>(2, 0), 0.);
  EXPECT_EQ(k.at<double>(2, 1), 0.);
  EXPECT_EQ(k.at<double>(2, 2), 1.);
}

TEST(CameraParameter, IntrinsicMatrixAsConstructorArgument) {
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};

  const cv::Mat K =
      (cv::Mat_<double>(3, 3) << 639., 0., 640., 0., 638., 360., 0., 0., 1.);
  const CameraParameter camera_parameter(K, dist_coeffs);

  const cv::Mat k = camera_parameter.Matrix();
  EXPECT_EQ(k.at<double>(0, 0), 639.);
  EXPECT_EQ(k.at<double>(0, 1), 0.);
  EXPECT_EQ(k.at<double>(0, 2), 640.);
  EXPECT_EQ(k.at<double>(1, 0), 0.);
  EXPECT_EQ(k.at<double>(1, 1), 638.);
  EXPECT_EQ(k.at<double>(1, 2), 360.);
  EXPECT_EQ(k.at<double>(2, 0), 0.);
  EXPECT_EQ(k.at<double>(2, 1), 0.);
  EXPECT_EQ(k.at<double>(2, 2), 1.);
}

TEST(CameraParameter, VectorArgumentsForConstructor) {
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};
  const CameraParameter camera_parameter({639., 638.}, {640., 360.},
                                         dist_coeffs);

  const cv::Mat k = camera_parameter.Matrix();
  EXPECT_EQ(k.at<double>(0, 0), 639.);
  EXPECT_EQ(k.at<double>(0, 1), 0.);
  EXPECT_EQ(k.at<double>(0, 2), 640.);
  EXPECT_EQ(k.at<double>(1, 0), 0.);
  EXPECT_EQ(k.at<double>(1, 1), 638.);
  EXPECT_EQ(k.at<double>(1, 2), 360.);
  EXPECT_EQ(k.at<double>(2, 0), 0.);
  EXPECT_EQ(k.at<double>(2, 1), 0.);
  EXPECT_EQ(k.at<double>(2, 2), 1.);
}

TEST(CameraParameter, UndistortPoints) {
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};
  const CameraParameter camera_parameter(639., 638., 640., 360., dist_coeffs);

  const std::vector<cv::Point2f> src = {
      cv::Point2f(10., 50.),
      cv::Point2f(40., 30.),
      cv::Point2f(400., 800.),
      cv::Point2f(900., 980.),
  };

  const std::vector<cv::Point2f> dst = camera_parameter.UndistortPoints(src);

  const double epsilon = 1e-5;

  ASSERT_EQ(dst.size(), 4);
  EXPECT_TRUE(std::abs(dst[0].x - -0.973191) < epsilon);
  EXPECT_TRUE(std::abs(dst[0].y - -0.477667) < epsilon);
  EXPECT_TRUE(std::abs(dst[1].x - -0.928332) < epsilon);
  EXPECT_TRUE(std::abs(dst[1].y - -0.509765) < epsilon);
  EXPECT_TRUE(std::abs(dst[2].x - -0.377952) < epsilon);
  EXPECT_TRUE(std::abs(dst[2].y - 0.700911) < epsilon);
  EXPECT_TRUE(std::abs(dst[3].x - 0.41709) < epsilon);
  EXPECT_TRUE(std::abs(dst[3].y - 0.989478) < epsilon);
}

TEST(CameraParameter, DistortPoints) {
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};
  const CameraParameter camera_parameter(639., 638., 640., 360., dist_coeffs);

  const std::vector<cv::Point2f> src = {
      cv::Point2f(-0.973191, -0.477667),
      cv::Point2f(-0.928332, -0.509765),
      cv::Point2f(-0.377952, 0.700911),
      cv::Point2f(0.41709, 0.989478),
  };

  const std::vector<cv::Point2f> dst = camera_parameter.DistortPoints(src);

  const double epsilon = 1e-2;

  ASSERT_EQ(dst.size(), 4);
  EXPECT_TRUE(std::abs(dst[0].x - 10.) < epsilon);
  EXPECT_TRUE(std::abs(dst[0].y - 50.) < epsilon);
  EXPECT_TRUE(std::abs(dst[1].x - 40.) < epsilon);
  EXPECT_TRUE(std::abs(dst[1].y - 30.) < epsilon);
  EXPECT_TRUE(std::abs(dst[2].x - 400.) < epsilon);
  EXPECT_TRUE(std::abs(dst[2].y - 800.) < epsilon);
  EXPECT_TRUE(std::abs(dst[3].x - 900.) < epsilon);
  EXPECT_TRUE(std::abs(dst[3].y - 980.) < epsilon);
}

TEST(ProjectPoints, WithoutDistortion) {
  const double pi = std::numbers::pi;

  const std::vector<Eigen::Vector3d> landmarks = {
      Eigen::Vector3d(-5., 0., 4.),
      Eigen::Vector3d(-3., 0., 1.),
  };

  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();
  const Eigen::AngleAxisd angle_axis(-0.5 * pi, y_axis);
  const Eigen::Vector3d translation(-1., 0., 3.);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.prerotate(angle_axis);
  pose.pretranslate(translation);

  const CameraParameter camera_parameter(10., 10., 50., 40.);
  const std::vector<Eigen::Vector2d> keypoints =
      camera_parameter.ProjectPoints(pose.inverse(), landmarks);

  ASSERT_EQ(keypoints.size(), 2);

  const Eigen::Vector2d expected0(0.25 * 10. + 50., 0. * 10. + 40.);
  const Eigen::Vector2d expected1(-1.0 * 10. + 50., 0. * 10. + 40.);
  EXPECT_TRUE(keypoints[0].isApprox(expected0));
  EXPECT_TRUE(keypoints[1].isApprox(expected1));
}

TEST(ProjectPoints, WithDistortion) {
  const double pi = std::numbers::pi;
  const cv::Vec3f zeros(0., 0., 0.);

  const cv::Mat k =
      (cv::Mat_<double>(3, 3) << 10., 0., 50., 0., 10., 40., 0., 0., 1.);
  const std::vector<double> dist_coeffs = {-0.034, 0.027, -0.0036, -0.0039};

  const std::vector<cv::Point3f> landmarks_camera_coordinate = {
      cv::Point3f(1., 0., 4.),
      cv::Point3f(-2., 0., 2.),
  };
  std::vector<cv::Point2f> expected;
  cv::projectPoints(landmarks_camera_coordinate, zeros, zeros, k, dist_coeffs,
                    expected);

  const std::vector<cv::Point3f> landmarks = {
      cv::Point3f(-5., 0., 4.),
      cv::Point3f(-3., 0., 1.),
  };

  const Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();
  const Eigen::AngleAxisd angle_axis(-0.5 * pi, y_axis);
  const Eigen::Vector3d translation(-1., 0., 3.);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.prerotate(angle_axis);
  pose.pretranslate(translation);

  const CameraParameter camera_parameter(10., 10., 50., 40., dist_coeffs);
  const std::vector<cv::Point2f> keypoints =
      camera_parameter.ProjectPoints(pose.inverse(), landmarks);

  const double epsilon = 1e-4;

  ASSERT_EQ(keypoints.size(), 2);
  EXPECT_TRUE(std::abs(keypoints[0].x - expected[0].x) < epsilon);
  EXPECT_TRUE(std::abs(keypoints[0].y - expected[0].y) < epsilon);
  EXPECT_TRUE(std::abs(keypoints[1].x - expected[1].x) < epsilon);
  EXPECT_TRUE(std::abs(keypoints[1].y - expected[1].y) < epsilon);
}
