// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/pnp.h"

#include <opencv2/opencv.hpp>

#include "visual_localization/cv_eigen.h"
#include "visual_localization/eigen_utils.h"

inline Eigen::Vector3d GetVector(const cv::Mat& m) {
  return {m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2)};
}

std::optional<Eigen::Isometry3d> SolvePnp(
    const std::vector<Eigen::Vector3d>& landmarks,
    const std::vector<Eigen::Vector2d>& image_points) {
  assert(landmarks.size() == image_points.size());

  if (landmarks.size() < 4) {
    return std::nullopt;
  }

  const std::vector<cv::Point3d> cvlandmarks =
      EigenToCvPoint<double>(landmarks);
  const std::vector<cv::Point2d> cvimage_points =
      EigenToCvPoint<double>(image_points);

  const cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  const cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

  cv::Mat cv_rvec;
  cv::Mat cv_tvec;
  cv::solvePnPRansac(cvlandmarks, cvimage_points, camera_matrix, dist_coeffs,
                     cv_rvec, cv_tvec, false, 1000, 0.1, 0.99999, cv::noArray(),
                     cv::SOLVEPNP_P3P);

  const Eigen::Vector3d rvec = GetVector(cv_rvec);
  const Eigen::Vector3d tvec = GetVector(cv_tvec);

  const Eigen::AngleAxisd angle_axis = eigen_utils::FromScaledAxis(rvec);
  return eigen_utils::MakeIsometry3d(angle_axis, tvec);
}
