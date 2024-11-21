// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_CAMERA_PARAMETER_H_
#define INCLUDE_VISUAL_LOCALIZATION_CAMERA_PARAMETER_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "visual_localization/cv_eigen.h"
#include "visual_localization/eigen_utils.h"

struct CameraParameter {
 public:
  CameraParameter(const std::vector<double>& focal_length,
                  const std::vector<double>& offset,
                  const std::vector<double>& dist_coeffs = {})
      : CameraParameter(focal_length.at(0), focal_length.at(1), offset.at(0),
                        offset.at(1), dist_coeffs) {}

  explicit CameraParameter(const cv::Mat& intrinsic_matrix,
                           const std::vector<double>& dist_coeffs = {})
      : fx(intrinsic_matrix.at<double>(0, 0)),
        fy(intrinsic_matrix.at<double>(1, 1)),
        cx(intrinsic_matrix.at<double>(0, 2)),
        cy(intrinsic_matrix.at<double>(1, 2)),
        dist_coeffs(dist_coeffs) {}

  CameraParameter(const double fx, const double fy, const double cx,
                  const double cy, const std::vector<double>& dist_coeffs = {})
      : fx(fx), fy(fy), cx(cx), cy(cy), dist_coeffs(dist_coeffs) {}

  cv::Mat Matrix() const {
    return (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  }

  template <typename KeypointsType>
  KeypointsType UndistortPoints(const KeypointsType& src) const {
    if (src.empty()) {
      return {};
    }

    const cv::Mat_<double> cv_dist_coeffs(dist_coeffs);

    KeypointsType dst;
    cv::undistortPoints(src, dst, this->Matrix(), cv_dist_coeffs);
    return dst;
  }

  template <typename KeypointsType>
  KeypointsType DistortPoints(const KeypointsType& src) const {
    const cv::Vec3f zeros(0., 0., 0.);
    const cv::Mat_<double> cv_dist_coeffs(dist_coeffs);

    std::vector<cv::Point3f> homogeneous;
    cv::convertPointsToHomogeneous(src, homogeneous);

    KeypointsType dst;
    cv::projectPoints(homogeneous, zeros, zeros, this->Matrix(), cv_dist_coeffs,
                      dst);
    return dst;
  }

  std::vector<cv::Point2f> ProjectPoints(
      const Eigen::Isometry3d& transform_cw,
      const std::vector<cv::Point3f>& landmarks) const {
    if (landmarks.empty()) {
      return {};
    }

    const cv::Mat_<double> cv_dist_coeffs(dist_coeffs);
    const Eigen::AngleAxisd angle_axis(transform_cw.rotation());
    const Eigen::Vector3d rot = eigen_utils::ToScaledAxis(angle_axis);
    const Eigen::Vector3d t = transform_cw.translation();

    const cv::Mat cv_rot = (cv::Mat_<double>(3, 1) << rot(0), rot(1), rot(2));
    const cv::Mat cv_t = (cv::Mat_<double>(3, 1) << t(0), t(1), t(2));

    std::vector<cv::Point2f> keypoints;
    cv::projectPoints(landmarks, cv_rot, cv_t, this->Matrix(), cv_dist_coeffs,
                      keypoints);
    return keypoints;
  }

  std::vector<Eigen::Vector2d> ProjectPoints(
      const Eigen::Isometry3d& transform_cw,
      const std::vector<Eigen::Vector3d>& landmarks) const {
    const auto cv_landmarks = EigenToCvPoint<float>(landmarks);
    const auto cv_keypoints = this->ProjectPoints(transform_cw, cv_landmarks);
    return CvPointToEigen(cv_keypoints);
  }

  const double fx;
  const double fy;
  const double cx;
  const double cy;
  const std::vector<double> dist_coeffs;
};

#endif  // INCLUDE_VISUAL_LOCALIZATION_CAMERA_PARAMETER_H_
