// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_CV_EIGEN_H_
#define INCLUDE_VISUAL_LOCALIZATION_CV_EIGEN_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <vector>

template <typename T>
cv::Point3_<T> EigenToCvPoint(const Eigen::Vector3d& p) {
  return cv::Point3_<T>(p(0), p(1), p(2));
}

template <typename T>
std::vector<cv::Point3_<T>> EigenToCvPoint(
    const std::vector<Eigen::Vector3d>& ps) {
  std::vector<cv::Point3_<T>> cvpoints(ps.size());
  for (size_t i = 0; i < ps.size(); i++) {
    cvpoints[i] = EigenToCvPoint<T>(ps[i]);
  }
  return cvpoints;
}

template <typename T>
cv::Point_<T> EigenToCvPoint(const Eigen::Vector2d& p) {
  return cv::Point_<T>(p(0), p(1));
}

template <typename T>
std::vector<cv::Point_<T>> EigenToCvPoint(
    const std::vector<Eigen::Vector2d>& xs) {
  std::vector<cv::Point_<T>> cvpoints(xs.size());
  for (size_t i = 0; i < xs.size(); i++) {
    cvpoints[i] = EigenToCvPoint<T>(xs[i]);
  }
  return cvpoints;
}

template <typename T>
Eigen::Vector3d CvPointToEigen(const cv::Point3_<T>& p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

template <typename T>
std::vector<Eigen::Vector3d> CvPointToEigen(
    const std::vector<cv::Point3_<T>>& cvpoints) {
  std::vector<Eigen::Vector3d> ps(cvpoints.size());
  for (size_t i = 0; i < cvpoints.size(); i++) {
    ps[i] = CvPointToEigen<T>(cvpoints[i]);
  }
  return ps;
}

template <typename T>
Eigen::Vector2d CvPointToEigen(const cv::Point_<T>& p) {
  return Eigen::Vector2d(p.x, p.y);
}

template <typename T>
std::vector<Eigen::Vector2d> CvPointToEigen(
    const std::vector<cv::Point_<T>>& cvpoints) {
  std::vector<Eigen::Vector2d> xs(cvpoints.size());
  for (size_t i = 0; i < cvpoints.size(); i++) {
    xs[i] = CvPointToEigen<T>(cvpoints[i]);
  }
  return xs;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_CV_EIGEN_H_
