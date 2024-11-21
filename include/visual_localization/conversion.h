// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_CONVERSION_H_
#define INCLUDE_VISUAL_LOCALIZATION_CONVERSION_H_

#include <Eigen/Core>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <vector>

inline std::vector<cv::KeyPoint> Vector2dToKeypoints(
    const std::vector<Eigen::Vector2d>& points, const float size = 0.) {
  std::vector<cv::KeyPoint> keypoints(points.size());
  std::transform(points.begin(), points.end(), keypoints.begin(),
                 [&](const Eigen::Vector2d& point) {
                   const auto x = static_cast<float>(point(0));
                   const auto y = static_cast<float>(point(1));
                   return cv::KeyPoint(x, y, size);
                 });
  return keypoints;
}

inline std::vector<cv::KeyPoint> Point2fToKeypoints(
    const std::vector<cv::Point2f>& points, const float size = 0.) {
  std::vector<cv::KeyPoint> keypoints(points.size());
  std::transform(points.begin(), points.end(), keypoints.begin(),
                 [&](const cv::Point2f& point) {
                   return cv::KeyPoint(point.x, point.y, size);
                 });
  return keypoints;
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_CONVERSION_H_
