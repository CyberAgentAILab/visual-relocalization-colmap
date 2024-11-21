// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/colmap/cv_colmap.h"

std::vector<cv::DMatch> ToCvDMatch(const colmap::FeatureMatches& matches) {
  std::vector<cv::DMatch> cv_matches;
  std::transform(matches.begin(), matches.end(), std::back_inserter(cv_matches),
                 [](const colmap::FeatureMatch& m) {
                   return cv::DMatch(m.point2D_idx1, m.point2D_idx2, 0.);
                 });
  return cv_matches;
}

std::vector<cv::KeyPoint> ToCvKeypoints(
    const colmap::FeatureKeypoints& keypoints) {
  std::vector<cv::KeyPoint> cv_keypoints;
  std::transform(keypoints.begin(), keypoints.end(),
                 std::back_inserter(cv_keypoints),
                 [](const colmap::FeatureKeypoint& p) {
                   return cv::KeyPoint(p.x, p.y, 0.);
                 });
  return cv_keypoints;
}
