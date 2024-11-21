// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_COLMAP_CV_COLMAP_H_
#define INCLUDE_VISUAL_LOCALIZATION_COLMAP_CV_COLMAP_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "colmap/feature/types.h"

std::vector<cv::DMatch> ToCvDMatch(const colmap::FeatureMatches& matches);
std::vector<cv::KeyPoint> ToCvKeypoints(
    const colmap::FeatureKeypoints& keypoints);

#endif  // INCLUDE_VISUAL_LOCALIZATION_COLMAP_CV_COLMAP_H_
