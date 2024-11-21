// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_COLMAP_FEATURE_H_
#define INCLUDE_VISUAL_LOCALIZATION_COLMAP_FEATURE_H_

#include <memory>
#include <utility>

#include "colmap/feature/sift.h"

inline colmap::SiftExtractionOptions InitSiftExtractionOptions() {
  // Disable GPU based extraction, otherwise the extractor can be nullptr
  colmap::SiftExtractionOptions options;
  options.use_gpu = false;
  return options;
}

class FeatureExtractor {
 public:
  FeatureExtractor();

  std::pair<std::shared_ptr<colmap::FeatureKeypoints>,
            std::shared_ptr<colmap::FeatureDescriptors>>
  Extract(const colmap::Bitmap& bitmap);

 private:
  const std::unique_ptr<colmap::FeatureExtractor> extractor_;
};

inline colmap::SiftMatchingOptions InitSiftMatchingOptions() {
  // Disable GPU based matching, otherwise the matcher can be nullptr
  colmap::SiftMatchingOptions options;
  options.use_gpu = false;
  return options;
}

class FeatureMatcher {
 public:
  FeatureMatcher();

  colmap::FeatureMatches Match(
      const std::shared_ptr<colmap::FeatureDescriptors>& descriptors1,
      const std::shared_ptr<colmap::FeatureDescriptors>& descriptors2);

 private:
  const std::unique_ptr<colmap::FeatureMatcher> matcher_;
};

#endif  // INCLUDE_VISUAL_LOCALIZATION_COLMAP_FEATURE_H_
