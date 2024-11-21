// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/colmap/feature.h"

FeatureExtractor::FeatureExtractor()
    : extractor_(
          colmap::CreateSiftFeatureExtractor(InitSiftExtractionOptions())) {
  if (extractor_ == nullptr) {
    throw std::runtime_error("Failed to initialize the feature extractor");
  }
}

std::pair<std::shared_ptr<colmap::FeatureKeypoints>,
          std::shared_ptr<colmap::FeatureDescriptors>>
FeatureExtractor::Extract(const colmap::Bitmap& bitmap) {
  const colmap::Bitmap& grey_bitmap =
      bitmap.IsGrey() ? bitmap : bitmap.CloneAsGrey();

  auto keypoints = std::make_shared<colmap::FeatureKeypoints>();
  auto descriptors = std::make_shared<colmap::FeatureDescriptors>();

  extractor_->Extract(grey_bitmap, keypoints.get(), descriptors.get());
  return {keypoints, descriptors};
}

FeatureMatcher::FeatureMatcher()
    : matcher_(colmap::CreateSiftFeatureMatcher(InitSiftMatchingOptions())) {}

colmap::FeatureMatches FeatureMatcher::Match(
    const std::shared_ptr<colmap::FeatureDescriptors>& descriptors1,
    const std::shared_ptr<colmap::FeatureDescriptors>& descriptors2) {
  colmap::FeatureMatches matches12;
  matcher_->Match(descriptors1, descriptors2, &matches12);
  return matches12;
}
