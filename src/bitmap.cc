// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/colmap/bitmap.h"

#include <fmt/core.h>

colmap::Bitmap ReadBitmap(const std::string& path, const bool as_rgb) {
  colmap::Bitmap bitmap;
  if (!bitmap.Read(path, as_rgb)) {
    throw std::runtime_error(
        fmt::format("Error reading file from path {}", path));
  }
  return bitmap;
}
