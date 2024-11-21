// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_COLMAP_BITMAP_H_
#define INCLUDE_VISUAL_LOCALIZATION_COLMAP_BITMAP_H_

#include <string>

#include "colmap/sensor/bitmap.h"

colmap::Bitmap ReadBitmap(const std::string& path, bool as_rgb = true);

#endif  // INCLUDE_VISUAL_LOCALIZATION_COLMAP_BITMAP_H_
