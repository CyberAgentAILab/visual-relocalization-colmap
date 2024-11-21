// Copyright [2024] <CyberAgent AI Lab>

#ifndef INCLUDE_VISUAL_LOCALIZATION_VECTOR_SEARCH_H_
#define INCLUDE_VISUAL_LOCALIZATION_VECTOR_SEARCH_H_

#include <optional>
#include <vector>

template <typename T>
std::optional<std::size_t> FindIndexInSorted(const std::vector<T>& vector,
                                             const T element) {
  const auto lower = std::lower_bound(vector.begin(), vector.end(), element);
  if (lower == vector.end() || *lower != element) {
    return std::nullopt;
  }
  return std::distance(vector.begin(), lower);
}

#endif  // INCLUDE_VISUAL_LOCALIZATION_VECTOR_SEARCH_H_
