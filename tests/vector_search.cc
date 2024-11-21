// Copyright [2024] <CyberAgent AI Lab>

#include "visual_localization/vector_search.h"

#include <gtest/gtest.h>

TEST(FindIndexInSorted, SmokeTest) {
  const std::vector<int> v = {2, 4, 5, 7, 8, 9};

  ASSERT_TRUE(FindIndexInSorted(v, 2).has_value());
  ASSERT_TRUE(FindIndexInSorted(v, 4).has_value());
  ASSERT_TRUE(FindIndexInSorted(v, 5).has_value());
  ASSERT_TRUE(FindIndexInSorted(v, 7).has_value());
  ASSERT_TRUE(FindIndexInSorted(v, 8).has_value());
  ASSERT_TRUE(FindIndexInSorted(v, 9).has_value());

  EXPECT_EQ(FindIndexInSorted(v, 2).value(), 0);
  EXPECT_EQ(FindIndexInSorted(v, 4).value(), 1);
  EXPECT_EQ(FindIndexInSorted(v, 5).value(), 2);
  EXPECT_EQ(FindIndexInSorted(v, 7).value(), 3);
  EXPECT_EQ(FindIndexInSorted(v, 8).value(), 4);
  EXPECT_EQ(FindIndexInSorted(v, 9).value(), 5);

  ASSERT_FALSE(FindIndexInSorted(v, 0).has_value());
  ASSERT_FALSE(FindIndexInSorted(v, 1).has_value());
  ASSERT_FALSE(FindIndexInSorted(v, 3).has_value());
  ASSERT_FALSE(FindIndexInSorted(v, 6).has_value());
}
