// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_grid_traversal.hpp"

/**
 * @note Test basic functionality. Test first operator obtaining correctness.
 */
TEST_F(GridTraversalTest, begin)
{
  auto it = traversal_->begin();

  ASSERT_TRUE(it != traversal_->end());
  const auto [x, y] = *it;

  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);
}

/**
 * @note Test basic functionality. Test sentinel obtaining correctness.
 */
TEST_F(GridTraversalTest, end)
{
  auto it = traversal_->begin();
  while (it != traversal_->end()) {
    ++it;
  }
  EXPECT_FALSE(it != traversal_->end());
}

/**
 * @note Test basic functionality. Test iterator dereferencing correcntess with a sample iterator.
 */
TEST_F(GridTraversalTest, iterator_operator_dereference)
{
  auto it = traversal_->begin();
  ASSERT_TRUE(it != traversal_->end());
  const auto [x, y] = *it;
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);

  ++it;
  ASSERT_TRUE(it != traversal_->end());
  const auto [x1, y1] = *it;
  EXPECT_EQ(x1, 0);
  EXPECT_EQ(y1, 1);
}

/**
 * @note Test basic functionality. Test iterator incrementing with a sample iterator.
 */
TEST_F(GridTraversalTest, iterator_operator_increment)
{
  auto it = traversal_->begin();
  ASSERT_TRUE(it != traversal_->end());

  ++it;
  ASSERT_TRUE(it != traversal_->end());
  const auto [x1, y1] = *it;
  EXPECT_EQ(x1, 0);
  EXPECT_EQ(y1, 1);

  ++it;
  ASSERT_TRUE(it != traversal_->end());
  const auto [x2, y2] = *it;
  EXPECT_EQ(x2, 1);
  EXPECT_EQ(y2, 1);

  ++it;
  ASSERT_TRUE(it != traversal_->end());
  const auto [x3, y3] = *it;
  EXPECT_EQ(x3, 1);
  EXPECT_EQ(y3, 2);
}

/**
 * @note Test basic functionality. Test iterator comparison with a sentinel correctness.
 */
TEST_F(GridTraversalTest, iterator_operator_notEqual)
{
  auto it = traversal_->begin();
  ASSERT_TRUE(it != traversal_->end());
  ++it;
  ASSERT_TRUE(it != traversal_->end());
  while (it != traversal_->end()) {
    ++it;
  }
  EXPECT_FALSE(it != traversal_->end());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
