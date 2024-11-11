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

#include <gtest/gtest.h>

#include <traffic_simulator/job/job.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @note Test basic functionality. Test functions execution correctness with a function on update
 * that returns true and function on cleanup that marks that it was called
 * - the goal is to check whether the function on cleanup is called
 * after calling onUpdate (which immediately finishes).
 */
TEST(Job, onUpdate)
{
  bool was_cleanup_func_called = false;

  auto job = traffic_simulator::job::Job(
    [](const double) { return true; },
    [&was_cleanup_func_called]() { was_cleanup_func_called = true; },
    traffic_simulator::job::Type::UNKOWN, true, traffic_simulator::job::Event::POST_UPDATE);
  job.onUpdate(0.0);

  EXPECT_TRUE(was_cleanup_func_called);
}
