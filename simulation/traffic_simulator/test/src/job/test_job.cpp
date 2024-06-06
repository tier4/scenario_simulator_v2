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
 * @note 
 */
TEST(Job, onUpdate)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);
  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(was_cleanup_func_called);
}

/**
 * @note 
 */
TEST(Job, get_type)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.type == type);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.type == type);
}

/**
 * @note 
 */
TEST(Job, get_exclusive)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.exclusive == is_exclusive);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.exclusive == is_exclusive);
}

/**
 * @note 
 */
TEST(Job, get_event)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.event == event);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.event == event);
}
