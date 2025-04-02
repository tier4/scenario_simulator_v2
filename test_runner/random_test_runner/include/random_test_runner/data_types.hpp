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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__DATA_TYPES_HPP
#define RANDOM_TEST_RUNNER__DATA_TYPES_HPP

#include <numeric>
#include <ostream>
#include <traffic_simulator/data_type/entity_status.hpp>

#include "spdlog/fmt/fmt.h"
#include "traffic_simulator_msgs/msg/action_status.hpp"
#include "traffic_simulator_msgs/msg/entity_status.hpp"
#include "traffic_simulator_msgs/msg/lanelet_pose.hpp"

struct NPCDescription
{
  traffic_simulator_msgs::msg::LaneletPose start_position;
  double speed;
  std::string name;
};

struct TestDescription
{
  traffic_simulator_msgs::msg::LaneletPose ego_goal_position;
  geometry_msgs::msg::Pose ego_goal_pose;
  traffic_simulator_msgs::msg::LaneletPose ego_start_position;

  std::vector<NPCDescription> npcs_descriptions;
};

enum RandomTestType { RANDOM_RUN, REPLAY };

enum SimulatorType { SIMPLE_SENSOR_SIMULATOR, AWSIM };
SimulatorType simulatorTypeFromString(const std::string & simulator_type_str);

enum ArchitectureType { AWF_AUTO, AWF_UNIVERSE, TIER4_PROPOSAL };
ArchitectureType architectureTypeFromString(const std::string & architecture_type_str);
std::string stringFromArchitectureType(const ArchitectureType architecture_type);

struct TestControlParameters
{
  std::string input_dir;
  std::string output_dir = "/tmp";
  RandomTestType random_test_type = RandomTestType::RANDOM_RUN;
  int64_t test_count = 5;
  SimulatorType simulator_type = SimulatorType::SIMPLE_SENSOR_SIMULATOR;
  ArchitectureType architecture_type = ArchitectureType::AWF_UNIVERSE;
  std::string simulator_host = "localhost";
  double test_timeout = 60.0;
};

struct TestSuiteParameters
{
  std::string name = "random_test";
  std::string map_name = "kashiwanoha_map";

  int64_t ego_goal_lanelet_id = -1;
  double ego_goal_s = 0.0;
  bool ego_goal_partial_randomization = false;
  double ego_goal_partial_randomization_distance = 30.0;

  int64_t npcs_count = 10;
  double npc_min_speed = 0.5;
  double npc_max_speed = 3.0;
  double npc_min_spawn_distance_from_ego = 10.0;
  double npc_max_spawn_distance_from_ego = 100.0;
};

struct TestCaseParameters
{
  int64_t seed = -1;
};

#define DEFINE_FMT_FORMATTER(type, format_str, ...)              \
  template <>                                                    \
  struct fmt::formatter<type>                                    \
  {                                                              \
    template <typename ParseContext>                             \
    constexpr auto parse(ParseContext & ctx)                     \
    {                                                            \
      return ctx.begin();                                        \
    }                                                            \
    template <typename FormatContext>                            \
    auto format(const type & v, FormatContext & ctx)             \
    {                                                            \
      return fmt::format_to(ctx.out(), format_str, __VA_ARGS__); \
    }                                                            \
  };

DEFINE_FMT_FORMATTER(geometry_msgs::msg::Vector3, "(x, y, z) : ({}, {}, {})", v.x, v.y, v.z)

DEFINE_FMT_FORMATTER(
  traffic_simulator_msgs::msg::LaneletPose, "lanelet_id: {}, s: {}, offset: {}, rpy: {}",
  v.lanelet_id, v.s, v.offset, v.rpy)

DEFINE_FMT_FORMATTER(geometry_msgs::msg::Twist, "linear: {}, angular: {}", v.linear, v.angular)

DEFINE_FMT_FORMATTER(geometry_msgs::msg::Accel, "linear: {}, angular: {}", v.linear, v.angular)

DEFINE_FMT_FORMATTER(geometry_msgs::msg::Point, "(x, y, z), ({}, {}, {})", v.x, v.y, v.z)

DEFINE_FMT_FORMATTER(
  geometry_msgs::msg::Quaternion, "(x, y, z, w), ({}, {}, {}, {})", v.x, v.y, v.z, v.w)

DEFINE_FMT_FORMATTER(
  geometry_msgs::msg::Pose, "position: {}, orientation {}", v.position, v.orientation)

DEFINE_FMT_FORMATTER(
  traffic_simulator_msgs::msg::ActionStatus, "current_action: {}, twist: {}, accel: {}",
  v.current_action, v.twist, v.accel)

DEFINE_FMT_FORMATTER(
  traffic_simulator::EntityStatus,
  "name {}, lanelet_pose: {}, pose: {}, action_status:{}"
  "time {}: lanelet_pose_valid: {}, type: {}",
  v.name, v.lanelet_pose, v.pose, v.action_status, v.time, v.lanelet_pose_valid, v.type)

DEFINE_FMT_FORMATTER(
  TestControlParameters,
  "input dir: {} output dir: {} random test type: {} test count {} test_timeout {}", v.input_dir,
  v.output_dir, v.random_test_type, v.test_count, v.test_timeout)

DEFINE_FMT_FORMATTER(
  TestSuiteParameters,
  "ego_goal_lanelet_id: {} ego_goal_s: {} ego_goal_partial_randomization: {} "
  "ego_goal_partial_randomization_distance: {} npcs_count: {} npc_min_speed: "
  "{} "
  "npc_max_speed: {} npc_min_spawn_distance_from_ego: {} "
  "npc_max_spawn_distance_from_ego: {} "
  "name: {} map_name: {}",
  v.ego_goal_lanelet_id, v.ego_goal_s, v.ego_goal_partial_randomization,
  v.ego_goal_partial_randomization_distance, v.npcs_count, v.npc_min_speed, v.npc_max_speed,
  v.npc_min_spawn_distance_from_ego, v.npc_max_spawn_distance_from_ego, v.name, v.map_name)

DEFINE_FMT_FORMATTER(TestCaseParameters, "seed: {}", v.seed)

DEFINE_FMT_FORMATTER(
  NPCDescription, "name: {}, start_position: {}, speed: {}", v.name, v.start_position, v.speed)

template <>
struct fmt::formatter<TestDescription>
{
  template <typename ParseContext>
  constexpr auto parse(ParseContext & ctx)
  {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const TestDescription & v, FormatContext & ctx)
  {
    fmt::format_to(
      ctx.out(),
      "ego_start_position: {} ego_goal_position: {} "
      "ego_goal_pose: {}\n"  // The spell checker detects the string as an illegal word, so it is necessary to split the string like this.
      "npc_descriptions:",
      v.ego_start_position, v.ego_goal_position, v.ego_goal_pose);
    for (size_t idx = 0; idx < v.npcs_descriptions.size(); idx++) {
      fmt::format_to(ctx.out(), "[{}]: {}\n", idx, v.npcs_descriptions[idx]);
    }
    return ctx.out();
  }
};

#endif  // RANDOM_TEST_RUNNER__DATA_TYPES_HPP
