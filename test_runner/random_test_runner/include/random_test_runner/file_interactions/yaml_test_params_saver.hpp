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

#ifndef RANDOM_TEST_RUNNER__YAML_SCENARIO_SAVER_HPP
#define RANDOM_TEST_RUNNER__YAML_SCENARIO_SAVER_HPP

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <rclcpp/logger.hpp>

#include "random_test_runner/data_types.hpp"
#include "test_suite_parameters.hpp"
#include "spdlog/fmt/fmt.h"

namespace YAML
{
template <>
struct convert<random_test_runner::Params::TestSuite>
{
  static Node encode(const random_test_runner::Params::TestSuite & rhs)
  {
    Node node;
    node["test_name"] = rhs.test_name;
    node["map_name"] = rhs.map_name;
    node["ego_goal_s"] = rhs.ego_goal_s;
    node["ego_goal_lanelet_id"] = rhs.ego_goal_lanelet_id;
    node["ego_goal_partial_randomization"] = rhs.ego_goal_partial_randomization;
    node["ego_goal_partial_randomization_distance"] = rhs.ego_goal_partial_randomization_distance;
    node["npc_vehicle_count"] = rhs.npc_vehicle_count;
    node["npc_vehicle_min_speed"] = rhs.npc_vehicle_min_speed;
    node["npc_vehicle_max_speed"] = rhs.npc_vehicle_max_speed;
    node["npc_vehicle_min_spawn_distance_from_ego"] = rhs.npc_vehicle_min_spawn_distance_from_ego;
    node["npc_vehicle_max_spawn_distance_from_ego"] = rhs.npc_vehicle_max_spawn_distance_from_ego;
    node["npc_pedestrian_count"] = rhs.npc_pedestrian_count;
    node["npc_pedestrian_planner"] = rhs.npc_pedestrian_planner;
    node["npc_pedestrian_min_speed"] = rhs.npc_pedestrian_min_speed;
    node["npc_pedestrian_max_speed"] = rhs.npc_pedestrian_max_speed;
    node["npc_pedestrian_behavior_static"] = rhs.npc_pedestrian_behavior_static;
    node["npc_pedestrian_behavior_crosswalk"] = rhs.npc_pedestrian_behavior_crosswalk;
    node["npc_pedestrian_behavior_walk_along_lane"] = rhs.npc_pedestrian_behavior_walk_along_lane;
    node["npc_pedestrian_lanelet_min_offset"] = rhs.npc_pedestrian_lanelet_min_offset;
    node["npc_pedestrian_lanelet_min_offset"] = rhs.npc_pedestrian_lanelet_min_offset;
    return node;
  }

  static bool decode(const Node & node, random_test_runner::Params::TestSuite & rhs)
  {
    if (!node.IsMap()) {
      return false;
    }

    rhs.test_name = node["test_name"].as<std::string>();
    rhs.map_name = node["map_name"].as<std::string>();
    rhs.ego_goal_s = node["ego_goal_s"].as<double>();
    rhs.ego_goal_lanelet_id = node["ego_goal_lanelet_id"].as<int64_t>();
    rhs.ego_goal_partial_randomization = node["ego_goal_partial_randomization"].as<bool>();
    rhs.ego_goal_partial_randomization_distance =
      node["ego_goal_partial_randomization_distance"].as<double>();
    rhs.npc_vehicle_count = node["npc_vehicle_count"].as<int64_t>();
    rhs.npc_vehicle_min_speed = node["npc_vehicle_min_speed"].as<double>();
    rhs.npc_vehicle_max_speed = node["npc_vehicle_max_speed"].as<double>();
    rhs.npc_vehicle_min_spawn_distance_from_ego =
      node["npc_vehicle_min_spawn_distance_from_ego"].as<double>();
    rhs.npc_vehicle_max_spawn_distance_from_ego =
      node["npc_vehicle_max_spawn_distance_from_ego"].as<double>();
    rhs.npc_pedestrian_count = node["npc_pedestrian_count"].as<int64_t>();
    rhs.npc_pedestrian_planner = node["npc_pedestrian_planner"].as<std::string>();
    rhs.npc_pedestrian_min_speed = node["npc_pedestrian_min_speed"].as<double>();
    rhs.npc_pedestrian_max_speed = node["npc_pedestrian_max_speed"].as<double>();
    rhs.npc_pedestrian_behavior_static = node["npc_pedestrian_behavior_static"].as<bool>();
    rhs.npc_pedestrian_behavior_crosswalk = node["npc_pedestrian_behavior_crosswalk"].as<bool>();
    rhs.npc_pedestrian_behavior_walk_along_lane =
      node["npc_pedestrian_behavior_walk_along_lane"].as<bool>();
    rhs.npc_pedestrian_lanelet_min_offset = node["npc_pedestrian_lanelet_min_offset"].as<double>();
    rhs.npc_pedestrian_lanelet_max_offset = node["npc_pedestrian_lanelet_max_offset"].as<double>();

    return true;
  }
};

template <>
struct convert<TestCaseParameters>
{
  static Node encode(const TestCaseParameters & rhs)
  {
    Node node;
    node["seed"] = rhs.seed;
    return node;
  }

  static bool decode(const Node & node, TestCaseParameters & rhs)
  {
    if (!node.IsMap()) {
      return false;
    }
    rhs.seed = node["seed"].as<int64_t>();
    return true;
  }
};
}  // namespace YAML

class YamlTestParamsIO
{
public:
  YamlTestParamsIO(const rclcpp::Logger & logger, std::string data_directory)
  : data_directory_(std::move(data_directory)), logger_(logger)
  {
  }

  void addTestSuite(const random_test_runner::Params::TestSuite & test_parameters, const std::string & description)
  {
    yaml_[description] = test_parameters;
  }

  void addTestCase(const TestCaseParameters & test_parameters, const std::string & description)
  {
    yaml_[description]["test_cases"].push_back(test_parameters);
  }

  void write()
  {
    std::string filepath = data_directory_ + "/result.yaml";
    std::string message = fmt::format("Saving yaml: {}", filepath);
    RCLCPP_INFO_STREAM(logger_, message);
    std::ofstream result_file(filepath);
    result_file << yaml_;
  }

  std::pair<random_test_runner::Params::TestSuite, std::vector<TestCaseParameters>> read()
  {
    std::string filepath = data_directory_ + "/result.yaml";
    std::string message = fmt::format("Reading yaml: {}", filepath);
    RCLCPP_INFO_STREAM(logger_, message);
    yaml_ = YAML::LoadFile(filepath);

    if (yaml_.size() != 1) {
      throw std::runtime_error(
        "yaml file either empty or has too many test "
        "suites (currently only one is supported)");
    }

    random_test_runner::Params::TestSuite test_suite_parameters = yaml_.begin()->second.as<random_test_runner::Params::TestSuite>();
    std::vector<TestCaseParameters> test_cases_parameters;
    auto test_cases_yaml = yaml_.begin()->second["test_cases"];
    for (YAML::iterator test_case_it = test_cases_yaml.begin();
         test_case_it != test_cases_yaml.end(); ++test_case_it) {
      test_cases_parameters.emplace_back(test_case_it->as<TestCaseParameters>());
    }

    return {test_suite_parameters, std::move(test_cases_parameters)};
  }

private:
  std::string data_directory_;
  YAML::Node yaml_;
  rclcpp::Logger logger_;
};

#endif  // RANDOM_TEST_RUNNER__YAML_SCENARIO_SAVER_HPP
