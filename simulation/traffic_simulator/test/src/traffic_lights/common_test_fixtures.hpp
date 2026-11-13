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

#ifndef TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__COMMON_TEST_FIXTURES_HPP_
#define TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__COMMON_TEST_FIXTURES_HPP_

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>

constexpr char architecture_old[] = "awf/universe/20230906";
constexpr char architecture_new[] = "awf/universe/20240605";

template <typename TrafficLightsT, const char * Architecture>
class TrafficLightsInternalTestArchitectureDependent : public testing::Test
{
public:
  explicit TrafficLightsInternalTestArchitectureDependent()
  : lights([this] {
      if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::ConventionalTrafficLights>) {
        return std::make_unique<TrafficLightsT>(node_ptr);
      } else if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::V2ITrafficLights>) {
        return std::make_unique<TrafficLightsT>(node_ptr, Architecture);
      }
    }())
  {
    static_assert(
      std::is_same_v<TrafficLightsT, traffic_simulator::ConventionalTrafficLights> or
        std::is_same_v<TrafficLightsT, traffic_simulator::V2ITrafficLights>,
      "Given TrafficLights type is not supported");

    const auto lanelet_path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                              "/map/standard_map/lanelet2_map.osm";
    traffic_simulator::lanelet_map::activate(lanelet_path);

    executor.add_node(node_ptr);
  }

  const lanelet::Id id{34836};

  const lanelet::Id signal_id{34806};

  const rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("TrafficLightsInternalTest");

  rclcpp::executors::SingleThreadedExecutor executor;

  std::unique_ptr<TrafficLightsT> lights;
};

template <typename TrafficLightsT>
class TrafficLightsInternalTest
: public TrafficLightsInternalTestArchitectureDependent<TrafficLightsT, architecture_old>
{
};

template <typename TrafficLightsT>
class TrafficLightsInternalTestNewArchitecture
: public TrafficLightsInternalTestArchitectureDependent<TrafficLightsT, architecture_new>
{
};

struct TrafficLightsNameGenerator
{
  template <typename TrafficLightsT>
  static auto GetName(int) -> std::string
  {
    if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::ConventionalTrafficLights>) {
      return "ConventionalTrafficLights";
    } else if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::V2ITrafficLights>) {
      return "V2ITrafficLights";
    }
  }
};
#endif  // TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__COMMON_TEST_FIXTURES_HPP_
