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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>
#include <vector>

using namespace vehicle_simulation;

TEST(EgoEntitySimulation, calculateAccelerationBySlope)
{
  // initialize rclcpp for rosparam in EgoEntitySimulation class
  rclcpp::init(0, nullptr);

  const auto lanelet_path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                            "/map/slope/lanelet2_map.osm";
  traffic_simulator::lanelet_map::activate(lanelet_path);

  constexpr double gravity_acceleration = -9.81;
  // expected value in the lanelet(id:7)
  // first 25m: 1m up
  constexpr double expected_slope_acceleration_first_25m =
    -std::sin(-std::atan2(1., 25.)) * gravity_acceleration;
  EXPECT_LE(expected_slope_acceleration_first_25m, 0.0);  // up -> negative slope acceleration
  // last 25m:  4m up
  constexpr double expected_slope_acceleration_last_25m =
    -std::sin(-std::atan2(4., 25.)) * gravity_acceleration;
  EXPECT_LE(expected_slope_acceleration_last_25m, 0.0);  // up -> negative slope acceleration

  auto get_slope_acceleration_at =
    [&](const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose, bool consider_slope) {
      traffic_simulator_msgs::msg::EntityStatus initial_status;
      initial_status.name = "ego";
      // use pitch-filled map pose
      initial_status.lanelet_pose_valid = false;
      initial_status.pose =
        traffic_simulator::lanelet_wrapper::pose::toMapPose(lanelet_pose, true).pose;

      EgoEntitySimulation ego_entity_simulation(
        initial_status, traffic_simulator_msgs::msg::VehicleParameters(), 1.f / 30.f,
        rclcpp::Parameter("use_sim_time", false), consider_slope);
      return ego_entity_simulation.calculateAccelerationBySlope();
    };

  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = 7;

  // This value is determined manually by @Hans_Robo.
  // Since it is affected by the calculation algorithm inside lanelet2, etc.,
  // it will not exactly match the ideal value, so we manually selected the smallest possible value specifically for this test.
  constexpr double compare_epsilon = 1e-7;

  // first 25m, up
  {
    lanelet_pose.s = 12.5;
    lanelet_pose.rpy.z = 0.0;

    // with considering slope
    EXPECT_NEAR(
      expected_slope_acceleration_first_25m, get_slope_acceleration_at(lanelet_pose, true),
      compare_epsilon);

    // without considering slope
    EXPECT_DOUBLE_EQ(0.0, get_slope_acceleration_at(lanelet_pose, false));
  }

  // last 25m, up
  {
    lanelet_pose.s = 37.5;
    lanelet_pose.rpy.z = 0.0;

    // with considering slope
    EXPECT_NEAR(
      expected_slope_acceleration_last_25m, get_slope_acceleration_at(lanelet_pose, true),
      compare_epsilon);

    // without considering slope
    EXPECT_DOUBLE_EQ(0.0, get_slope_acceleration_at(lanelet_pose, false));
  }

  // first 25m, down
  {
    lanelet_pose.s = 12.5;
    lanelet_pose.rpy.z = M_PI;

    // with considering slope
    EXPECT_NEAR(
      -expected_slope_acceleration_first_25m, get_slope_acceleration_at(lanelet_pose, true),
      compare_epsilon);

    // without considering slope
    EXPECT_DOUBLE_EQ(0.0, get_slope_acceleration_at(lanelet_pose, false));
  }

  // last 25m, down
  {
    lanelet_pose.s = 37.5;
    lanelet_pose.rpy.z = M_PI;

    // with considering slope
    EXPECT_NEAR(
      -expected_slope_acceleration_last_25m, get_slope_acceleration_at(lanelet_pose, true),
      compare_epsilon);

    // without considering slope
    EXPECT_DOUBLE_EQ(0.0, get_slope_acceleration_at(lanelet_pose, false));
  }

  rclcpp::shutdown();

  // EXPECT_NEAR(ego_entity_simulation.);
}
