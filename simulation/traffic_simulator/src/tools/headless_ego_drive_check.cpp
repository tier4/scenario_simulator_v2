// Copyright 2025 TIER IV, Inc. All rights reserved.
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

// R1c standalone driving check + Phase 0b measurement (SSV2_HEADLESS_EGO only).
//
// Builds a single-process, standalone (no ZeroMQ, no simple_sensor_simulator) traffic_simulator
// API, spawns a headless EgoEntity on the kashiwanoha map, injects a straight Diffusion-Planner
// trajectory, and steps api.updateFrame(). It verifies the ego advances along the trajectory and
// times the per-step cost (sim.step budget for plan/04). This is a manual check executable, not a
// gtest, so it does not pull in the whole traffic_simulator test suite.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/data_type/behavior.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <vector>

namespace
{
auto makeVehicleParameters() -> traffic_simulator_msgs::msg::VehicleParameters
{
  traffic_simulator_msgs::msg::VehicleParameters p;
  p.name = "headless_ego";
  p.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  p.performance.max_speed = 50.0;
  p.performance.max_acceleration = 10.0;
  p.performance.max_deceleration = 10.0;
  p.bounding_box.center.x = 1.5;
  p.bounding_box.center.z = 0.9;
  p.bounding_box.dimensions.x = 4.5;
  p.bounding_box.dimensions.y = 2.1;
  p.bounding_box.dimensions.z = 1.8;
  p.axles.front_axle.max_steering = 0.5;
  p.axles.front_axle.wheel_diameter = 0.6;
  p.axles.front_axle.track_width = 1.8;
  p.axles.front_axle.position_x = 3.1;
  p.axles.front_axle.position_z = 0.3;
  p.axles.rear_axle.max_steering = 0.0;
  p.axles.rear_axle.wheel_diameter = 0.6;
  p.axles.rear_axle.track_width = 1.8;
  p.axles.rear_axle.position_x = 0.0;
  p.axles.rear_axle.position_z = 0.3;
  return p;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("headless_ego_drive_check");

  const auto map_path = ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map";
  traffic_simulator::Configuration configuration(map_path, "lanelet2_map.osm", __FILE__);
  configuration.standalone_mode = true;

  constexpr double frame_rate = 10.0;
  constexpr double step_time = 1.0 / frame_rate;
  traffic_simulator::API api(node.get(), configuration, 1.0 /* realtime_factor */, frame_rate);

  const auto spawn_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(120545, 0.0, 0.0);
  api.spawn(
    "ego", spawn_pose, makeVehicleParameters(), traffic_simulator::VehicleBehavior::autoware());
  auto & ego = api.getEgoEntity("ego");
  api.updateFrame();

  // Capture the spawn map pose and its heading, then build a straight trajectory ahead of the ego.
  const auto start_pose = ego.getMapPose();
  const double yaw = math::geometry::convertQuaternionToEulerAngle(start_pose.orientation).z;
  constexpr double target_speed = 3.0;
  autoware_planning_msgs::msg::Trajectory trajectory;
  for (std::size_t i = 0; i < 60; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint tp;
    tp.pose.position.x = start_pose.position.x + static_cast<double>(i) * std::cos(yaw);
    tp.pose.position.y = start_pose.position.y + static_cast<double>(i) * std::sin(yaw);
    tp.pose.position.z = start_pose.position.z;
    tp.pose.orientation = start_pose.orientation;
    tp.longitudinal_velocity_mps = static_cast<float>(target_speed);
    trajectory.points.push_back(tp);
  }
  ego.setDiffusionTrajectory(rclcpp::Time(1, 0), trajectory);

  api.startNpcLogic();

  constexpr int steps = 50;
  double total_step_ms = 0.0;
  double max_step_ms = 0.0;
  for (int i = 0; i < steps; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    api.updateFrame();
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_step_ms += ms;
    max_step_ms = std::max(max_step_ms, ms);
  }

  const auto end_pose = ego.getMapPose();
  const double dx = end_pose.position.x - start_pose.position.x;
  const double dy = end_pose.position.y - start_pose.position.y;
  const double travelled = std::hypot(dx, dy);
  const double expected = target_speed * step_time * steps;

  std::printf("=== R1c headless ego drive check ===\n");
  std::printf("start: (%.3f, %.3f, %.3f) yaw=%.3f\n",
    start_pose.position.x, start_pose.position.y, start_pose.position.z, yaw);
  std::printf("end:   (%.3f, %.3f, %.3f)\n",
    end_pose.position.x, end_pose.position.y, end_pose.position.z);
  std::printf("travelled=%.3f m (expected ~%.3f m over %d steps @ %.1f m/s)\n",
    travelled, expected, steps, target_speed);
  std::printf("ego vx=%.3f m/s\n", ego.getCurrentTwist().linear.x);
  std::printf("[Phase 0b] sim.step: mean=%.3f ms  max=%.3f ms  (%d steps)\n",
    total_step_ms / steps, max_step_ms, steps);

  const bool moved_forward = travelled > 0.5 * expected;
  std::printf("RESULT: %s\n", moved_forward ? "PASS (ego advanced along trajectory)" : "FAIL");

  rclcpp::shutdown();
  return moved_forward ? 0 : 1;
}
