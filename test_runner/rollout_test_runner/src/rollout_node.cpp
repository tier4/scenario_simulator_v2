// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include <iomanip>
#include <rollout_test_runner/rollout_node.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace rollout_test_runner
{
namespace
{
auto defaultVehicleParameters() -> traffic_simulator_msgs::msg::VehicleParameters
{
  traffic_simulator_msgs::msg::VehicleParameters parameters;
  parameters.name = "ego";
  parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  parameters.performance.max_speed = 69.444;
  parameters.performance.max_acceleration = 200;
  parameters.performance.max_deceleration = 10.0;
  parameters.bounding_box.center.x = 1.5;
  parameters.bounding_box.center.y = 0.0;
  parameters.bounding_box.center.z = 0.9;
  parameters.bounding_box.dimensions.x = 4.5;
  parameters.bounding_box.dimensions.y = 2.1;
  parameters.bounding_box.dimensions.z = 1.8;
  parameters.axles.front_axle.max_steering = 0.5;
  parameters.axles.front_axle.wheel_diameter = 0.6;
  parameters.axles.front_axle.track_width = 1.8;
  parameters.axles.front_axle.position_x = 3.1;
  parameters.axles.front_axle.position_z = 0.3;
  parameters.axles.rear_axle.max_steering = 0.0;
  parameters.axles.rear_axle.wheel_diameter = 0.6;
  parameters.axles.rear_axle.track_width = 1.8;
  parameters.axles.rear_axle.position_x = 0.0;
  parameters.axles.rear_axle.position_z = 0.3;
  return parameters;
}
}  // namespace

RolloutNode::RolloutNode(const rclcpp::NodeOptions & option)
: Node("rollout_test_runner", option),
  ego_replay_source_(
    declare_parameter<std::string>("bag_path", ""),
    declare_parameter<std::string>("odometry_topic", "/localization/kinematic_state"),
    declare_parameter<std::string>("acceleration_topic", "/localization/acceleration")),
  bag_anchor_seconds_(
    ego_replay_source_.startTime().seconds() +
    declare_parameter<double>("replay_start_offset", 0.0)),
  api_(
    this, configure(), declare_parameter<double>("global_real_time_factor", 1.0),
    declare_parameter<double>("global_frame_rate", 20.0), bag_anchor_seconds_),
  passthrough_(
    *this, get_parameter("bag_path").as_string(),
    declare_parameter<std::vector<std::string>>("passthrough_topics", std::vector<std::string>()),
    declare_parameter<std::vector<std::string>>(
      "transient_local_topics", std::vector<std::string>())),
  on_bag_end_(declare_parameter<std::string>("on_bag_end", "hold")),
  auto_switch_time_(declare_parameter<double>("auto_switch_time", -1.0)),
  global_timeout_(declare_parameter<double>("global_timeout", 0.0))
{
  if (not get_parameter("use_sim_time").as_bool()) {
    THROW_SIMULATION_ERROR(
      "rollout_test_runner requires use_sim_time:=true so that the simulation ROS time follows "
      "the rosbag time.");
  }
  if (on_bag_end_ != "hold" and on_bag_end_ != "switch") {
    THROW_SIMULATION_ERROR(
      "Parameter on_bag_end must be either \"hold\" or \"switch\", but got ",
      std::quoted(on_bag_end_), ".");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), "Replay source: " << std::fixed << ego_replay_source_.startTime().seconds()
                                    << " - " << ego_replay_source_.endTime().seconds()
                                    << " [s], anchor = " << bag_anchor_seconds_ << " [s]");
}

auto RolloutNode::configure() -> traffic_simulator::Configuration
{
  const auto map_path = declare_parameter<std::string>("map_path", "");
  auto configuration = traffic_simulator::Configuration(
    map_path, traffic_simulator::Configuration::Pathname("rollout"));
  configuration.verbose = declare_parameter<bool>("verbose", false);
  return configuration;
}

void RolloutNode::start()
{
  const auto initial_sample = ego_replay_source_.interpolate(api_.getCurrentRosTime());
  api_.spawn(
    ego_name_, initial_sample.pose, defaultVehicleParameters(),
    traffic_simulator::VehicleBehavior::autoware(),
    declare_parameter<std::string>("ego_model", ""));

  auto & ego = api_.getEgoEntity(ego_name_);
  ego.setParameter<bool>("allow_goal_modification", true);

  if (const auto goal = declare_parameter<std::vector<double>>("goal_pose", std::vector<double>());
      goal.size() == 6) {
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal[0];
    goal_pose.position.y = goal[1];
    goal_pose.position.z = goal[2];
    tf2::Quaternion quaternion;
    quaternion.setRPY(goal[3], goal[4], goal[5]);
    goal_pose.orientation = tf2::toMsg(quaternion);
    ego.requestAssignRoute(std::vector<geometry_msgs::msg::Pose>{goal_pose});
  } else if (goal.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "No goal_pose is given. Autoware never engages, so switching to closed-loop will be "
      "refused.");
  } else {
    THROW_SIMULATION_ERROR(
      "Parameter goal_pose must be [x, y, z, roll, pitch, yaw] in the map frame (6 elements), "
      "but got ",
      goal.size(), " elements.");
  }

  /// @note Required for EgoEntitySimulation::overwrite() to seed the vehicle model every frame.
  api_.startNpcLogic();

  switch_service_ = create_service<std_srvs::srv::Trigger>(
    "/rollout/switch", [this](
                         const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto error = switchToClosedLoop();
      response->success = error.empty();
      response->message = error;
    });

  const auto rate =
    std::chrono::duration<double>(1.0 / get_parameter("global_frame_rate").as_double());
  update_timer_ = create_wall_timer(rate, [this]() { onUpdate(); });
}

void RolloutNode::onUpdate()
{
  try {
    const auto bag_time = api_.getCurrentRosTime();

    passthrough_.publishUntil(bag_time);

    auto & ego = api_.getEgoEntity(ego_name_);

    if (phase_ == Phase::REPLAY) {
      if (not ego.isEngaged() and ego.isEngageable()) {
        RCLCPP_INFO_STREAM(get_logger(), "Engaging Autoware.");
        ego.engage();
      }
      if (bag_time > ego_replay_source_.endTime() and on_bag_end_ == "switch") {
        if (const auto error = switchToClosedLoop(); not error.empty()) {
          RCLCPP_WARN_STREAM(get_logger(), "Automatic switch on bag end failed: " << error);
        }
      } else {
        injectEgoStateFromBag(bag_time);
      }
    }

    api_.updateFrame();

    if (
      phase_ == Phase::REPLAY and auto_switch_time_ >= 0.0 and
      api_.getCurrentTime() >= auto_switch_time_) {
      if (const auto error = switchToClosedLoop(); not error.empty()) {
        RCLCPP_WARN_STREAM_THROTTLE(
          get_logger(), *get_clock(), 5000, "Automatic switch failed: " << error);
      }
    }

    if (global_timeout_ > 0.0 and api_.getCurrentTime() >= global_timeout_) {
      RCLCPP_INFO_STREAM(get_logger(), "Reached global timeout. Shutting down.");
      update_timer_->cancel();
      rclcpp::shutdown();
    }
  } catch (const common::scenario_simulator_exception::Error & error) {
    RCLCPP_ERROR_STREAM(get_logger(), error.what());
    update_timer_->cancel();
    rclcpp::shutdown();
  }
}

void RolloutNode::injectEgoStateFromBag(const rclcpp::Time & bag_time)
{
  const auto sample = ego_replay_source_.interpolate(bag_time);
  auto & ego = api_.getEgoEntity(ego_name_);
  /// @note While this flag is set, API::updateEntitiesStatusInSim() requests the vehicle
  /// simulation to overwrite the vehicle model state instead of integrating control commands.
  ego.setControlledBySimulator(true);
  ego.setMapPose(sample.pose);
  ego.setTwist(sample.twist);
  ego.setAcceleration(sample.accel);
}

auto RolloutNode::switchToClosedLoop() -> std::string
{
  auto & ego = api_.getEgoEntity(ego_name_);
  if (phase_ != Phase::REPLAY) {
    return "Already in closed-loop phase.";
  } else if (not ego.isEngaged()) {
    return "Autoware is not engaged yet. The control command is not trustworthy.";
  } else {
    /// @note The vehicle model has been seeded with the replayed pose, velocity, steering and
    /// acceleration on every frame, so just lowering the flag continues seamlessly.
    ego.setControlledBySimulator(false);
    phase_ = Phase::CLOSED_LOOP;
    RCLCPP_INFO_STREAM(
      get_logger(), "Switched to closed-loop at scenario time " << api_.getCurrentTime()
                                                                << " [s] (bag time "
                                                                << api_.getCurrentRosTime().seconds()
                                                                << " [s]).");
    return "";
  }
}
}  // namespace rollout_test_runner
