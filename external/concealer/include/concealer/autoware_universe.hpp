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

#ifndef CONCEALER__AUTOWARE_UNIVERSE_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_HPP_

#include <atomic>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/publisher_wrapper.hpp>
#include <concealer/subscriber_wrapper.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
/*
 * Implements Autoware interface for Autoware Universe
 * NOTE: This class is intended to be move to simple_sensor_simulator
 */
class AutowareUniverse : public rclcpp::Node,
                         public ContinuousTransformBroadcaster<AutowareUniverse>
{
  // clang-format off
  using ControlModeCommand          = autoware_auto_vehicle_msgs::srv::ControlModeCommand;
  using ControlModeReport           = autoware_auto_vehicle_msgs::msg::ControlModeReport;
  using GearCommand                 = autoware_auto_vehicle_msgs::msg::GearCommand;
  using GearReport                  = autoware_auto_vehicle_msgs::msg::GearReport;
  using TurnIndicatorsCommand       = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using PathWithLaneId              = autoware_auto_planning_msgs::msg::PathWithLaneId;
  using SteeringReport              = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using VelocityReport              = autoware_auto_vehicle_msgs::msg::VelocityReport;
  using TurnIndicatorsReport        = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using AckermannControlCommand     = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using AccelWithCovarianceStamped  = geometry_msgs::msg::AccelWithCovarianceStamped;

  SubscriberWrapper<AckermannControlCommand, ThreadSafety::safe> getAckermannControlCommand;
public:
  SubscriberWrapper<GearCommand,             ThreadSafety::safe> getGearCommand;
private:
  SubscriberWrapper<TurnIndicatorsCommand,   ThreadSafety::safe> getTurnIndicatorsCommand;
  SubscriberWrapper<PathWithLaneId,          ThreadSafety::safe> getPathWithLaneId;

  PublisherWrapper<AccelWithCovarianceStamped>  setAcceleration;
  PublisherWrapper<nav_msgs::msg::Odometry>     setOdometry;
  PublisherWrapper<SteeringReport>              setSteeringReport;
  PublisherWrapper<GearReport>                  setGearReport;
  PublisherWrapper<ControlModeReport>           setControlModeReport;
  PublisherWrapper<VelocityReport>              setVelocityReport;
  PublisherWrapper<TurnIndicatorsReport>        setTurnIndicatorsReport;
  // clang-format on

  rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_request_server;

  const rclcpp::TimerBase::SharedPtr localization_update_timer;

  const rclcpp::TimerBase::SharedPtr vehicle_state_update_timer;

  std::thread localization_and_vehicle_state_update_thread;

  std::atomic<bool> is_stop_requested = false;

  std::atomic<std::uint8_t> current_control_mode = ControlModeReport::AUTONOMOUS;

  std::atomic<bool> is_thrown = false;

  std::exception_ptr thrown;

  std::atomic<geometry_msgs::msg::Accel> current_acceleration;

  std::atomic<geometry_msgs::msg::Twist> current_twist;

  std::atomic<geometry_msgs::msg::Pose> current_pose;

public:
  CONCEALER_PUBLIC explicit AutowareUniverse();

  ~AutowareUniverse();

  auto rethrow() -> void;

  auto getAcceleration() const -> double;

  auto getSteeringAngle() const -> double;

  auto getVelocity() const -> double;

  auto updateLocalization() -> void;

  auto updateVehicleState() -> void;

  auto getGearSign() const -> double;

  auto getVehicleCommand() const -> std::tuple<AckermannControlCommand, GearCommand>;

  auto getRouteLanelets() const -> std::vector<std::int64_t>;

  auto getControlModeReport() const -> ControlModeReport;

  auto set(const geometry_msgs::msg::Accel & acceleration) -> void
  {
    current_acceleration.store(acceleration);
  }

  auto set(const geometry_msgs::msg::Twist & twist) -> void { current_twist.store(twist); }

  auto set(const geometry_msgs::msg::Pose & pose) -> void { current_pose.store(pose); }

  auto setManualMode() -> void;
};

}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
