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

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <concealer/autoware.hpp>
#include <concealer/publisher_wrapper.hpp>
#include <concealer/subscriber_wrapper.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

namespace concealer
{
/*
 * Implements Autoware interface for Autoware Universe
 * NOTE: This class is intended to be move to simple_sensor_simulator
 */
class AutowareUniverse : public Autoware
{
  // clang-format off
  SubscriberWrapper<autoware_control_msgs::msg::Control, ThreadSafety::safe> getCommand;
  SubscriberWrapper<autoware_vehicle_msgs::msg::GearCommand,             ThreadSafety::safe> getGearCommandImpl;
  SubscriberWrapper<autoware_vehicle_msgs::msg::TurnIndicatorsCommand,   ThreadSafety::safe> getTurnIndicatorsCommand;
  SubscriberWrapper<tier4_planning_msgs::msg::PathWithLaneId,         ThreadSafety::safe> getPathWithLaneId;

  PublisherWrapper<geometry_msgs::msg::AccelWithCovarianceStamped>        setAcceleration;
  PublisherWrapper<nav_msgs::msg::Odometry>                               setOdometry;
  PublisherWrapper<autoware_vehicle_msgs::msg::SteeringReport>       setSteeringReport;
  PublisherWrapper<autoware_vehicle_msgs::msg::GearReport>           setGearReport;
  PublisherWrapper<autoware_vehicle_msgs::msg::ControlModeReport>    setControlModeReport;
  PublisherWrapper<autoware_vehicle_msgs::msg::VelocityReport>       setVelocityReport;
  PublisherWrapper<autoware_vehicle_msgs::msg::TurnIndicatorsReport> setTurnIndicatorsReport;
  // clang-format on

  const rclcpp::TimerBase::SharedPtr localization_update_timer;

  const rclcpp::TimerBase::SharedPtr vehicle_state_update_timer;

  std::thread localization_and_vehicle_state_update_thread;

  std::atomic<bool> is_stop_requested = false;

  std::atomic<uint8_t> current_control_mode =
    autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

  std::atomic<bool> is_thrown = false;

  std::exception_ptr thrown;

  auto stopAndJoin() -> void;

public:
  CONCEALER_PUBLIC explicit AutowareUniverse();

  ~AutowareUniverse();

  auto rethrow() -> void override;

  auto getAcceleration() const -> double override;

  auto getSteeringAngle() const -> double override;

  auto getVelocity() const -> double override;

  auto updateLocalization() -> void;

  auto updateVehicleState() -> void;

  auto getGearCommand() const -> autoware_vehicle_msgs::msg::GearCommand override;

  auto getGearSign() const -> double override;

  auto getVehicleCommand() const -> std::tuple<
    autoware_control_msgs::msg::Control, autoware_vehicle_msgs::msg::GearCommand> override;

  auto getRouteLanelets() const -> std::vector<std::int64_t>;

  auto setManualMode() -> void override;

  auto setAutonomousMode() -> void override;
};

}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
