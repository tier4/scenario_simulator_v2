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

#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>)
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#else
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#endif

#include <atomic>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/publisher.hpp>
#include <concealer/subscriber.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
class AutowareUniverse : public rclcpp::Node,
                         public ContinuousTransformBroadcaster<AutowareUniverse>
{
public:
  // clang-format off
  using AccelWithCovarianceStamped  = geometry_msgs::msg::AccelWithCovarianceStamped;
  using Control                     = autoware_control_msgs::msg::Control;
  using ControlModeCommand          = autoware_vehicle_msgs::srv::ControlModeCommand;
  using ControlModeReport           = autoware_vehicle_msgs::msg::ControlModeReport;
  using GearCommand                 = autoware_vehicle_msgs::msg::GearCommand;
  using GearReport                  = autoware_vehicle_msgs::msg::GearReport;
  using Odometry                    = nav_msgs::msg::Odometry;
#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>)
  using PathWithLaneId              = autoware_internal_planning_msgs::msg::PathWithLaneId;
#else
  using PathWithLaneId              = tier4_planning_msgs::msg::PathWithLaneId;
#endif
  using PoseWithCovarianceStamped   = geometry_msgs::msg::PoseWithCovarianceStamped;
  using SteeringReport              = autoware_vehicle_msgs::msg::SteeringReport;
  using TurnIndicatorsCommand       = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using TurnIndicatorsReport        = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using VelocityReport              = autoware_vehicle_msgs::msg::VelocityReport;

  Subscriber<Control>               getCommand;
  Subscriber<GearCommand>           getGearCommand;
  Subscriber<TurnIndicatorsCommand> getTurnIndicatorsCommand;
  Subscriber<PathWithLaneId>        getPathWithLaneId;

  Publisher<AccelWithCovarianceStamped> setAcceleration;
  Publisher<Odometry>                   setOdometry;
  Publisher<PoseWithCovarianceStamped>  setPose;
  Publisher<SteeringReport>             setSteeringReport;
  Publisher<GearReport>                 setGearReport;
  Publisher<ControlModeReport>          setControlModeReport;
  Publisher<VelocityReport>             setVelocityReport;
  Publisher<TurnIndicatorsReport>       setTurnIndicatorsReport;

  std::atomic<geometry_msgs::msg::Accel> current_acceleration;
  std::atomic<geometry_msgs::msg::Pose>  current_pose;
  std::atomic<geometry_msgs::msg::Twist> current_twist;
  // clang-format on

private:
  rclcpp::Service<ControlModeCommand>::SharedPtr control_mode_request_server;

  const rclcpp::TimerBase::SharedPtr localization_update_timer;

  const rclcpp::TimerBase::SharedPtr vehicle_state_update_timer;

  std::thread spinner;

  std::atomic<bool> is_stop_requested = false;

  std::atomic<std::uint8_t> current_control_mode = ControlModeReport::AUTONOMOUS;

  std::atomic<bool> is_thrown = false;

  std::exception_ptr thrown;

public:
  CONCEALER_PUBLIC explicit AutowareUniverse(bool);

  ~AutowareUniverse();

  auto rethrow() -> void;

  auto getVehicleCommand() const -> std::tuple<double, double, double, double, int>;

  auto getControlModeReport() const -> ControlModeReport;

  auto setManualMode() -> void;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_UNIVERSE_HPP_
