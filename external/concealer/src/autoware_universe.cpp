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

#include <concealer/autoware_universe.hpp>

namespace concealer
{
/*
   clang-format seems not to understand the constructor function try-block: it
   misidentifies the try as a jump label and the formatting afterwards falls
   apart.
*/
// clang-format off
AutowareUniverse::AutowareUniverse(bool simulate_localization) try
: rclcpp::Node("concealer", "simulation"),
  getCommand("/control/command/control_cmd", rclcpp::QoS(1), *this),
  getGearCommand("/control/command/gear_cmd", rclcpp::QoS(1), *this),
  getTurnIndicatorsCommand("/control/command/turn_indicators_cmd", rclcpp::QoS(1), *this),
  getPathWithLaneId(
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", rclcpp::QoS(1),
    *this),
  setAcceleration(
    simulate_localization ? "/localization/acceleration"
                          : "/simulation/debug/localization/acceleration",
    *this),
  setOdometry(
    simulate_localization ? "/localization/kinematic_state"
                          : "/simulation/debug/localization/kinematic_state",
    *this),
  setPose(
    simulate_localization ? "/simulation/debug/localization/pose_estimator/pose_with_covariance"
                          : "/localization/pose_estimator/pose_with_covariance",
    *this),
  setSteeringReport("/vehicle/status/steering_status", *this),
  setGearReport("/vehicle/status/gear_status", *this),
  setControlModeReport("/vehicle/status/control_mode", *this),
  setVelocityReport("/vehicle/status/velocity_status", *this),
  setTurnIndicatorsReport("/vehicle/status/turn_indicators_status", *this),
  control_mode_request_server(create_service<ControlModeCommand>(
    "/control/control_mode_request",
    [this](
      const ControlModeCommand::Request::SharedPtr request,
      ControlModeCommand::Response::SharedPtr response) {
      switch (request->mode) {
        case ControlModeCommand::Request::AUTONOMOUS:
          current_control_mode.store(ControlModeReport::AUTONOMOUS);
          response->success = true;
          break;
        case ControlModeCommand::Request::MANUAL:
          /*
             NOTE: MANUAL request will come when a remote override is
             triggered. But scenario_simulator_v2 don't support a remote
             override for now.
          */
          response->success = false;
          break;
        default:
          response->success = false;
          break;
      }
    })),
  localization_update_timer(rclcpp::create_timer(
    this, get_clock(),
    std::chrono::milliseconds(
      20),  // Autoware.Universe requires localization topics to send data at 50Hz
    [this]() {
      setAcceleration([this]() {
        AccelWithCovarianceStamped message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = "/base_link";
        message.accel.accel = current_acceleration.load();
        message.accel.covariance.at(6 * 0 + 0) = 0.001;  // linear x
        message.accel.covariance.at(6 * 1 + 1) = 0.001;  // linear y
        message.accel.covariance.at(6 * 2 + 2) = 0.001;  // linear z
        message.accel.covariance.at(6 * 3 + 3) = 0.001;  // angular x
        message.accel.covariance.at(6 * 4 + 4) = 0.001;  // angular y
        message.accel.covariance.at(6 * 5 + 5) = 0.001;  // angular z
        return message;
      }());

      setOdometry([this]() {
        Odometry message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = "map";
        message.pose.pose = current_pose.load();
        message.pose.covariance = {};
        message.twist.twist = current_twist.load();
        return message;
      }());

      setPose([this]() {
        // See https://github.com/tier4/autoware.universe/blob/45ab20af979c5663e5a8d4dda787b1dea8d6e55b/simulator/simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L785-L803
        PoseWithCovarianceStamped message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = "map";
        message.pose.pose = current_pose.load();
        message.pose.covariance.at(6 * 0 + 0) = 0.0225;    // XYZRPY_COV_IDX::X_X
        message.pose.covariance.at(6 * 1 + 1) = 0.0225;    // XYZRPY_COV_IDX::Y_Y
        message.pose.covariance.at(6 * 2 + 2) = 0.0225;    // XYZRPY_COV_IDX::Z_Z
        message.pose.covariance.at(6 * 3 + 3) = 0.000625;  // XYZRPY_COV_IDX::ROLL_ROLL
        message.pose.covariance.at(6 * 4 + 4) = 0.000625;  // XYZRPY_COV_IDX::PITCH_PITCH
        message.pose.covariance.at(6 * 5 + 5) = 0.000625;  // XYZRPY_COV_IDX::YAW_YAW
        return message;
      }());

      setTransform(current_pose.load());
    })),
  vehicle_state_update_timer(rclcpp::create_timer(
    this, get_clock(),
    std::chrono::milliseconds(
      33),  // Autoware.Universe requires vehicle state topics to send data at 30Hz
    [this]() {
      setControlModeReport(getControlModeReport());

      setGearReport([this]() {
        GearReport message;
        message.stamp = get_clock()->now();
        message.report = getGearCommand().command;
        return message;
      }());

      setSteeringReport([this]() {
        SteeringReport message;
        message.stamp = get_clock()->now();
        message.steering_tire_angle = getCommand().lateral.steering_tire_angle;
        return message;
      }());

      setVelocityReport([this]() {
        const auto twist = current_twist.load();
        VelocityReport message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = "base_link";
        message.longitudinal_velocity = twist.linear.x;
        message.lateral_velocity = twist.linear.y;
        message.heading_rate = twist.angular.z;
        return message;
      }());

      setTurnIndicatorsReport([this]() {
        TurnIndicatorsReport message;
        message.stamp = get_clock()->now();

        auto turn_indicators_command = getTurnIndicatorsCommand();
        message.report = turn_indicators_command.command == TurnIndicatorsCommand::NO_COMMAND 
                          ? TurnIndicatorsReport::DISABLE
                          : turn_indicators_command.command;

        return message;
      }());
    })),
   spinner()
  {
    spinner = std::thread([this]() {
    try {
      while (rclcpp::ok() and not is_stop_requested.load()) {
        rclcpp::spin_some(get_node_base_interface());
      }
    } catch (...) {
      thrown = std::current_exception();
      is_thrown.store(true);
    }
  });
}
catch (...)
{
  thrown = std::current_exception();
  is_thrown.store(true);
}
// clang-format on

AutowareUniverse::~AutowareUniverse()
{
  is_stop_requested.store(true);
  spinner.join();
}

auto AutowareUniverse::rethrow() -> void
{
  if (is_thrown.load()) {
    throw thrown;
  }
}

auto AutowareUniverse::getVehicleCommand() const -> std::tuple<double, double, double, double, int>
{
  const auto control_command = getCommand();

  const auto gear_command = getGearCommand();

  auto sign_of = [](auto command) {
    switch (command) {
      case GearCommand::REVERSE:
      case GearCommand::REVERSE_2:
        return -1.0;
      case GearCommand::NONE:
        return 0.0;
      default:
        return 1.0;
    }
  };

  /*
     TODO Currently, acceleration is returned as an unsigned value
     (`control_command.longitudinal.acceleration`) and a signed value
     (`sign_of(gear_command.command)`), but this is for historical reasons and
     there is no longer any reason to do so.

     return std::make_tuple(
       control_command.longitudinal.velocity,
       sign_of(gear_command.command) * control_command.longitudinal.acceleration,
       control_command.lateral.steering_tire_angle,
       gear_command.command);
  */
  return std::make_tuple(
    control_command.longitudinal.velocity, control_command.longitudinal.acceleration,
    control_command.lateral.steering_tire_angle, sign_of(gear_command.command),
    gear_command.command);
}

auto AutowareUniverse::getRouteLanelets() const -> std::vector<std::int64_t>
{
  std::vector<std::int64_t> ids{};
  for (const auto & point : getPathWithLaneId().points) {
    std::copy(point.lane_ids.begin(), point.lane_ids.end(), std::back_inserter(ids));
  }
  return ids;
}

auto AutowareUniverse::getControlModeReport() const -> ControlModeReport
{
  ControlModeReport message;
  message.mode = current_control_mode.load();
  return message;
}

auto AutowareUniverse::setManualMode() -> void
{
  current_control_mode.store(ControlModeReport::MANUAL);
}
}  // namespace concealer
