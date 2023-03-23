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

#include <boost/range/adaptor/sliced.hpp>
#include <concealer/autoware_universe.hpp>

namespace concealer
{
AutowareUniverse::~AutowareUniverse() { shutdownAutoware(); }

auto AutowareUniverse::approve(const CooperateStatusArray & cooperate_status_array) -> void
{
  auto request = std::make_shared<tier4_rtc_msgs::srv::CooperateCommands::Request>();
  request->stamp = cooperate_status_array.stamp;

  auto approvable = [](auto && cooperate_status) {
    return cooperate_status.safe xor
           (cooperate_status.command_status.type == tier4_rtc_msgs::msg::Command::ACTIVATE);
  };

  auto flip = [](auto && type) {
    using Command = tier4_rtc_msgs::msg::Command;
    return type == Command::ACTIVATE ? Command::DEACTIVATE : Command::ACTIVATE;
  };

  for (auto && cooperate_status : cooperate_status_array.statuses) {
    if (approvable(cooperate_status)) {
      tier4_rtc_msgs::msg::CooperateCommand cooperate_command;
      cooperate_command.module = cooperate_status.module;
      cooperate_command.uuid = cooperate_status.uuid;
      cooperate_command.command.type = flip(cooperate_status.command_status.type);
      request->commands.push_back(cooperate_command);
    }
  }

  if (not request->commands.empty()) {
    task_queue.delay([this, request]() { requestCooperateCommands(request); });
  }
}

auto AutowareUniverse::cooperate(const CooperateStatusArray & cooperate_status_array) -> void
{
  switch (current_cooperator) {
    case Cooperator::simulator:
      return cooperation_queue.delay([=]() { return approve(cooperate_status_array); });

    default:
      return;
  }
}

auto AutowareUniverse::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      set(initial_pose);
      waitForAutowareStateToBeWaitingForRoute([&]() {
        InitialPose initial_pose;
        {
          initial_pose.header.stamp = get_clock()->now();
          initial_pose.header.frame_id = "map";
          initial_pose.pose.pose = current_pose;
        }
        return setInitialPose(initial_pose);
      });

      // TODO(yamacir-kit) AFTER /api/autoware/set/initialize_pose IS SUPPORTED.
      // waitForAutowareStateToBeWaitingForRoute([&]() {
      //   auto request = std::make_shared<InitializePose::Request>();
      //   request->pose.header.stamp = get_clock()->now();
      //   request->pose.header.frame_id = "map";
      //   request->pose.pose.pose = initial_pose;
      //   requestInitializePose(request);
      // });
    });
  }
}

auto AutowareUniverse::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) -> void
{
  assert(not route.empty());

  task_queue.delay([this, route] {
    waitForAutowareStateToBeWaitingForRoute();  // NOTE: This is assertion.
    setGoalPose(route.back());
    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      setCheckpoint(each);
    }
    waitForAutowareStateToBeWaitingForEngage();
  });
}

auto AutowareUniverse::engage() -> void
{
  task_queue.delay([this]() {
    waitForAutowareStateToBeDriving([this]() {
      auto request = std::make_shared<Engage::Request>();
      request->engage = true;
      requestEngage(request);
    });
  });
}

auto AutowareUniverse::engageable() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isWaitingForEngage();
}

auto AutowareUniverse::engaged() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isDriving();
}

auto AutowareUniverse::update() -> void
{
  setAcceleration([this]() {
    Acceleration message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "/base_link";
    message.accel.accel = current_acceleration;
    message.accel.covariance.at(6 * 0 + 0) = 0.001;  // linear x
    message.accel.covariance.at(6 * 1 + 1) = 0.001;  // linear y
    message.accel.covariance.at(6 * 2 + 2) = 0.001;  // linear z
    message.accel.covariance.at(6 * 3 + 3) = 0.001;  // angular x
    message.accel.covariance.at(6 * 4 + 4) = 0.001;  // angular y
    message.accel.covariance.at(6 * 5 + 5) = 0.001;  // angular z
    return message;
  }());

  setControlModeReport([this]() {
    ControlModeReport message;
    message.mode = ControlModeReport::AUTONOMOUS;
    return message;
  }());

  setGearReport([this]() {
    GearReport message;
    message.stamp = get_clock()->now();
    message.report = getGearCommand().command;
    return message;
  }());

  setTurnIndicatorsReport([this]() {
    TurnIndicatorsReport message;
    message.stamp = get_clock()->now();
    message.report = getTurnIndicatorsCommand().command;
    return message;
  }());

  setSteeringReport([this]() {
    SteeringReport message;
    message.stamp = get_clock()->now();
    message.steering_tire_angle = getSteeringAngle();
    return message;
  }());

  setVelocityReport([this]() {
    VelocityReport message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "base_link";
    message.longitudinal_velocity = current_twist.linear.x;
    message.lateral_velocity = current_twist.linear.y;
    message.heading_rate = current_twist.angular.z;
    return message;
  }());

  setOdometry([this]() {
    Odometry message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "map";
    message.pose.pose = current_pose;
    message.pose.covariance = {};
    message.twist.twist = current_twist;
    return message;
  }());

  setTransform(current_pose);
}

auto AutowareUniverse::getAcceleration() const -> double
{
  return getAckermannControlCommand().longitudinal.acceleration;
}

auto AutowareUniverse::getVelocity() const -> double
{
  return getAckermannControlCommand().longitudinal.speed;
}

auto AutowareUniverse::getSteeringAngle() const -> double
{
  return getAckermannControlCommand().lateral.steering_tire_angle;
}

auto AutowareUniverse::getGearSign() const -> double
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  return getGearCommand().command == GearCommand::REVERSE or
             getGearCommand().command == GearCommand::REVERSE_2
           ? -1.0
           : 1.0;
}

auto AutowareUniverse::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto AutowareUniverse::restrictTargetSpeed(double value) const -> double
{
  // no restrictions here
  return value;
}

auto AutowareUniverse::getAutowareStateName() const -> std::string
{
  using autoware_auto_system_msgs::msg::AutowareState;

#define CASE(IDENTIFIER)          \
  case AutowareState::IDENTIFIER: \
    return #IDENTIFIER

  switch (getAutowareState().state) {
    CASE(INITIALIZING);
    CASE(WAITING_FOR_ROUTE);
    CASE(PLANNING);
    CASE(WAITING_FOR_ENGAGE);
    CASE(DRIVING);
    CASE(ARRIVED_GOAL);
    CASE(FINALIZING);

    default:
      return "";
  }

#undef CASE
}

auto AutowareUniverse::getEmergencyStateName() const -> std::string
{
  return minimum_risk_maneuver_state;
}

auto AutowareUniverse::getMinimumRiskManeuverBehaviorName() const -> std::string
{
  return minimum_risk_maneuver_behavior;
}

auto AutowareUniverse::getMinimumRiskManeuverStateName() const -> std::string
{
  return minimum_risk_maneuver_state;
}

auto AutowareUniverse::sendSIGINT() -> void  //
{
  ::kill(process_id, SIGINT);
}

auto AutowareUniverse::setVelocityLimit(double velocity_limit) -> void
{
  task_queue.delay([this, velocity_limit]() {
    auto request = std::make_shared<SetVelocityLimit::Request>();
    request->velocity = velocity_limit;
    // We attempt to resend the service up to 30 times, but this number of times was determined by
    // heuristics, not for any technical reason
    requestSetVelocityLimit(request, 30);
  });
}

auto AutowareUniverse::getVehicleCommand() const -> std::tuple<
  autoware_auto_control_msgs::msg::AckermannControlCommand,
  autoware_auto_vehicle_msgs::msg::GearCommand>
{
  return std::make_tuple(getAckermannControlCommand(), getGearCommand());
}

auto AutowareUniverse::receiveEmergencyState(const EmergencyState & msg) -> void
{
#define CASE(IDENTIFIER)                       \
  case EmergencyState::IDENTIFIER:             \
    minimum_risk_maneuver_state = #IDENTIFIER; \
    break

  switch (msg.state) {
    CASE(MRM_FAILED);
    CASE(MRM_OPERATING);
    CASE(MRM_SUCCEEDED);
    CASE(NORMAL);
    CASE(OVERRIDE_REQUESTING);

    default:
      throw common::Error("Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
  }
  minimum_risk_maneuver_behavior = "";
#undef CASE
}

auto AutowareUniverse::receiveMrmState(const MrmState & msg) -> void
{
#define CASE(IDENTIFIER, VARIABLE) \
  case MrmState::IDENTIFIER:       \
    VARIABLE = #IDENTIFIER;        \
    break

  switch (msg.state) {
    CASE(MRM_FAILED, minimum_risk_maneuver_state);
    CASE(MRM_OPERATING, minimum_risk_maneuver_state);
    CASE(MRM_SUCCEEDED, minimum_risk_maneuver_state);
    CASE(NORMAL, minimum_risk_maneuver_state);
    CASE(UNKNOWN, minimum_risk_maneuver_state);
    default:
      throw common::Error("Unsupported MrmState::state, number : ", static_cast<int>(msg.state));
  }

  switch (msg.behavior) {
    CASE(COMFORTABLE_STOP, minimum_risk_maneuver_behavior);
    CASE(EMERGENCY_STOP, minimum_risk_maneuver_behavior);
    CASE(NONE, minimum_risk_maneuver_behavior);
    CASE(UNKNOWN, minimum_risk_maneuver_behavior);
    default:
      throw common::Error(
        "Unsupported MrmState::behavior, number : ", static_cast<int>(msg.behavior));
  }
#undef CASE
}
}  // namespace concealer

namespace autoware_auto_vehicle_msgs::msg
{
auto operator<<(std::ostream & out, const TurnIndicatorsCommand & message) -> std::ostream &
{
#define CASE(IDENTIFIER)                  \
  case TurnIndicatorsCommand::IDENTIFIER: \
    out << #IDENTIFIER;                   \
    break

  switch (message.command) {
    CASE(DISABLE);
    CASE(ENABLE_LEFT);
    CASE(ENABLE_RIGHT);
    CASE(NO_COMMAND);

    default:
      throw common::Error(
        "Unsupported TurnIndicatorsCommand, state number : ", static_cast<int>(message.command));
  }

#undef CASE

  return out;
}

auto operator>>(std::istream & is, TurnIndicatorsCommand & message) -> std::istream &
{
#define STATE(IDENTIFIER) {#IDENTIFIER, TurnIndicatorsCommand::IDENTIFIER}

  std::unordered_map<std::string, std::uint8_t> state_dictionary{
    STATE(DISABLE),
    STATE(ENABLE_LEFT),
    STATE(ENABLE_RIGHT),
    STATE(NO_COMMAND),
  };

#undef STATE

  std::string command_string;
  is >> command_string;

  if (auto iter = state_dictionary.find(command_string); iter != state_dictionary.end()) {
    message.set__command(iter->second);
  } else {
    throw common::Error("Unsupported TurnIndicatorsCommand::command : ", command_string.c_str());
  }

  return is;
}
}  // namespace autoware_auto_vehicle_msgs::msg
