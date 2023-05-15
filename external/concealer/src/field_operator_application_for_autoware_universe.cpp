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
#include <concealer/field_operator_application_for_autoware_universe.hpp>

namespace concealer
{
FieldOperatorApplicationFor<AutowareUniverse>::~FieldOperatorApplicationFor()
{
  shutdownAutoware();
  // All tasks should be complete before the services used in them will be deinitialized.
  task_queue.stopAndJoin();
}

auto FieldOperatorApplicationFor<AutowareUniverse>::approve(
  const tier4_rtc_msgs::msg::CooperateStatusArray & cooperate_status_array) -> void
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

auto FieldOperatorApplicationFor<AutowareUniverse>::cooperate(
  const tier4_rtc_msgs::msg::CooperateStatusArray & cooperate_status_array) -> void
{
  switch (current_cooperator) {
    case Cooperator::simulator:
      return task_queue.delay([=]() { return approve(cooperate_status_array); });

    default:
      return;
  }
}

auto FieldOperatorApplicationFor<AutowareUniverse>::initialize(
  const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      waitForAutowareStateToBeWaitingForRoute([&]() {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = get_clock()->now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose = initial_pose;
        return setInitialPose(initial_pose_msg);
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

auto FieldOperatorApplicationFor<AutowareUniverse>::plan(
  const std::vector<geometry_msgs::msg::PoseStamped> & route) -> void
{
  assert(not route.empty());

  task_queue.delay([this, route] {
    waitForAutowareStateToBeWaitingForRoute();  // NOTE: This is assertion.

    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>();

    request->header = route.back().header;

    request->option.allow_goal_modification =
      get_parameter("allow_goal_modification").get_value<bool>();

    request->goal = route.back().pose;

    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      request->waypoints.push_back(each.pose);
    }

    requestSetRoutePoints(request);

    waitForAutowareStateToBeWaitingForEngage();
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::engage() -> void
{
  task_queue.delay([this]() {
    waitForAutowareStateToBeDriving([this]() {
      auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
      request->engage = true;
      requestEngage(request);
    });
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::engageable() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isWaitingForEngage();
}

auto FieldOperatorApplicationFor<AutowareUniverse>::engaged() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isDriving();
}

auto FieldOperatorApplicationFor<AutowareUniverse>::getWaypoints() const
  -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto FieldOperatorApplicationFor<AutowareUniverse>::getTurnIndicatorsCommand() const
  -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand
{
  return getTurnIndicatorsCommandImpl();
}

auto FieldOperatorApplicationFor<AutowareUniverse>::restrictTargetSpeed(double value) const
  -> double
{
  // no restrictions here
  return value;
}

auto FieldOperatorApplicationFor<AutowareUniverse>::getAutowareStateName() const -> std::string
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

auto FieldOperatorApplicationFor<AutowareUniverse>::getEmergencyStateName() const -> std::string
{
  return minimum_risk_maneuver_state;
}

auto FieldOperatorApplicationFor<AutowareUniverse>::getMinimumRiskManeuverBehaviorName() const
  -> std::string
{
  return minimum_risk_maneuver_behavior;
}

auto FieldOperatorApplicationFor<AutowareUniverse>::getMinimumRiskManeuverStateName() const
  -> std::string
{
  return minimum_risk_maneuver_state;
}

auto FieldOperatorApplicationFor<AutowareUniverse>::sendSIGINT() -> void  //
{
  ::kill(process_id, SIGINT);
}

auto FieldOperatorApplicationFor<AutowareUniverse>::setVelocityLimit(double velocity_limit) -> void
{
  task_queue.delay([this, velocity_limit]() {
    auto request = std::make_shared<tier4_external_api_msgs::srv::SetVelocityLimit::Request>();
    request->velocity = velocity_limit;
    // We attempt to resend the service up to 30 times, but this number of times was determined by
    // heuristics, not for any technical reason
    requestSetVelocityLimit(request, 30);
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::setCooperator(const std::string & cooperator)
  -> void
{
  current_cooperator = boost::lexical_cast<Cooperator>(cooperator);
}

auto FieldOperatorApplicationFor<AutowareUniverse>::receiveEmergencyState(
  const autoware_auto_system_msgs::msg::EmergencyState & message) -> void
{
#define CASE(IDENTIFIER)                                           \
  case autoware_auto_system_msgs::msg::EmergencyState::IDENTIFIER: \
    minimum_risk_maneuver_state = #IDENTIFIER;                     \
    break

  switch (message.state) {
    CASE(MRM_FAILED);
    CASE(MRM_OPERATING);
    CASE(MRM_SUCCEEDED);
    CASE(NORMAL);
    CASE(OVERRIDE_REQUESTING);

    default:
      throw common::Error("Unsupported MrmState::state, number: ", static_cast<int>(message.state));
  }

  minimum_risk_maneuver_behavior = "";
#undef CASE
}

auto FieldOperatorApplicationFor<AutowareUniverse>::receiveMrmState(
  const autoware_adapi_v1_msgs::msg::MrmState & message) -> void
{
#define CASE(IDENTIFIER, VARIABLE)                        \
  case autoware_adapi_v1_msgs::msg::MrmState::IDENTIFIER: \
    VARIABLE = #IDENTIFIER;                               \
    break

  switch (message.state) {
    CASE(MRM_FAILED, minimum_risk_maneuver_state);
    CASE(MRM_OPERATING, minimum_risk_maneuver_state);
    CASE(MRM_SUCCEEDED, minimum_risk_maneuver_state);
    CASE(NORMAL, minimum_risk_maneuver_state);
    CASE(UNKNOWN, minimum_risk_maneuver_state);
    default:
      throw common::Error(
        "Unsupported MrmState::state, number : ", static_cast<int>(message.state));
  }

  switch (message.behavior) {
    CASE(COMFORTABLE_STOP, minimum_risk_maneuver_behavior);
    CASE(EMERGENCY_STOP, minimum_risk_maneuver_behavior);
    CASE(NONE, minimum_risk_maneuver_behavior);
    CASE(UNKNOWN, minimum_risk_maneuver_behavior);
    default:
      throw common::Error(
        "Unsupported MrmState::behavior, number : ", static_cast<int>(message.behavior));
  }
#undef CASE
}
}  // namespace concealer
