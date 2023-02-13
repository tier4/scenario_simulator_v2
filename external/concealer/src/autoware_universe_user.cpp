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
#include <concealer/autoware_universe_user.hpp>

namespace concealer
{
AutowareUniverseUser::~AutowareUniverseUser() { shutdownAutoware(); }

auto AutowareUniverseUser::approve(const CooperateStatusArray & cooperate_status_array) -> void
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

auto AutowareUniverseUser::cooperate(const CooperateStatusArray & cooperate_status_array) -> void
{
  switch (current_cooperator) {
    case Cooperator::simulator:
      return cooperation_queue.delay([=]() { return approve(cooperate_status_array); });

    default:
      return;
  }
}

auto AutowareUniverseUser::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      waitForAutowareStateToBeWaitingForRoute([&]() {
        InitialPose initial_pose_msg;
        {
          initial_pose_msg.header.stamp = get_clock()->now();
          initial_pose_msg.header.frame_id = "map";
          initial_pose_msg.pose.pose = initial_pose;
        }
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

auto AutowareUniverseUser::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route) -> void
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

auto AutowareUniverseUser::engage() -> void
{
  task_queue.delay([this]() {
    waitForAutowareStateToBeDriving([this]() {
      auto request = std::make_shared<Engage::Request>();
      request->engage = true;
      requestEngage(request);
    });
  });
}

auto AutowareUniverseUser::engageable() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isWaitingForEngage();
}

auto AutowareUniverseUser::engaged() const -> bool
{
  rethrow();
  return task_queue.exhausted() and isDriving();
}

auto AutowareUniverseUser::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto AutowareUniverseUser::getTurnIndicatorsCommand() const -> autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand {
  return getTurnIndicatorsCommandImpl();
}

auto AutowareUniverseUser::getEmergencyState() const -> autoware_auto_system_msgs::msg::EmergencyState {
  return getEmergencyStateImpl();
}

auto AutowareUniverseUser::restrictTargetSpeed(double value) const -> double
{
  // no restrictions here
  return value;
}

auto AutowareUniverseUser::getAutowareStateName() const -> std::string
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

auto AutowareUniverseUser::sendSIGINT() -> void  //
{
  ::kill(process_id, SIGINT);
}

auto AutowareUniverseUser::setVelocityLimit(double velocity_limit) -> void
{
  task_queue.delay([this, velocity_limit]() {
    auto request = std::make_shared<SetVelocityLimit::Request>();
    request->velocity = velocity_limit;
    // We attempt to resend the service up to 30 times, but this number of times was determined by heuristics, not for any technical reason
    requestSetVelocityLimit(request, 30);
  });
}

  auto AutowareUniverseUser::setCooperator(const std::string & cooperator) -> void
  {
    current_cooperator = boost::lexical_cast<Cooperator>(cooperator);
  }
}  // namespace concealer

namespace autoware_auto_system_msgs::msg
{
auto operator<<(std::ostream & out, const EmergencyState & message) -> std::ostream &
{
#define CASE(IDENTIFIER)           \
  case EmergencyState::IDENTIFIER: \
    out << #IDENTIFIER;            \
    break

  switch (message.state) {
    CASE(MRM_FAILED);
    CASE(MRM_OPERATING);
    CASE(MRM_SUCCEEDED);
    CASE(NORMAL);
    CASE(OVERRIDE_REQUESTING);

    default:
      throw common::Error(
        "Unsupported EmergencyState, state number : ", static_cast<int>(message.state));
  }

#undef CASE

  return out;
}

auto operator>>(std::istream & is, EmergencyState & message) -> std::istream &
{
#define STATE(IDENTIFIER) {#IDENTIFIER, EmergencyState::IDENTIFIER}

  std::unordered_map<std::string, std::uint8_t> state_dictionary{
    STATE(MRM_FAILED), STATE(MRM_OPERATING),       STATE(MRM_SUCCEEDED),
    STATE(NORMAL),     STATE(OVERRIDE_REQUESTING),
  };

#undef STATE

  std::string state_string;
  is >> state_string;

  if (auto iter = state_dictionary.find(state_string); iter != state_dictionary.end()) {
    message.set__state(iter->second);
  } else {
    throw common::Error("Unsupported EmergencyState::state : ", state_string.c_str());
  }

  return is;
}
}  // namespace autoware_auto_system_msgs::msg

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
