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

template <auto N, typename Tuples>
struct lister
{
  std::reference_wrapper<const Tuples> tuples;

  explicit lister(const Tuples & tuples) : tuples(std::cref(tuples)) {}
};

template <auto N, typename Tuples>
auto operator<<(std::ostream & ostream, const lister<N, Tuples> & lister) -> std::ostream &
{
  for (auto iterator = std::begin(lister.tuples.get()); iterator != std::end(lister.tuples.get());
       ++iterator) {
    switch (std::distance(iterator, std::end(lister.tuples.get()))) {
      case 1:
        return ostream << std::get<N>(*iterator);

      case 2:
        ostream << std::get<N>(*iterator) << " and ";
        break;

      default:
        ostream << std::get<N>(*iterator) << ", ";
        break;
    }
  }

  return ostream;
}

template <auto N, typename Tuples>
auto listup(const Tuples & tuples) -> lister<N, Tuples>
{
  return lister<N, Tuples>(tuples);
}

#define DEFINE_STATIC_DATA_MEMBER_DETECTOR(NAME)                                    \
  template <typename T, typename = void>                                            \
  struct HasStatic##NAME : public std::false_type                                   \
  {                                                                                 \
  };                                                                                \
                                                                                    \
  template <typename T>                                                             \
  struct HasStatic##NAME<T, std::void_t<decltype(T::NAME)>> : public std::true_type \
  {                                                                                 \
  }

DEFINE_STATIC_DATA_MEMBER_DETECTOR(NONE);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(LANE_CHANGE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(LANE_CHANGE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(GOAL_PLANNER);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(START_PLANNER);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(PULL_OUT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(TRAFFIC_LIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(INTERSECTION);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(INTERSECTION_OCCLUSION);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(CROSSWALK);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(BLIND_SPOT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(DETECTION_AREA);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(NO_STOPPING_AREA);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(OCCLUSION_SPOT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_BY_LC_LEFT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(AVOIDANCE_BY_LC_RIGHT);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(NO_DRIVABLE_LANE);

#undef DEFINE_STATIC_DATA_MEMBER_DETECTOR

/**
 * NOTE: for changes from `distance` to start/finish distance
 * see https://github.com/tier4/tier4_autoware_msgs/commit/8b85e6e43aa48cf4a439c77bf4bf6aee2e70c3ef
 */
template <typename T, typename = void>
struct HasDistance : public std::false_type
{
};

template <typename T>
struct HasDistance<T, std::void_t<decltype(std::declval<T>().distance)>> : public std::true_type
{
};

template <typename T>
auto toModuleType(const std::string & module_name)
{
  static const std::unordered_map<std::string, std::uint8_t> module_type_map = [&]() {
    std::unordered_map<std::string, std::uint8_t> module_type_map;

#define EMPLACE(IDENTIFIER)                              \
  if constexpr (HasStatic##IDENTIFIER<T>::value) {       \
    module_type_map.emplace(#IDENTIFIER, T::IDENTIFIER); \
  }                                                      \
  static_assert(true)

    /*
       The following elements are in order of definition in the
       tier4_rtc_msgs/msg/Module.msg file. Of course, unordered_map doesn't
       preserve the insertion order, so the order itself doesn't matter.
    */
    EMPLACE(NONE);
    EMPLACE(LANE_CHANGE_LEFT);
    EMPLACE(LANE_CHANGE_RIGHT);
    EMPLACE(AVOIDANCE_LEFT);
    EMPLACE(AVOIDANCE_RIGHT);
    EMPLACE(GOAL_PLANNER);
    EMPLACE(START_PLANNER);
    EMPLACE(PULL_OUT);
    EMPLACE(TRAFFIC_LIGHT);
    EMPLACE(INTERSECTION);
    EMPLACE(INTERSECTION_OCCLUSION);
    EMPLACE(CROSSWALK);
    EMPLACE(BLIND_SPOT);
    EMPLACE(DETECTION_AREA);
    EMPLACE(NO_STOPPING_AREA);
    EMPLACE(OCCLUSION_SPOT);
    EMPLACE(EXT_REQUEST_LANE_CHANGE_LEFT);
    EMPLACE(EXT_REQUEST_LANE_CHANGE_RIGHT);
    EMPLACE(AVOIDANCE_BY_LC_LEFT);
    EMPLACE(AVOIDANCE_BY_LC_RIGHT);
    EMPLACE(NO_DRIVABLE_LANE);

#undef EMPLACE

    return module_type_map;
  }();

  if (const auto module_type = module_type_map.find(module_name);
      module_type == module_type_map.end()) {
    throw common::Error(
      "Unexpected module name for tier4_rtc_msgs::msg::Module: ", module_name,
      ". One of the following module names is expected: ", listup<0>(module_type_map), ".");
  } else {
    return module_type->second;
  }
}

template <typename CooperateStatusType>
bool isValidCooperateStatus(
  const CooperateStatusType & cooperate_status, std::uint8_t command_type, std::uint8_t module_type)
{
  /**
   * NOTE1: the finish_distance filter is set to over -20.0,
   * because some valid rtc statuses has negative finish_distance due to the errors of
   * localization or numerical calculation. This threshold is advised by a member of TIER IV
   * planning and control team.
   */

  /**
   * NOTE2: The difference in the variable referred as a distance is the impact of the
   * message specification changes in the following URL.
   * This was also decided after consulting with a member of TIER IV planning and control team.
   * ref: https://github.com/tier4/tier4_autoware_msgs/commit/8b85e6e43aa48cf4a439c77bf4bf6aee2e70c3ef
   */
  if constexpr (HasDistance<CooperateStatusType>::value) {
    return cooperate_status.module.type == module_type &&
           command_type != cooperate_status.command_status.type &&
           cooperate_status.distance >= -20.0;
  } else {
    return cooperate_status.module.type == module_type &&
           command_type != cooperate_status.command_status.type &&
           cooperate_status.finish_distance >= -20.0;
  }
}

auto FieldOperatorApplicationFor<AutowareUniverse>::sendCooperateCommand(
  const std::string & module_name, const std::string & command) -> void
{
  auto to_command_type = [](const auto & command) {
    static const std::unordered_map<std::string, std::uint8_t> command_type_map = {
      {"ACTIVATE", tier4_rtc_msgs::msg::Command::ACTIVATE},
      {"DEACTIVATE", tier4_rtc_msgs::msg::Command::DEACTIVATE},
    };
    if (const auto command_type = command_type_map.find(command);
        command_type == command_type_map.end()) {
      throw common::Error(
        "Unexpected command for tier4_rtc_msgs::msg::Command: ", command,
        ", One of the following commands is expected: ", listup<0>(command_type_map), ".");
    } else {
      return command_type->second;
    }
  };

  if (const auto cooperate_status = std::find_if(
        latest_cooperate_status_array.statuses.begin(),
        latest_cooperate_status_array.statuses.end(),
        [module_type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name),
         command_type = to_command_type(command)](const auto & cooperate_status) {
          return isValidCooperateStatus<tier4_rtc_msgs::msg::CooperateStatus>(
            cooperate_status, command_type, module_type);
        });
      cooperate_status == latest_cooperate_status_array.statuses.end()) {
    std::stringstream what;
    what
      << "Failed to send a cooperate command: Cannot find a valid request to cooperate for module "
      << std::quoted(module_name) << " and command " << std::quoted(command) << "."
      << "Please check if the situation is such that the request occurs when sending.";
    throw common::Error(what.str());
  } else {
    tier4_rtc_msgs::msg::CooperateCommand cooperate_command;
    cooperate_command.module = cooperate_status->module;
    cooperate_command.uuid = cooperate_status->uuid;
    cooperate_command.command.type = to_command_type(command);

    auto request = std::make_shared<tier4_rtc_msgs::srv::CooperateCommands::Request>();
    request->stamp = latest_cooperate_status_array.stamp;
    request->commands.push_back(cooperate_command);

    task_queue.delay([this, request]() { requestCooperateCommands(request); });
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
    setGoalPose(route.back());
    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      setCheckpoint(each);
    }
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
