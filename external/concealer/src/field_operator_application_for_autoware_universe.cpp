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
#include <concealer/has_data_member_allow_goal_modification.hpp>
#include <concealer/has_data_member_option.hpp>
#include <concealer/is_package_exists.hpp>

namespace concealer
{
FieldOperatorApplicationFor<AutowareUniverse>::~FieldOperatorApplicationFor()
{
  shutdownAutoware();
  // All tasks should be complete before the services used in them will be deinitialized.
  task_queue.stopAndJoin();
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

// For MrmState::behavior
DEFINE_STATIC_DATA_MEMBER_DETECTOR(COMFORTABLE_STOP);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EMERGENCY_STOP);
// DEFINE_STATIC_DATA_MEMBER_DETECTOR(NONE); // NOTE: This is defined above.
DEFINE_STATIC_DATA_MEMBER_DETECTOR(UNKNOWN);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(PULL_OVER);

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

  /**
   * NOTE: Used cooperate statuses will be deleted correctly in Autoware side and provided via topic update.
   *       But, their update rate (typ. 10Hz) is lower than the one of scenario_simulator_v2.
   *       So, we need to check cooperate statuses if they are used or not in scenario_simulator_v2 side
   *       to avoid sending the same cooperate command when sending multiple commands between updates of cooperate statuses.
   */
  static std::vector<tier4_rtc_msgs::msg::CooperateStatus> used_cooperate_statuses;
  auto is_used_cooperate_status = [this](const auto & cooperate_status) {
    return std::find_if(
             used_cooperate_statuses.begin(), used_cooperate_statuses.end(),
             [&cooperate_status](const auto & used_cooperate_status) {
               return used_cooperate_status.module == cooperate_status.module &&
                      used_cooperate_status.uuid == cooperate_status.uuid &&
                      used_cooperate_status.command_status.type ==
                        cooperate_status.command_status.type;
             }) != used_cooperate_statuses.end();
  };

  if (const auto cooperate_status = std::find_if(
        latest_cooperate_status_array.statuses.begin(),
        latest_cooperate_status_array.statuses.end(),
        [module_type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name),
         command_type = to_command_type(command),
         is_used_cooperate_status](const auto & cooperate_status) {
          return isValidCooperateStatus<tier4_rtc_msgs::msg::CooperateStatus>(
                   cooperate_status, command_type, module_type) &&
                 not is_used_cooperate_status(cooperate_status);
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

    used_cooperate_statuses.push_back(*cooperate_status);
  }
}

auto FieldOperatorApplicationFor<AutowareUniverse>::initialize(
  const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialize_was_called, true)) {
    task_queue.delay([this, initial_pose]() {
      waitForAutowareStateToBeWaitingForRoute([&]() {

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
        if (
          getLocalizationState().state !=
          autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNINITIALIZED) {
          return;
        }
#endif
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = get_clock()->now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose = initial_pose;

        auto request =
          std::make_shared<autoware_adapi_v1_msgs::srv::InitializeLocalization::Request>();
        request->pose.push_back(initial_pose_msg);
        try {
          return requestInitialPose(request);
        } catch (const decltype(requestInitialPose)::TimeoutError &) {
          // ignore timeout error because this service is validated by Autoware state transition.
          return;
        }
      });
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

    /*
       NOTE: The autoware_adapi_v1_msgs::srv::SetRoutePoints::Request type was
       created on 2022/09/05 [1], and the autoware_adapi_v1_msgs::msg::Option
       type data member was added to the
       autoware_adapi_v1_msgs::srv::SetRoutePoints::Request type on 2023/04/12
       [2]. Therefore, we cannot expect
       autoware_adapi_v1_msgs::srv::SetRoutePoints::Request to always have a
       data member `option`.

       [1] https://github.com/autowarefoundation/autoware_adapi_msgs/commit/805f8ebd3ca24564844df9889feeaf183101fbef
       [2] https://github.com/autowarefoundation/autoware_adapi_msgs/commit/cf310bd038673b6cbef3ae3b61dfe607212de419
    */
    if constexpr (
      has_data_member_option_v<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request> and
      has_data_member_allow_goal_modification_v<
        decltype(std::declval<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>().option)>) {
      request->option.allow_goal_modification =
        get_parameter("allow_goal_modification").get_value<bool>();
    }

    request->goal = route.back().pose;

    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      request->waypoints.push_back(each.pose);
    }

    requestSetRoutePoints(request);

    waitForAutowareStateToBeWaitingForEngage();
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::clearRoute() -> void
{
  task_queue.delay([this] {
    /*
       Since this service tends to be available long after the launch of
       Autoware, set the attempts_count to a high value. There is no technical
       basis for the number 30.
    */
    requestClearRoute(std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>(), 30);
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::engage() -> void
{
  task_queue.delay([this]() {
    waitForAutowareStateToBeDriving([this]() {
      auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
      request->engage = true;
      try {
        return requestEngage(request);
      } catch (const decltype(requestEngage)::TimeoutError &) {
        // ignore timeout error because this service is validated by Autoware state transition.
        return;
      }
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
  -> autoware_vehicle_msgs::msg::TurnIndicatorsCommand
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
  return autoware_state;
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

auto FieldOperatorApplicationFor<AutowareUniverse>::requestAutoModeForCooperation(
  const std::string & module_name, bool enable) -> void
{
  // Note: The implementation of this function will not work properly
  //       if the `rtc_auto_mode_manager` package is present.
  if (not isPackageExists("rtc_auto_mode_manager")) {
    task_queue.delay([this, module_name, enable]() {
      auto request = std::make_shared<tier4_rtc_msgs::srv::AutoModeWithModule::Request>();
      request->module.type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name);
      request->enable = enable;
      // We attempt to resend the service up to 30 times, but this number of times was determined by
      // heuristics, not for any technical reason
      requestSetRtcAutoMode(request, 30);
    });
  } else {
    throw common::Error(
      "FieldOperatorApplicationFor<AutowareUniverse>::requestAutoModeForCooperation is not "
      "supported in this environment, because rtc_auto_mode_manager is present.");
  }
}

auto FieldOperatorApplicationFor<AutowareUniverse>::enableAutowareControl() -> void
{
  task_queue.delay([this]() {
    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    requestEnableAutowareControl(request);
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::disableAutowareControl() -> void
{
  task_queue.delay([this]() {
    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    requestDisableAutowareControl(request);
  });
}

auto FieldOperatorApplicationFor<AutowareUniverse>::receiveEmergencyState(
  const tier4_external_api_msgs::msg::Emergency & message) -> void
{
  if (message.emergency) {
    throw common::Error("Emergency state received");
  }
}

template <typename T>
auto toMinimumRiskManeuverBehaviorString(const std::uint8_t & behavior_number) -> std::string
{
  static const std::unordered_map<std::uint8_t, std::string> behavior_string_map = [&]() {
    std::unordered_map<std::uint8_t, std::string> behavior_string_map;

#define EMPLACE(IDENTIFIER)                                  \
  if constexpr (HasStatic##IDENTIFIER<T>::value) {           \
    behavior_string_map.emplace(T::IDENTIFIER, #IDENTIFIER); \
  }                                                          \
  static_assert(true)

    EMPLACE(COMFORTABLE_STOP);
    EMPLACE(EMERGENCY_STOP);
    EMPLACE(NONE);
    EMPLACE(UNKNOWN);
    EMPLACE(PULL_OVER);

#undef EMPLACE
    return behavior_string_map;
  }();

  if (const auto behavior = behavior_string_map.find(behavior_number);
      behavior == behavior_string_map.end()) {
    throw common::Error(
      "Unexpected autoware_adapi_v1_msgs::msg::MrmState::behavior, number: ", behavior_number);
  } else {
    return behavior->second;
  }
}

auto toMinimumRiskManeuverStateString(const std::uint8_t & state_number) -> std::string
{
  static const std::unordered_map<std::uint8_t, std::string> state_string_map = {
    {autoware_adapi_v1_msgs::msg::MrmState::MRM_FAILED, "MRM_FAILED"},
    {autoware_adapi_v1_msgs::msg::MrmState::MRM_OPERATING, "MRM_OPERATING"},
    {autoware_adapi_v1_msgs::msg::MrmState::MRM_SUCCEEDED, "MRM_SUCCEEDED"},
    {autoware_adapi_v1_msgs::msg::MrmState::NORMAL, "NORMAL"},
    {autoware_adapi_v1_msgs::msg::MrmState::UNKNOWN, "UNKNOWN"},
  };

  if (const auto state = state_string_map.find(state_number); state == state_string_map.end()) {
    throw common::Error(
      "Unexpected autoware_adapi_v1_msgs::msg::MrmState::state, number: ", state_number);
  } else {
    return state->second;
  }
}

auto FieldOperatorApplicationFor<AutowareUniverse>::receiveMrmState(
  const autoware_adapi_v1_msgs::msg::MrmState & message) -> void
{
  minimum_risk_maneuver_state = toMinimumRiskManeuverStateString(message.state);

  minimum_risk_maneuver_behavior =
    toMinimumRiskManeuverBehaviorString<autoware_adapi_v1_msgs::msg::MrmState>(message.behavior);
}
}  // namespace concealer
