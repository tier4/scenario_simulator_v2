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
#include <concealer/field_operator_application.hpp>
#include <concealer/is_package_exists.hpp>
#include <concealer/member_detector.hpp>
#include <cstdlib>
#include <exception>
#include <scenario_simulator_exception/exception.hpp>
#include <system_error>

namespace concealer
{
template <typename T>
auto toModuleType(const std::string & module_name)
{
  static const std::unordered_map<std::string, std::uint8_t> module_type_map = [&]() {
    std::unordered_map<std::string, std::uint8_t> module_type_map;

#define EMPLACE(IDENTIFIER)                                  \
  if constexpr (DetectStaticMember_##IDENTIFIER<T>::value) { \
    module_type_map.emplace(#IDENTIFIER, T::IDENTIFIER);     \
  }                                                          \
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
      "Unexpected module name for tier4_rtc_msgs::msg::Module: ", module_name, ".");
  } else {
    return module_type->second;
  }
}

// clang-format off
FieldOperatorApplication::FieldOperatorApplication(const pid_t pid)
: rclcpp::Node("concealer_user", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
  process_id(pid),
  time_limit(std::chrono::steady_clock::now() + std::chrono::seconds(common::getParameter<int>("initialize_duration"))),
  getAutowareState("/autoware/state", rclcpp::QoS(1), *this, [this](const auto & message) {
    auto state_name_of = [](auto state) constexpr {
      switch (state) {
        case AutowareState::INITIALIZING:
          return "INITIALIZING";
        case AutowareState::WAITING_FOR_ROUTE:
          return "WAITING_FOR_ROUTE";
        case AutowareState::PLANNING:
          return "PLANNING";
        case AutowareState::WAITING_FOR_ENGAGE:
          return "WAITING_FOR_ENGAGE";
        case AutowareState::DRIVING:
          return "DRIVING";
        case AutowareState::ARRIVED_GOAL:
          return "ARRIVED_GOAL";
        case AutowareState::FINALIZING:
          return "FINALIZING";
        default:
          return "";
      }
    };

    autoware_state = state_name_of(message.state);
  }),
  getCommand("/control/command/control_cmd", rclcpp::QoS(1), *this),
  getCooperateStatusArray("/api/external/get/rtc_status", rclcpp::QoS(1), *this),
  getEmergencyState("/api/external/get/emergency", rclcpp::QoS(1), *this, [this](const auto & message) {
    if (message.emergency) {
      throw common::Error("Emergency state received");
    }
  }),
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  getLocalizationState("/api/localization/initialization_state", rclcpp::QoS(1).transient_local(), *this),
#endif
  getMrmState("/api/fail_safe/mrm_state", rclcpp::QoS(1), *this, [this](const auto & message) {
    auto state_name_of = [](auto state) constexpr {
      switch (state) {
        case MrmState::MRM_FAILED:
          return "MRM_FAILED";
        case MrmState::MRM_OPERATING:
          return "MRM_OPERATING";
        case MrmState::MRM_SUCCEEDED:
          return "MRM_SUCCEEDED";
        case MrmState::NORMAL:
          return "NORMAL";
        case MrmState::UNKNOWN:
          return "UNKNOWN";
        default:
          throw common::Error(
            "Unexpected autoware_adapi_v1_msgs::msg::MrmState::state, number: ", state);
      }
    };

    auto behavior_name_of = [](auto behavior) constexpr {
      if constexpr (DetectStaticMember_COMFORTABLE_STOP<MrmState>::value) {
        if (behavior == MrmState::COMFORTABLE_STOP) {
          return "COMFORTABLE_STOP";
        }
      }
      if constexpr (DetectStaticMember_EMERGENCY_STOP<MrmState>::value) {
        if (behavior == MrmState::EMERGENCY_STOP) {
          return "EMERGENCY_STOP";
        }
      }
      if constexpr (DetectStaticMember_NONE<MrmState>::value) {
        if (behavior == MrmState::NONE) {
          return "NONE";
        }
      }
      if constexpr (DetectStaticMember_UNKNOWN<MrmState>::value) {
        if (behavior == MrmState::UNKNOWN) {
          return "UNKNOWN";
        }
      }
      if constexpr (DetectStaticMember_PULL_OVER<MrmState>::value) {
        if (behavior == MrmState::PULL_OVER) {
          return "PULL_OVER";
        }
      }
      throw common::Error(
        "Unexpected autoware_adapi_v1_msgs::msg::MrmState::behavior, number: ", behavior);
    };

    minimum_risk_maneuver_state = state_name_of(message.state);
    minimum_risk_maneuver_behavior = behavior_name_of(message.behavior);
  }),
  getPathWithLaneId("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", rclcpp::QoS(1), *this),
  getTrajectory("/api/iv_msgs/planning/scenario_planning/trajectory", rclcpp::QoS(1), *this),
  getTurnIndicatorsCommand("/control/command/turn_indicators_cmd", rclcpp::QoS(1), *this),
  requestClearRoute("/api/routing/clear_route", *this),
  requestCooperateCommands("/api/external/set/rtc_commands", *this),
  requestEngage("/api/external/set/engage", *this),
  requestInitialPose("/api/localization/initialize", *this),
  // NOTE: /api/routing/set_route_points takes a long time to return. But the specified duration is not decided by any technical reasons.
  requestSetRoutePoints("/api/routing/set_route_points", *this, std::chrono::seconds(10)),
  requestSetRtcAutoMode("/api/external/set/rtc_auto_mode", *this),
  requestSetVelocityLimit("/api/autoware/set/velocity_limit", *this),
  requestEnableAutowareControl("/api/operation_mode/enable_autoware_control", *this)
{
}
// clang-format on

FieldOperatorApplication::~FieldOperatorApplication()
{
  if (process_id) {
    const auto sigset = [this]() {
      if (auto signal_set = sigset_t();
          sigemptyset(&signal_set) or sigaddset(&signal_set, SIGCHLD)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else if (auto error = pthread_sigmask(SIG_BLOCK, &signal_set, nullptr)) {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(error, std::system_category()).what());
        std::exit(EXIT_FAILURE);
      } else {
        return signal_set;
      }
    }();

    const auto timeout = []() {
      auto timeout = timespec();
      timeout.tv_sec = common::getParameter<int>("sigterm_timeout", 5);
      timeout.tv_nsec = 0;
      return timeout;
    }();

    if (::kill(process_id, SIGINT); sigtimedwait(&sigset, nullptr, &timeout) < 0) {
      switch (errno) {
        case EINTR:
          /*
             The wait was interrupted by an unblocked, caught signal. It shall
             be documented in system documentation whether this error causes
             these functions to fail.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The wait for Autoware launch process termination was interrupted by an unblocked, "
            "caught signal.");
          break;

        case EAGAIN:
          /*
             No signal specified by set was generated within the specified
             timeout period.
          */
          RCLCPP_ERROR_STREAM(get_logger(), "Autoware launch process does not respond. Kill it.");
          killpg(process_id, SIGKILL);
          break;

        default:
        case EINVAL:
          /*
             The timeout argument specified a tv_nsec value less than zero or
             greater than or equal to 1000 million.
          */
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "The parameter sigterm_timeout specified a value less than zero or greater than or "
            "equal to 1000 million.");
          break;
      }
    }

    if (int status = 0; waitpid(process_id, &status, 0) < 0) {
      if (errno == ECHILD) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Try to wait for the autoware process but it was already exited.");
      } else {
        RCLCPP_ERROR_STREAM(get_logger(), std::system_error(errno, std::system_category()).what());
      }
    }

    process_id = 0;
  }

  finalized.store(true);
}

auto FieldOperatorApplication::clearRoute() -> void
{
  task_queue.delay([this] {
    /*
       Since this service tends to be available long after the launch of
       Autoware, set the attempts_count to a high value. There is no technical
       basis for the number 30.
    */
    requestClearRoute(std::make_shared<ClearRoute::Request>(), 30);
  });
}

auto FieldOperatorApplication::enableAutowareControl() -> void
{
  task_queue.delay([this]() {
    auto request = std::make_shared<ChangeOperationMode::Request>();
    requestEnableAutowareControl(request, 30);
  });
}

auto FieldOperatorApplication::engage() -> void
{
  task_queue.delay([this]() {
    try {
      auto request = std::make_shared<Engage::Request>();
      request->engage = true;
      return requestEngage(request, 1);
    } catch (const common::AutowareError &) {
      return;  // Ignore error because this service is validated by Autoware state transition.
    }

    waitForAutowareStateToBe("DRIVING");

    time_limit = std::decay_t<decltype(time_limit)>::max();
  });
}

auto FieldOperatorApplication::engageable() const -> bool
{
  task_queue.rethrow();
  return task_queue.empty() and autoware_state == "WAITING_FOR_ENGAGE";
}

auto FieldOperatorApplication::engaged() const -> bool
{
  task_queue.rethrow();
  return task_queue.empty() and autoware_state == "DRIVING";
}

auto FieldOperatorApplication::getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;

  for (const auto & point : getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }

  return waypoints;
}

auto FieldOperatorApplication::initialize(const geometry_msgs::msg::Pose & initial_pose) -> void
{
  if (not std::exchange(initialized, true)) {
    task_queue.delay([this, initial_pose]() {
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
      if (getLocalizationState().state != LocalizationInitializationState::UNINITIALIZED) {
        return;
      }
#endif
      try {
        auto request =
          std::make_shared<autoware_adapi_v1_msgs::srv::InitializeLocalization::Request>();
        request->pose.push_back([&]() {
          auto initial_pose_stamped = geometry_msgs::msg::PoseWithCovarianceStamped();
          initial_pose_stamped.header.stamp = get_clock()->now();
          initial_pose_stamped.header.frame_id = "map";
          initial_pose_stamped.pose.pose = initial_pose;
          return initial_pose_stamped;
        }());
        return requestInitialPose(request, 1);
      } catch (const common::AutowareError &) {
        return;  // Ignore error because this service is validated by Autoware state transition.
      }

      waitForAutowareStateToBe("WAITING_FOR_ROUTE");
    });
  }
}

auto FieldOperatorApplication::plan(const std::vector<geometry_msgs::msg::PoseStamped> & route)
  -> void
{
  assert(not route.empty());

  task_queue.delay([this, route] {
    waitForAutowareStateToBe("WAITING_FOR_ROUTE");  // NOTE: This is assertion.

    auto request = std::make_shared<SetRoutePoints::Request>();

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
      DetectMember_option<SetRoutePoints::Request>::value and
      DetectMember_allow_goal_modification<
        decltype(std::declval<SetRoutePoints::Request>().option)>::value) {
      request->option.allow_goal_modification =
        common::getParameter<bool>(get_node_parameters_interface(), "allow_goal_modification");
    }

    request->goal = route.back().pose;

    for (const auto & each : route | boost::adaptors::sliced(0, route.size() - 1)) {
      request->waypoints.push_back(each.pose);
    }

    requestSetRoutePoints(request, 1);

    waitForAutowareStateToBe("WAITING_FOR_ENGAGE");
  });
}

auto FieldOperatorApplication::requestAutoModeForCooperation(
  const std::string & module_name, bool enable) -> void
{
  /*
     The implementation of this function will not work properly if the
     `rtc_auto_mode_manager` package is present.
  */
  if (not isPackageExists("rtc_auto_mode_manager")) {
    task_queue.delay([this, module_name, enable]() {
      auto request = std::make_shared<AutoModeWithModule::Request>();
      request->module.type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name);
      request->enable = enable;
      /*
         We attempt to resend the service up to 30 times, but this number of
         times was determined by heuristics, not for any technical reason.
      */
      requestSetRtcAutoMode(request, 30);
    });
  } else {
    throw common::Error(
      "FieldOperatorApplication::requestAutoModeForCooperation is not supported in this "
      "environment, because rtc_auto_mode_manager is present.");
  }
}

auto FieldOperatorApplication::sendCooperateCommand(
  const std::string & module_name, const std::string & command) -> void
{
  const auto command_type = [&]() {
    if (command == "ACTIVATE") {
      return tier4_rtc_msgs::msg::Command::ACTIVATE;
    } else if (command == "DEACTIVATE") {
      return tier4_rtc_msgs::msg::Command::DEACTIVATE;
    } else {
      throw common::Error("Unexpected command for tier4_rtc_msgs::msg::Command: ", command, ".");
    }
  }();

  /*
     NOTE: Used cooperate statuses will be deleted correctly in Autoware side
     and provided via topic update. But, their update rate (typ. 10Hz) is lower
     than the one of scenario_simulator_v2. So, we need to check cooperate
     statuses if they are used or not in scenario_simulator_v2 side to avoid
     sending the same cooperate command when sending multiple commands between
     updates of cooperate statuses.
  */
  static std::vector<tier4_rtc_msgs::msg::CooperateStatus> used_cooperate_statuses;

  auto is_used_cooperate_status = [](const auto & cooperate_status) {
    return std::find_if(
             used_cooperate_statuses.begin(), used_cooperate_statuses.end(),
             [&cooperate_status](const auto & used_cooperate_status) {
               return used_cooperate_status.module == cooperate_status.module &&
                      used_cooperate_status.uuid == cooperate_status.uuid &&
                      used_cooperate_status.command_status.type ==
                        cooperate_status.command_status.type;
             }) != used_cooperate_statuses.end();
  };

  auto is_valid_cooperate_status =
    [](const auto & cooperate_status, auto command_type, auto module_type) {
      /**
         The finish_distance filter is set to over -20.0, because some valid rtc
         statuses has negative finish_distance due to the errors of localization or
         numerical calculation. This threshold is advised by a member of TIER IV
         planning and control team.

         The difference in the variable referred as a distance is the impact of the
         message specification changes in the following URL. This was also decided
         after consulting with a member of TIER IV planning and control team. ref:
         https://github.com/tier4/tier4_autoware_msgs/commit/8b85e6e43aa48cf4a439c77bf4bf6aee2e70c3ef
      */
      if constexpr (DetectMember_distance<tier4_rtc_msgs::msg::CooperateStatus>::value) {
        return cooperate_status.module.type == module_type &&
               command_type != cooperate_status.command_status.type &&
               cooperate_status.distance >= -20.0;
      } else {
        return cooperate_status.module.type == module_type &&
               command_type != cooperate_status.command_status.type &&
               cooperate_status.finish_distance >= -20.0;
      }
    };

  const auto cooperate_status_array = getCooperateStatusArray();

  if (const auto cooperate_status = std::find_if(
        cooperate_status_array.statuses.begin(), cooperate_status_array.statuses.end(),
        [&, module_type = toModuleType<tier4_rtc_msgs::msg::Module>(module_name)](
          const auto & cooperate_status) {
          return is_valid_cooperate_status(cooperate_status, command_type, module_type) &&
                 not is_used_cooperate_status(cooperate_status);
        });
      cooperate_status == cooperate_status_array.statuses.end()) {
    std::stringstream what;
    what
      << "Failed to send a cooperate command: Cannot find a valid request to cooperate for module "
      << std::quoted(module_name) << " and command " << std::quoted(command) << ". "
      << "Please check if the situation is such that the request occurs when sending.";
    throw common::Error(what.str());
  } else {
    tier4_rtc_msgs::msg::CooperateCommand cooperate_command;
    cooperate_command.module = cooperate_status->module;
    cooperate_command.uuid = cooperate_status->uuid;
    cooperate_command.command.type = command_type;

    auto request = std::make_shared<tier4_rtc_msgs::srv::CooperateCommands::Request>();
    request->stamp = cooperate_status_array.stamp;
    request->commands.push_back(cooperate_command);

    task_queue.delay([this, request]() { requestCooperateCommands(request, 1); });

    used_cooperate_statuses.push_back(*cooperate_status);
  }
}

auto FieldOperatorApplication::setVelocityLimit(double velocity_limit) -> void
{
  task_queue.delay([this, velocity_limit]() {
    auto request = std::make_shared<SetVelocityLimit::Request>();
    request->velocity = velocity_limit;
    /*
       We attempt to resend the service up to 30 times, but this number of
       times was determined by heuristics, not for any technical reason.
    */
    requestSetVelocityLimit(request, 30);
  });
}

auto FieldOperatorApplication::spinSome() -> void
{
  task_queue.rethrow();

  if (rclcpp::ok()) {
    if (process_id) {
      auto status = 0;
      if (const auto id = waitpid(process_id, &status, WNOHANG); id < 0) {
        switch (errno) {
          case ECHILD:
            process_id = 0;
            throw common::AutowareError("Autoware process is already terminated");
          default:
            RCLCPP_ERROR_STREAM(
              get_logger(), std::system_error(errno, std::system_category()).what());
            std::exit(EXIT_FAILURE);
        }
      } else if (0 < id) {
        if (WIFEXITED(status)) {
          process_id = 0;
          throw common::AutowareError(
            "Autoware process is unintentionally exited. exit code: ", WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          process_id = 0;
          throw common::AutowareError("Autoware process is killed. signal is ", WTERMSIG(status));
        }
      }
    }
    rclcpp::spin_some(get_node_base_interface());
  }
}
}  // namespace concealer
