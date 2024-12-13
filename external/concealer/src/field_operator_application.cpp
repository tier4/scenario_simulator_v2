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

#include <concealer/field_operator_application.hpp>
#include <cstdlib>
#include <exception>
#include <scenario_simulator_exception/exception.hpp>
#include <system_error>

namespace concealer
{
template <typename T>
auto toAutowareStateString(std::uint8_t state) -> char const *
{
#define CASE(IDENTIFIER) \
  case T::IDENTIFIER:    \
    return #IDENTIFIER

  switch (state) {
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

// clang-format off
FieldOperatorApplication::FieldOperatorApplication(const pid_t pid)
: rclcpp::Node("concealer_user", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
  process_id(pid),
  getAutowareState("/autoware/state", rclcpp::QoS(1), *this, [this](const auto & v) {
    autoware_state = toAutowareStateString<autoware_system_msgs::msg::AutowareState>(v.state);
  }),
  getCommand("/control/command/control_cmd", rclcpp::QoS(1), *this),
  getCooperateStatusArray("/api/external/get/rtc_status", rclcpp::QoS(1), *this, [this](const auto & v) { latest_cooperate_status_array = v; }),
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
        case autoware_adapi_v1_msgs::msg::MrmState::MRM_FAILED:
          return "MRM_FAILED";
        case autoware_adapi_v1_msgs::msg::MrmState::MRM_OPERATING:
          return "MRM_OPERATING";
        case autoware_adapi_v1_msgs::msg::MrmState::MRM_SUCCEEDED:
          return "MRM_SUCCEEDED";
        case autoware_adapi_v1_msgs::msg::MrmState::NORMAL:
          return "NORMAL";
        case autoware_adapi_v1_msgs::msg::MrmState::UNKNOWN:
          return "UNKNOWN";
        default:
          throw common::Error(
            "Unexpected autoware_adapi_v1_msgs::msg::MrmState::state, number: ", state);
      }
    };

    auto behavior_name_of = [](auto behavior) constexpr {
      if constexpr (HasStaticCOMFORTABLE_STOP<autoware_adapi_v1_msgs::msg::MrmState>::value) {
        if (behavior == autoware_adapi_v1_msgs::msg::MrmState::COMFORTABLE_STOP) {
          return "COMFORTABLE_STOP";
        }
      }
      if constexpr (HasStaticEMERGENCY_STOP<autoware_adapi_v1_msgs::msg::MrmState>::value) {
        if (behavior == autoware_adapi_v1_msgs::msg::MrmState::EMERGENCY_STOP) {
          return "EMERGENCY_STOP";
        }
      }
      if constexpr (HasStaticNONE<autoware_adapi_v1_msgs::msg::MrmState>::value) {
        if (behavior == autoware_adapi_v1_msgs::msg::MrmState::NONE) {
          return "NONE";
        }
      }
      if constexpr (HasStaticUNKNOWN<autoware_adapi_v1_msgs::msg::MrmState>::value) {
        if (behavior == autoware_adapi_v1_msgs::msg::MrmState::UNKNOWN) {
          return "UNKNOWN";
        }
      }
      if constexpr (HasStaticPULL_OVER<autoware_adapi_v1_msgs::msg::MrmState>::value) {
        if (behavior == autoware_adapi_v1_msgs::msg::MrmState::PULL_OVER) {
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
  shutdownAutoware();
  // All tasks should be complete before the services used in them will be deinitialized.
  task_queue.stopAndJoin();
}

auto FieldOperatorApplication::spinSome() -> void
{
  if (rclcpp::ok() and not is_stop_requested.load()) {
    if (process_id != 0) {
      auto status = 0;
      if (const auto id = waitpid(process_id, &status, WNOHANG); id < 0) {
        switch (errno) {
          case ECHILD:
            is_autoware_exited = true;
            throw common::AutowareError("Autoware process is already terminated");
          default:
            AUTOWARE_SYSTEM_ERROR("waitpid");
            std::exit(EXIT_FAILURE);
        }
      } else if (0 < id) {
        if (WIFEXITED(status)) {
          is_autoware_exited = true;
          throw common::AutowareError(
            "Autoware process is unintentionally exited. exit code: ", WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          is_autoware_exited = true;
          throw common::AutowareError("Autoware process is killed. signal is ", WTERMSIG(status));
        }
      }
    }
    rclcpp::spin_some(get_node_base_interface());
  }
}

auto FieldOperatorApplication::shutdownAutoware() -> void
{
  if (is_stop_requested.store(true);
      process_id != 0 && not std::exchange(is_autoware_exited, true)) {
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
      auto sigterm_timeout = [](auto value) {
        auto node = rclcpp::Node("get_parameter_sigterm_timeout", "simulation");
        node.declare_parameter<int>("sigterm_timeout", value);
        node.get_parameter<int>("sigterm_timeout", value);
        return value;
      };
      auto timeout = timespec();
      timeout.tv_sec = sigterm_timeout(5);
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
  }
}
}  // namespace concealer
