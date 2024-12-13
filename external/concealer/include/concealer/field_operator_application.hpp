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

#ifndef CONCEALER__AUTOWARE_USER_HPP_
#define CONCEALER__AUTOWARE_USER_HPP_

#include <sys/wait.h>

#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#endif

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <concealer/autoware_stream.hpp>
#include <concealer/autoware_universe.hpp>
#include <concealer/launch.hpp>
#include <concealer/publisher_wrapper.hpp>
#include <concealer/service_with_validation.hpp>
#include <concealer/subscriber_wrapper.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/visibility.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/trajectory.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <utility>

namespace concealer
{
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

DEFINE_STATIC_DATA_MEMBER_DETECTOR(COMFORTABLE_STOP);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(EMERGENCY_STOP);
// DEFINE_STATIC_DATA_MEMBER_DETECTOR(NONE); // NOTE: This is defined above.
DEFINE_STATIC_DATA_MEMBER_DETECTOR(UNKNOWN);
DEFINE_STATIC_DATA_MEMBER_DETECTOR(PULL_OVER);

#undef DEFINE_STATIC_DATA_MEMBER_DETECTOR

/* ---- NOTE -------------------------------------------------------------------
 *
 *  The magic class 'FieldOperatorApplication' is a class that makes it easy to work with
 *  Autoware from C++. The main features of this class are as follows
 *
 *    (1) Launch Autoware in an independent process upon instantiation of the
 *        class.
 *    (2) Properly terminates the Autoware process started by the constructor
 *        upon destruction of the class.
 *    (3) Probably the simplest instructions to Autoware, consisting of
 *        initialize, plan, and engage.
 *
 * -------------------------------------------------------------------------- */
struct FieldOperatorApplication : public rclcpp::Node,
                                  public TransitionAssertion<FieldOperatorApplication>
{
  std::atomic<bool> is_stop_requested = false;

  bool is_autoware_exited = false;

  const pid_t process_id = 0;

  TaskQueue task_queue;

  bool initialize_was_called = false;

  std::string autoware_state;

  tier4_rtc_msgs::msg::CooperateStatusArray latest_cooperate_status_array;

  std::string minimum_risk_maneuver_state;

  std::string minimum_risk_maneuver_behavior;

  // clang-format off
  SubscriberWrapper<autoware_system_msgs::msg::AutowareState, ThreadSafety::safe> getAutowareState;
  SubscriberWrapper<autoware_control_msgs::msg::Control>                          getCommand;
  SubscriberWrapper<tier4_rtc_msgs::msg::CooperateStatusArray>                    getCooperateStatusArray;
  SubscriberWrapper<tier4_external_api_msgs::msg::Emergency>                      getEmergencyState;
#if __has_include(<autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>)
  SubscriberWrapper<autoware_adapi_v1_msgs::msg::LocalizationInitializationState> getLocalizationState;
#endif
  SubscriberWrapper<autoware_adapi_v1_msgs::msg::MrmState>                        getMrmState;
  SubscriberWrapper<tier4_planning_msgs::msg::PathWithLaneId>                     getPathWithLaneId;
  SubscriberWrapper<tier4_planning_msgs::msg::Trajectory>                         getTrajectory;
  SubscriberWrapper<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>            getTurnIndicatorsCommand;

  ServiceWithValidation<autoware_adapi_v1_msgs::srv::ClearRoute>                  requestClearRoute;
  ServiceWithValidation<tier4_rtc_msgs::srv::CooperateCommands>                   requestCooperateCommands;
  ServiceWithValidation<tier4_external_api_msgs::srv::Engage>                     requestEngage;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::InitializeLocalization>      requestInitialPose;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::SetRoutePoints>              requestSetRoutePoints;
  ServiceWithValidation<tier4_rtc_msgs::srv::AutoModeWithModule>                  requestSetRtcAutoMode;
  ServiceWithValidation<tier4_external_api_msgs::srv::SetVelocityLimit>           requestSetVelocityLimit;
  ServiceWithValidation<autoware_adapi_v1_msgs::srv::ChangeOperationMode>         requestEnableAutowareControl;
  // clang-format on

  CONCEALER_PUBLIC explicit FieldOperatorApplication(const pid_t = 0);

  template <typename... Ts>
  CONCEALER_PUBLIC explicit FieldOperatorApplication(Ts &&... xs)
  : FieldOperatorApplication(ros2_launch(std::forward<decltype(xs)>(xs)...))
  {
  }

  ~FieldOperatorApplication();

  /*
     NOTE: This predicate should not take the state being compared as an
     argument or template parameter. Otherwise, code using this class would
     need to have knowledge of the Autoware state type.
  */
#define DEFINE_STATE_PREDICATE(NAME, VALUE)                           \
  auto is##NAME() const noexcept { return autoware_state == #VALUE; } \
  static_assert(true, "")

  DEFINE_STATE_PREDICATE(Initializing, INITIALIZING_VEHICLE);
  DEFINE_STATE_PREDICATE(WaitingForRoute, WAITING_FOR_ROUTE);
  DEFINE_STATE_PREDICATE(Planning, PLANNING);
  DEFINE_STATE_PREDICATE(WaitingForEngage, WAITING_FOR_ENGAGE);
  DEFINE_STATE_PREDICATE(Driving, DRIVING);
  DEFINE_STATE_PREDICATE(ArrivedGoal, ARRIVAL_GOAL);
  DEFINE_STATE_PREDICATE(Finalizing, FINALIZING);

#undef DEFINE_STATE_PREDICATE

  auto spinSome() -> void;

  auto shutdownAutoware() -> void;

  auto engage() -> void;

  auto engageable() const -> bool;

  auto engaged() const -> bool;

  auto initialize(const geometry_msgs::msg::Pose &) -> void;

  auto plan(const std::vector<geometry_msgs::msg::PoseStamped> &) -> void;

  auto clearRoute() -> void;

  auto getAutowareStateName() const { return autoware_state; }

  auto getMinimumRiskManeuverBehaviorName() const { return minimum_risk_maneuver_behavior; }

  auto getMinimumRiskManeuverStateName() const { return minimum_risk_maneuver_state; }

  auto getEmergencyStateName() const { return minimum_risk_maneuver_state; }

  auto getWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray;

  auto initialized() const noexcept { return initialize_was_called; }

  auto requestAutoModeForCooperation(const std::string &, bool) -> void;

  auto rethrow() const { task_queue.rethrow(); }

  auto sendCooperateCommand(const std::string &, const std::string &) -> void;

  auto setVelocityLimit(double) -> void;

  auto enableAutowareControl() -> void;
};
}  // namespace concealer

#endif  // CONCEALER__AUTOWARE_USER_HPP_
