// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__FUNDAMENTAL_API_HPP_
#define CONCEALER__FUNDAMENTAL_API_HPP_

#if defined AUTOWARE_ARCHITECTURE_PROPOSAL
#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_api_msgs/msg/velocity_limit.hpp>
#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/lane_change_command.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#elif defined AUTOWARE_AUTO
// TODO (robotec.ai)
#endif

#include <cassert>
#include <concealer/conversion.hpp>
#include <concealer/define_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <type_traits>
#include <utility>

namespace concealer
{
template <typename Node>
class FundamentalAPI
{
public:
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  /* ---- AutowareEngage -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/engage
   *
   *  Overloads:
   *    setAutowareEngage(const AutowareEngage &) const
   *    setAutowareEngage(const bool) const
   *
   * ------------------------------------------------------------------------ */
  using AutowareEngage = autoware_vehicle_msgs::msg::Engage;

  DEFINE_PUBLISHER(AutowareEngage);

  decltype(auto) setAutowareEngage(const bool value = true)
  {
    return setAutowareEngage(convertTo<AutowareEngage>(value));
  }

  /* ---- AutowareRoute --------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/route
   *
   * ------------------------------------------------------------------------ */
  // using AutowareRoute = autoware_planning_msgs::msg::Route;
  //
  // DEFINE_PUBLISHER(AutowareRoute);

  /* ---- LaneChangeApproval ---------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/approval
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeApproval = autoware_planning_msgs::msg::LaneChangeCommand;

  DEFINE_PUBLISHER(LaneChangeApproval);

  decltype(auto) setLaneChangeApproval(const bool approve = true)
  {
    LaneChangeForce message;
    {
      message.stamp = static_cast<Node &>(*this).get_clock()->now();
      message.command = approve;
    }

    return setLaneChangeApproval(message);
  }

  /* ---- LaneChangeForce ------------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/force
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeForce = autoware_planning_msgs::msg::LaneChangeCommand;

  DEFINE_PUBLISHER(LaneChangeForce);

  decltype(auto) setLaneChangeForce(const bool force = true)
  {
    LaneChangeForce message;
    {
      message.stamp = static_cast<Node &>(*this).get_clock()->now();
      message.command = force;
    }

    return setLaneChangeForce(message);
  }

  /* ---- TrafficLightStateArray -----------------------------------------------
   *
   *  Overwrite the recognition result of traffic light.
   *
   *  Topic: /awapi/traffic_light/put/traffic_light
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStateArray = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_PUBLISHER(TrafficLightStateArray);

  /* ---- VehicleVelocity ------------------------------------------------------
   *
   *  Set upper bound of velocity.
   *
   *  Topic: /awapi/vehicle/put/velocity
   *
   * ------------------------------------------------------------------------ */
  // using VehicleVelocity = autoware_api_msgs::msg::VelocityLimit;
  //
  // DEFINE_PUBLISHER(VehicleVelocity);
  //
  // template <typename T, REQUIRES(std::is_convertible<T, decltype(VehicleVelocity::max_velocity)>)>
  // decltype(auto) setVehicleVelocity(const T value)
  // {
  //   VehicleVelocity vehicle_velocity;
  //   {
  //     vehicle_velocity.stamp = static_cast<Node &>(*this).get_clock()->now();
  //     vehicle_velocity.max_velocity = value;
  //   }
  //
  //   return setVehicleVelocity(vehicle_velocity);
  // }

  /* ---- AutowareStatus -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/get/status
   *
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;

  DEFINE_SUBSCRIPTION(AutowareStatus);

  /* ---- TrafficLightStatus ---------------------------------------------------
   *
   *  Topic: /awapi/traffic_light/get/status
   *
   * ------------------------------------------------------------------------ */
  // using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;
  //
  // DEFINE_SUBSCRIPTION(TrafficLightStatus);

  /* ---- VehicleStatus --------------------------------------------------------
   *
   *  Topic: /awapi/vehicle/get/status
   *
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

  DEFINE_SUBSCRIPTION(VehicleStatus);

public:
#define DEFINE_STATE_PREDICATE(NAME, VALUE)                                                   \
  auto is##NAME() const noexcept                                                              \
  {                                                                                           \
    using autoware_system_msgs::msg::AutowareState;                                           \
    assert(AutowareState::VALUE == #NAME);                                                    \
    return CONCEALER_CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::VALUE; \
  }                                                                                           \
  static_assert(true, "")

  DEFINE_STATE_PREDICATE(InitializingVehicle, INITIALIZING_VEHICLE);
  DEFINE_STATE_PREDICATE(WaitingForRoute, WAITING_FOR_ROUTE);
  DEFINE_STATE_PREDICATE(Planning, PLANNING);
  DEFINE_STATE_PREDICATE(WaitingForEngage, WAITING_FOR_ENGAGE);
  DEFINE_STATE_PREDICATE(Driving, DRIVING);
  DEFINE_STATE_PREDICATE(ArrivedGoal, ARRIVAL_GOAL);
  DEFINE_STATE_PREDICATE(Emergency, EMERGENCY);
  DEFINE_STATE_PREDICATE(Finalizing, FINALIZING);

#undef DEFINE_STATE_PREDICATE

  bool ready = false;

  auto isReady() noexcept { return ready or (ready = isWaitingForRoute()); }

  auto isNotReady() noexcept { return not isReady(); }

  void checkAutowareState()
  {
    if (isReady() and isEmergency()) {
      // throw common::AutowareError("Autoware is in emergency state now");
    }
  }
#endif

#ifdef AUTOWARE_AUTO
#endif
public:
  explicit FundamentalAPI()
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
    : INIT_PUBLISHER(AutowareEngage, "/awapi/autoware/put/engage"),
    // INIT_PUBLISHER(AutowareRoute, "/awapi/autoware/put/route"),
    INIT_PUBLISHER(LaneChangeApproval, "/awapi/lane_change/put/approval"),
    INIT_PUBLISHER(LaneChangeForce, "/awapi/lane_change/put/force"),
    INIT_PUBLISHER(TrafficLightStateArray, "/awapi/traffic_light/put/traffic_light_status"),
    // INIT_PUBLISHER(VehicleVelocity, "/awapi/vehicle/put/velocity"),
    INIT_SUBSCRIPTION(AutowareStatus, "/awapi/autoware/get/status", checkAutowareState),
    // INIT_SUBSCRIPTION(TrafficLightStatus, "/awapi/traffic_light/get/status", []() {}),
    INIT_SUBSCRIPTION(VehicleStatus, "/awapi/vehicle/get/status", []() {})
#endif

#ifdef AUTOWARE_AUTO
#endif
  {
  }
};
}  // namespace concealer

#endif  // CONCEALER__FUNDAMENTAL_API_HPP_
