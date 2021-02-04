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

#ifndef AWAPI_ACCESSOR__ACCESSOR_HPP_
#define AWAPI_ACCESSOR__ACCESSOR_HPP_

// Note: headers are lexicographically sorted.

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <awapi_accessor/define_macro.hpp>
#include <awapi_accessor/utility/visibility.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

namespace autoware_api
{

struct AutowareError
{
};

class Accessor : public rclcpp::Node
{
  std::mutex mutex;

public:
#ifndef NDEBUG
  /** ---- DummyData -----------------------------------------------------------
   *
   *  Topic: ~/dummy
   *
   * ------------------------------------------------------------------------ */
  using DebugString = std_msgs::msg::String;

  DEFINE_PUBLISHER(DebugString);
  DEFINE_SUBSCRIPTION(DebugString);
#endif

  /** ---- AutowareEngage ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/engage
   *
   * ------------------------------------------------------------------------ */
  using AutowareEngage = std_msgs::msg::Bool;

  DEFINE_PUBLISHER(AutowareEngage);

  /** ---- AutowareRoute -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/route
   *
   * ------------------------------------------------------------------------ */
  using AutowareRoute = autoware_planning_msgs::msg::Route;

  DEFINE_PUBLISHER(AutowareRoute);

  /** ---- LaneChangeApproval --------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/approval
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeApproval = std_msgs::msg::Bool;

  DEFINE_PUBLISHER(LaneChangeApproval);

  /** ---- LaneChangeForce -----------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/force
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeForce = std_msgs::msg::Bool;

  DEFINE_PUBLISHER(LaneChangeForce);

  /** ---- TrafficLightStateArray ----------------------------------------------
   *
   *  Overwrite the recognition result of traffic light.
   *
   *  Topic: /awapi/traffic_light/put/traffic_light
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStateArray = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_PUBLISHER(TrafficLightStateArray);

  /** ---- VehicleVelocity -----------------------------------------------------
   *
   *  Set upper bound of velocity.
   *
   *  Topic: /awapi/vehicle/put/velocity
   *
   * ------------------------------------------------------------------------ */
  using VehicleVelocity = std_msgs::msg::Float32;

  DEFINE_PUBLISHER(VehicleVelocity);

  /** ---- AutowareStatus ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/get/status
   *
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;

  DEFINE_SUBSCRIPTION(AutowareStatus);

  /** ---- TrafficLightStatus --------------------------------------------------
   *
   *  Topic: /awapi/traffic_light/get/status
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_SUBSCRIPTION(TrafficLightStatus);

  /** ---- VehicleStatus -------------------------------------------------------
   *
   *  Topic: /awapi/vehicle/get/status
   *
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

  DEFINE_SUBSCRIPTION(VehicleStatus);

public:
  /** ---- Checkpoint ----------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/checkpoint
   *
   * ------------------------------------------------------------------------ */
  using Checkpoint = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(Checkpoint);

  /** ---- GoalPose ------------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/goal
   *
   * ------------------------------------------------------------------------ */
  using GoalPose = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(GoalPose);

  /** ---- InitialPose ---------------------------------------------------------
   *
   *  Set initial pose of Autoware.
   *
   *  Topic: /initialpose
   *
   * ------------------------------------------------------------------------ */
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;

  DEFINE_PUBLISHER(InitialPose);

  /** ---- InitialTwist --------------------------------------------------------
   *
   *  Set initial velocity of Autoware.
   *
   *  Topic: /initialtwist
   *
   * ------------------------------------------------------------------------ */
  using InitialTwist = geometry_msgs::msg::TwistStamped;

  DEFINE_PUBLISHER(InitialTwist);

public:
  auto isWaitingForRoute() const noexcept
  {
    using autoware_system_msgs::msg::AutowareState;

    return CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::WAITING_FOR_ROUTE;
  }

  auto isEmergency() const noexcept
  {
    using autoware_system_msgs::msg::AutowareState;

    return CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::EMERGENCY;
  }

  auto isReady() const noexcept
  {
    static auto ready = false;

    return ready || (ready = isWaitingForRoute());
  }

  auto isNotReady() const noexcept
  {
    return !isReady();
  }

  void checkAutowareState() const
  {
    if (isReady() && isEmergency())
    {
      throw AutowareError();
    }
  }

public:
  template
  <
    typename ... Ts
  >
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(Ts && ... xs)
  : rclcpp::Node("awapi_accessor_node", std::forward<decltype(xs)>(xs)...),
#ifndef NDEBUG
    INIT_PUBLISHER(DebugString, "debug/string"),
    INIT_SUBSCRIPTION(DebugString, "debug/string", []() {}),
#endif
    // AWAPI topics (lexicographically sorted)
    INIT_PUBLISHER(AutowareEngage, "/autoware/put/engage"),
    INIT_PUBLISHER(AutowareRoute, "/autoware/put/route"),
    INIT_PUBLISHER(LaneChangeApproval, "/lane_change/put/approval"),
    INIT_PUBLISHER(LaneChangeForce, "/lane_change/put/force"),
    INIT_PUBLISHER(TrafficLightStateArray, "/traffic_light/put/traffic_light"),
    INIT_PUBLISHER(VehicleVelocity, "/vehicle/put/velocity"),
    INIT_SUBSCRIPTION(AutowareStatus, "/autoware/get/status", checkAutowareState),
    INIT_SUBSCRIPTION(TrafficLightStatus, "/traffic_light/get/status", []() {}),
    INIT_SUBSCRIPTION(VehicleStatus, "/vehicle/get/status", []() {}),

    // Simulation specific topics (lexicographically sorted)
    INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    INIT_PUBLISHER(InitialTwist, "/initialtwist")
  {}
};

}  // namespace autoware_api

#include <awapi_accessor/undefine_macro.hpp>

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
