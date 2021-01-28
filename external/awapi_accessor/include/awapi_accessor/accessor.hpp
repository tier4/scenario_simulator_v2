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

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <awapi_accessor/define_macro.hpp>
#include <awapi_accessor/utility/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace autoware_api
{

class Accessor
{
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

public:
  template
  <
    typename Node
  >
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(Node && node)
  : MAKE_PUBLISHER(AutowareEngage, "/awapi/autoware/put/engage"),
    MAKE_PUBLISHER(AutowareRoute, "/awapi/autoware/put/route"),
    MAKE_PUBLISHER(LaneChangeApproval, "/awapi/lane_change/put/approval"),
    MAKE_PUBLISHER(LaneChangeForce, "/awapi/lane_change/put/force"),
    MAKE_PUBLISHER(TrafficLightStateArray, "/awapi/traffic_light/put/traffic_light"),
    MAKE_PUBLISHER(VehicleVelocity, "/awapi/vehicle/put/velocity"),
    MAKE_SUBSCRIPTION(AutowareStatus, "/awapi/autoware/get/status"),
    MAKE_SUBSCRIPTION(TrafficLightStatus, "/awapi/traffic_light/get/status"),
    MAKE_SUBSCRIPTION(VehicleStatus, "/awapi/vehicle/get/status"),
#ifndef NDEBUG
    MAKE_PUBLISHER(DebugString, "debug/string"),
    MAKE_SUBSCRIPTION(DebugString, "debug/string")
#endif
  {}
};

}  // namespace autoware_api

#include <awapi_accessor/undefine_macro.hpp>

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
