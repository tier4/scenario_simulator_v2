// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <awapi_accessor/accessor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace autoware_api
{

#define MAKE_SUBSCRIPTION(TYPE, TOPIC) \
  subscription_of_ ## TYPE( \
    create_subscription<TYPE>( \
      TOPIC, 1, \
      [this](const TYPE::SharedPtr message) \
      { \
        current_value_of_ ## TYPE = *message; \
      }))

#define MAKE_PUBLICATION(TYPE, TOPIC) \
  publisher_of_ ## TYPE( \
    create_publisher<TYPE>(TOPIC, 10))

Accessor::Accessor(const rclcpp::NodeOptions & options)
: rclcpp::Node("autoware_api_accessor", options),
  MAKE_PUBLICATION(AutowareEngage, "/awapi/autoware/put/engage"),
  MAKE_PUBLICATION(AutowareRoute, "/awapi/autoware/put/route"),
  MAKE_PUBLICATION(LaneChangeApproval, "/awapi/lane_change/put/approval"),
  MAKE_PUBLICATION(LaneChangeForce, "/awapi/lane_change/put/force"),
  MAKE_PUBLICATION(TrafficLightStates, "/awapi/traffic_light/put/traffic_light"),
  MAKE_PUBLICATION(VehicleVelocity, "/awapi/vehicle/put/velocity"),
  MAKE_SUBSCRIPTION(AutowareStatus, "/awapi/autoware/get/status"),
  MAKE_SUBSCRIPTION(VehicleStatus, "/awapi/vehicle/get/status")
{}

#undef MAKE_SUBSCRIPTION
#undef MAKE_PUBLICATION

}  // namespace autoware_api

RCLCPP_COMPONENTS_REGISTER_NODE(autoware_api::Accessor)
