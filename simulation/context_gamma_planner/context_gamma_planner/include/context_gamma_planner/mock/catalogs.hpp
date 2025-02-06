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

#ifndef CONTEXT_GAMMA_PLANNER__MOCK__CATALOGS_HPP_
#define CONTEXT_GAMMA_PLANNER__MOCK__CATALOGS_HPP_

#include <string>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace context_gamma_planner
{
namespace mock
{
class Catalogs
{
public:
  /**
   * @brief Get the default vehicle parameters.
   * @return traffic_simulator_msgs::msg::VehicleParameters The vehicle parameters.
   */
  auto getVehicleParameters() const -> traffic_simulator_msgs::msg::VehicleParameters
  {
    traffic_simulator_msgs::msg::VehicleParameters parameters;
    parameters.name = "vehicle.volkswagen.t";
    parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
    parameters.performance.max_speed = 69.444;
    parameters.performance.max_acceleration = 200;
    parameters.performance.max_deceleration = 10.0;
    parameters.bounding_box.center.x = 1.5;
    parameters.bounding_box.center.y = 0.0;
    parameters.bounding_box.center.z = 0.9;
    parameters.bounding_box.dimensions.x = 4.5;
    parameters.bounding_box.dimensions.y = 1.7;
    parameters.bounding_box.dimensions.z = 1.8;
    parameters.axles.front_axle.max_steering = 0.5;
    parameters.axles.front_axle.wheel_diameter = 0.6;
    parameters.axles.front_axle.track_width = 1.8;
    parameters.axles.front_axle.position_x = 3.1;
    parameters.axles.front_axle.position_z = 0.3;
    parameters.axles.rear_axle.max_steering = 0.0;
    parameters.axles.rear_axle.wheel_diameter = 0.6;
    parameters.axles.rear_axle.track_width = 1.8;
    parameters.axles.rear_axle.position_x = 0.0;
    parameters.axles.rear_axle.position_z = 0.3;
    return parameters;
  }

  /**
   * @brief Get the default pedestrian parameters.
   * @return traffic_simulator_msgs::msg::PedestrianParameters The pedestrian parameters.
   */
  auto getPedestrianParameters() const -> traffic_simulator_msgs::msg::PedestrianParameters
  {
    traffic_simulator_msgs::msg::PedestrianParameters parameters;
    parameters.name = "pedestrian";
    parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
    parameters.bounding_box.center.x = 0.0;
    parameters.bounding_box.center.y = 0.0;
    parameters.bounding_box.center.z = 0.5;
    parameters.bounding_box.dimensions.x = 1.0;
    parameters.bounding_box.dimensions.y = 1.0;
    parameters.bounding_box.dimensions.z = 2.0;
    return parameters;
  }

  /**
   * @brief Get the default parameters of a miscellaneous object.
   * @return traffic_simulator_msgs::msg::MiscObjectParameters The parameters of the miscellaneous object.
   */
  auto getMiscObjectParameters() const -> traffic_simulator_msgs::msg::MiscObjectParameters
  {
    traffic_simulator_msgs::msg::MiscObjectParameters misc_object_param;
    misc_object_param.bounding_box.dimensions.x = 1.0;
    misc_object_param.bounding_box.dimensions.y = 1.0;
    misc_object_param.bounding_box.dimensions.z = 1.0;
    misc_object_param.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
    misc_object_param.name = "obstacle";
    return misc_object_param;
  }
};
}  // namespace mock
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__MOCK__CATALOGS_HPP_
