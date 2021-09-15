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

#ifndef TRAFFIC_SIMULATOR__TEST__CATALOGS_HPP_
#define TRAFFIC_SIMULATOR__TEST__CATALOGS_HPP_

#include <openscenario_msgs/msg/misc_object_parameters.hpp>
#include <openscenario_msgs/msg/pedestrian_parameters.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <string>

auto getVehicleParameters() -> openscenario_msgs::msg::VehicleParameters
{
  openscenario_msgs::msg::VehicleParameters parameters;
  parameters.name = "vehicle.volkswagen.t";
  parameters.vehicle_category = "car";
  parameters.performance.max_speed = 69.444;
  parameters.performance.max_acceleration = 200;
  parameters.performance.max_deceleration = 10.0;
  parameters.bounding_box.center.x = 1.5;
  parameters.bounding_box.center.y = 0.0;
  parameters.bounding_box.center.z = 0.9;
  parameters.bounding_box.dimensions.x = 4.5;
  parameters.bounding_box.dimensions.y = 2.1;
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

auto getPedestrianParameters() -> openscenario_msgs::msg::PedestrianParameters
{
  openscenario_msgs::msg::PedestrianParameters parameters;
  parameters.name = "pedestrian";
  parameters.pedestrian_category = "pedestrian";
  parameters.bounding_box.center.x = 0.0;
  parameters.bounding_box.center.y = 0.0;
  parameters.bounding_box.center.z = 0.5;
  parameters.bounding_box.dimensions.x = 1.0;
  parameters.bounding_box.dimensions.y = 1.0;
  parameters.bounding_box.dimensions.z = 2.0;
  return parameters;
}

auto getMiscObjectParameters() -> openscenario_msgs::msg::MiscObjectParameters
{
  openscenario_msgs::msg::MiscObjectParameters misc_object_param;
  misc_object_param.bounding_box.dimensions.x = 1.0;
  misc_object_param.bounding_box.dimensions.y = 1.0;
  misc_object_param.bounding_box.dimensions.z = 1.0;
  misc_object_param.misc_object_category = "obstacle";
  misc_object_param.name = "obstacle";
  return misc_object_param;
}

#endif  // TRAFFIC_SIMULATOR__TEST__CATALOGS_HPP_
