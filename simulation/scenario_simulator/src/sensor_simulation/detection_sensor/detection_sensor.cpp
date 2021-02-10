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

#include <scenario_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>

#include <memory>

namespace scenario_simulator
{
DetectionSensor::DetectionSensor(
  const simulation_api_schema::DetectionSensorConfiguration & configuration,
  std::shared_ptr<
    rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>> publisher_ptr)
{
}
}  // namespace scenario_simulator
