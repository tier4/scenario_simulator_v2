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
: configuration_(configuration), publisher_ptr_(publisher_ptr)
{
  last_update_stamp_ = 0;
}


void DetectionSensor::update(
  double current_time,
  const std::vector<openscenario_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp,
  const std::vector<std::string> & detected_objects)
{
  if ((current_time - last_update_stamp_) >= configuration_.update_duration()) {
    autoware_perception_msgs::msg::DynamicObjectArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = configuration_.entity();
    last_update_stamp_ = current_time;
    for (const auto & s : status) {
      auto result = std::find(detected_objects.begin(), detected_objects.end(), s.name());
      if (result != detected_objects.end()) {
        /*
        Generate Detection Result for Entity;
        */
      }
    }
    publisher_ptr_->publish(msg);
  }
}
}  // namespace scenario_simulator
