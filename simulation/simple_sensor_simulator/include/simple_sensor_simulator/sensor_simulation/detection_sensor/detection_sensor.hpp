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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
class DetectionSensor
{
public:
  DetectionSensor(
    const double current_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    std::shared_ptr<rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>>
      publisher_ptr);
  void update(
    double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const rclcpp::Time & stamp, const std::vector<std::string> & detected_objects);

private:
  simulation_api_schema::DetectionSensorConfiguration configuration_;
  std::shared_ptr<rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>>
    publisher_ptr_;
  double last_update_stamp_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
