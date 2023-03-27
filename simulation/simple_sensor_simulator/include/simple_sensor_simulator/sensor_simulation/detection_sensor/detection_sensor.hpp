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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <memory>
#include <queue>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
class DetectionSensorBase
{
protected:
  double last_update_stamp_;

  simulation_api_schema::DetectionSensorConfiguration configuration_;

  explicit DetectionSensorBase(
    const double last_update_stamp,
    const simulation_api_schema::DetectionSensorConfiguration & configuration)
  : last_update_stamp_(last_update_stamp), configuration_(configuration)
  {
  }

  auto getDetectedObjects(const std::vector<traffic_simulator_msgs::EntityStatus> &) const
    -> std::vector<std::string>;

  auto getSensorPose(const std::vector<traffic_simulator_msgs::EntityStatus> &) const
    -> geometry_msgs::Pose;

public:
  virtual ~DetectionSensorBase() = default;

  virtual void update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entity) = 0;
};

template <typename T>
class DetectionSensor : public DetectionSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  std::mt19937 random_engine_;

  std::queue<std::pair<autoware_auto_perception_msgs::msg::DetectedObjects, double>> queue_objects_;

  auto applyPositionNoise(autoware_auto_perception_msgs::msg::DetectedObject)
    -> autoware_auto_perception_msgs::msg::DetectedObject;

public:
  explicit DetectionSensor(
    const double current_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher)
  : DetectionSensorBase(current_time, configuration),
    publisher_ptr_(publisher),
    random_engine_(configuration.random_seed())
  {
  }

  ~DetectionSensor() override = default;

  auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entity) -> void override;
};

template <>
auto DetectionSensor<autoware_auto_perception_msgs::msg::DetectedObjects>::applyPositionNoise(
  autoware_auto_perception_msgs::msg::DetectedObject)
  -> autoware_auto_perception_msgs::msg::DetectedObject;

template <>
auto DetectionSensor<autoware_auto_perception_msgs::msg::DetectedObjects>::update(
  const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
  const std::vector<std::string> & lidar_detected_entity) -> void;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
