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
  double previous_simulation_time_;

  simulation_api_schema::DetectionSensorConfiguration configuration_;

  explicit DetectionSensorBase(
    const double current_simulation_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration)
  : previous_simulation_time_(current_simulation_time), configuration_(configuration)
  {
  }

  auto isEgoEntityStatusToWhichThisSensorIsAttached(
    const traffic_simulator_msgs::EntityStatus &) const -> bool;

  auto findEgoEntityStatusToWhichThisSensorIsAttached(
    const std::vector<traffic_simulator_msgs::EntityStatus> &) const
    -> std::vector<traffic_simulator_msgs::EntityStatus>::const_iterator;

public:
  virtual ~DetectionSensorBase() = default;

  virtual void update(
    const double current_simulation_time, const std::vector<traffic_simulator_msgs::EntityStatus> &,
    const rclcpp::Time & current_ros_time,
    const std::vector<std::string> & lidar_detected_entities) = 0;
};

template <typename T, typename U = autoware_perception_msgs::msg::TrackedObjects>
class DetectionSensor : public DetectionSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr detected_objects_publisher;

  const typename rclcpp::Publisher<U>::SharedPtr ground_truth_objects_publisher;

  std::default_random_engine random_engine_;

  std::queue<std::pair<autoware_perception_msgs::msg::DetectedObjects, double>>
    detected_objects_queue;

  std::queue<std::pair<autoware_perception_msgs::msg::TrackedObjects, double>>
    ground_truth_objects_queue;

public:
  explicit DetectionSensor(
    const double current_simulation_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher,
    const typename rclcpp::Publisher<U>::SharedPtr & ground_truth_publisher = nullptr)
  : DetectionSensorBase(current_simulation_time, configuration),
    detected_objects_publisher(publisher),
    ground_truth_objects_publisher(ground_truth_publisher),
    random_engine_(configuration.random_seed())
  {
  }

  ~DetectionSensor() override = default;

  auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entities) -> void override;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
