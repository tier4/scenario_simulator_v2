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

  auto isWithinRange(
    const geometry_msgs::Point & point1, const geometry_msgs::Point & point2,
    const double range) const -> bool;

  auto filterObjectsBySensorRange(
    const std::vector<traffic_simulator_msgs::EntityStatus> &, const std::vector<std::string> &,
    const double) const -> std::vector<std::string>;

  auto getEntityPose(const std::vector<traffic_simulator_msgs::EntityStatus> &, const std::string &)
    const -> geometry_msgs::Pose;

  auto getDetectedObjects(const std::vector<traffic_simulator_msgs::EntityStatus> &) const
    -> std::vector<std::string>;

  auto getSensorPose(const std::vector<traffic_simulator_msgs::EntityStatus> &) const
    -> geometry_msgs::Pose;

public:
  virtual ~DetectionSensorBase() = default;

  virtual void update(
    const double current_simulation_time, const std::vector<traffic_simulator_msgs::EntityStatus> &,
    const rclcpp::Time & current_ros_time,
    const std::vector<std::string> & lidar_detected_entities) = 0;
};

template <typename T>
class DetectionSensor : public DetectionSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  typename rclcpp::PublisherBase::SharedPtr ground_truth_publisher_base_ptr_;

  std::mt19937 random_engine_;

  auto applyPositionNoise(typename T::_objects_type::value_type) ->
    typename T::_objects_type::value_type;

public:
  explicit DetectionSensor(
    const double current_simulation_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher,
    const typename rclcpp::PublisherBase::SharedPtr & ground_truth_publisher = nullptr)
  : DetectionSensorBase(current_simulation_time, configuration),
    publisher_ptr_(publisher),
    ground_truth_publisher_base_ptr_(ground_truth_publisher),
    random_engine_(configuration.random_seed())
  {
  }

  ~DetectionSensor() override = default;

  auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entity) -> void override;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
