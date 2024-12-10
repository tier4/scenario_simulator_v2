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

#include <geometry/plane.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <optional>
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

  auto isOnOrAboveEgoPlane(
    const geometry_msgs::Pose & npc_pose, const geometry_msgs::Pose & ego_pose) -> bool;

public:
  virtual ~DetectionSensorBase() = default;

  virtual void update(
    const double current_simulation_time, const std::vector<traffic_simulator_msgs::EntityStatus> &,
    const rclcpp::Time & current_ros_time,
    const std::vector<std::string> & lidar_detected_entities) = 0;

private:
  /*
      The threshold for detecting significant changes in ego vehicle's orientation (unit: radian).
      The value determines the minimum angular difference required to consider the ego orientation
      as "changed".

      There is no technical basis for this value, it was determined based on experiments.
  */
  constexpr static double rotation_threshold_ = 0.04;

  /*
      Maximum downward offset in Z-axis relative to the ego position (unit: meter).
      If the NPC is lower than this offset relative to the ego position,
      the NPC will be excluded from detection

      There is no technical basis for this value, it was determined based on experiments.
  */
  constexpr static double max_downward_z_offset_ = 1.0;

  geometry_msgs::msg::Pose ego_pose_;
  std::optional<geometry_msgs::msg::Pose> previous_ego_pose_{std::nullopt};
  std::optional<math::geometry::Plane> ego_plane_{std::nullopt};

  auto isEntityAltitudeAcceptable(const geometry_msgs::msg::Pose & entity_pose) const -> bool;
  auto needToUpdateEgoPlane() const -> bool;
  auto hasEgoOrientationChanged() const -> bool;
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
