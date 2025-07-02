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

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry/plane.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <get_parameter/get_parameter.hpp>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <unordered_map>
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
  std::optional<math::geometry::Plane> ego_plane_opt_{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> ego_plane_pose_opt_{std::nullopt};
};

template <typename T, typename U = autoware_perception_msgs::msg::TrackedObjects>
class DetectionSensor : public DetectionSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr detected_objects_publisher;

  const typename rclcpp::Publisher<U>::SharedPtr ground_truth_objects_publisher;

  int noise_model_version;

  std::default_random_engine random_engine_;

  std::queue<std::pair<std::vector<traffic_simulator_msgs::EntityStatus>, double>>
    unpublished_detected_entities, unpublished_ground_truth_entities;

  struct NoiseOutput
  {
    double simulation_time, distance_noise, yaw_noise;

    bool true_positive, flip;

    explicit NoiseOutput(double simulation_time = 0.0)
    : simulation_time(simulation_time),
      distance_noise(0.0),
      yaw_noise(0.0),
      true_positive(true),
      flip(false)
    {
    }
  };

  std::unordered_map<std::string, NoiseOutput> noise_outputs;

  template <typename Message, typename std::enable_if_t<std::is_same_v<Message, T>, int> = 0>
  auto delay() const
  {
    static const auto override_legacy_configuration = common::getParameter<bool>(
      detected_objects_publisher->get_topic_name() + std::string(".override_legacy_configuration"));
    if (override_legacy_configuration) {
      static const auto delay = common::getParameter<double>(
        detected_objects_publisher->get_topic_name() + std::string(".delay"));
      return delay;
    } else {
      return configuration_.object_recognition_delay();
    }
  }

  template <typename Message, typename std::enable_if_t<std::is_same_v<Message, U>, int> = 0>
  auto delay() const
  {
    static const auto override_legacy_configuration = common::getParameter<bool>(
      ground_truth_objects_publisher->get_topic_name() +
      std::string(".override_legacy_configuration"));
    if (override_legacy_configuration) {
      static const auto delay = common::getParameter<double>(
        ground_truth_objects_publisher->get_topic_name() + std::string(".delay"));
      return delay;
    } else {
      return configuration_.object_recognition_ground_truth_delay();
    }
  }

  auto range() const
  {
    static const auto override_legacy_configuration = common::getParameter<bool>(
      detected_objects_publisher->get_topic_name() + std::string(".override_legacy_configuration"));
    if (override_legacy_configuration) {
      static const auto range = common::getParameter<double>(
        detected_objects_publisher->get_topic_name() + std::string(".range"), 300.0);
      return range;
    } else {
      return configuration_.range();
    }
  }

  auto detect_all_objects_in_range() const
  {
    // cspell: ignore occlusionless
    static const auto override_legacy_configuration = common::getParameter<bool>(
      detected_objects_publisher->get_topic_name() + std::string(".override_legacy_configuration"));
    if (override_legacy_configuration) {
      static const auto occlusionless = common::getParameter<bool>(
        detected_objects_publisher->get_topic_name() + std::string(".occlusionless"));
      return occlusionless;
    } else {
      return configuration_.detect_all_objects_in_range();
    }
  }

public:
  explicit DetectionSensor(
    const double current_simulation_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher,
    const typename rclcpp::Publisher<U>::SharedPtr & ground_truth_publisher = nullptr)
  : DetectionSensorBase(current_simulation_time, configuration),
    detected_objects_publisher(publisher),
    ground_truth_objects_publisher(ground_truth_publisher),
    noise_model_version(common::getParameter<int>(
      detected_objects_publisher->get_topic_name() + std::string(".noise.model.version"))),
    random_engine_([this]() {
      if (const auto seed = [this]() -> std::random_device::result_type {
            switch (noise_model_version) {
              default:
                [[fallthrough]];
              case 1:
                return configuration_.random_seed();
              case 2:
                [[fallthrough]];
              case 3:
                return common::getParameter<int>(
                  detected_objects_publisher->get_topic_name() + std::string(".seed"));
            }
          }();
          seed) {
        if (std::random_device::min() <= seed and seed <= std::random_device::max()) {
          return seed;
        } else {
          throw common::scenario_simulator_exception::Error(
            "The value of parameter ",
            std::quoted(detected_objects_publisher->get_topic_name() + std::string(".seed")),
            " must be greater than or equal to ", std::random_device::min(),
            " and less than or equal to ", std::random_device::max());
        }
      } else {
        return std::random_device()();
      }
    }())
  {
  }

  ~DetectionSensor() override = default;

  auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entities) -> void override;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__DETECTION_SENSOR__DETECTION_SENSOR_HPP_
