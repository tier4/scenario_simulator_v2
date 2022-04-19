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

#include <algorithm>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <memory>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
const std::vector<std::string> DetectionSensorBase::getDetectedObjects(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status) const
{
  std::vector<std::string> detected_objects;
  const auto pose = getSensorPose(status);
  for (const auto & s : status) {
    double distance = std::hypot(
      s.pose().position().x() - pose.position().x(), s.pose().position().y() - pose.position().y(),
      s.pose().position().z() - pose.position().z());
    if (s.name() != configuration_.entity() && distance <= configuration_.range()) {
      detected_objects.emplace_back(s.name());
    }
  }
  return detected_objects;
}

geometry_msgs::Pose DetectionSensorBase::getSensorPose(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status) const
{
  for (const auto & s : status) {
    if (
      s.type().type() == traffic_simulator_msgs::EntityType::EGO &&
      s.name() == configuration_.entity()) {
      return s.pose();
    }
  }
  throw SimulationRuntimeError("Detection sensor can be attached only ego entity.");
}

template <>
void DetectionSensor<autoware_auto_perception_msgs::msg::DetectedObjects>::update(
  const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp, const std::vector<std::string> & lidar_detected_entity)
{
  auto makeObjectClassification = [](const auto & label) {
    autoware_auto_perception_msgs::msg::ObjectClassification object_classification;
    {
      object_classification.label = label;
      object_classification.probability = 1;
    }

    return object_classification;
  };
  std::vector<std::string> detected_objects;
  if (configuration_.filter_by_range()) {
    detected_objects = getDetectedObjects(status);
  } else {
    detected_objects = lidar_detected_entity;
  }
  if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
    autoware_auto_perception_msgs::msg::DetectedObjects msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    last_update_stamp_ = current_time;
    for (const auto & s : status) {
      auto result = std::find(detected_objects.begin(), detected_objects.end(), s.name());
      if (result != detected_objects.end()) {
        autoware_auto_perception_msgs::msg::DetectedObject object;
        bool is_ego = false;
        if (s.type().type() == traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO) {
          is_ego = true;
        } else {
          switch (s.subtype().value()) {
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_UNKNOWN:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_CAR:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::CAR));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRUCK:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BUS:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::BUS));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_TRAILER:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_MOTORCYCLE:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BICYCLE:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE));
              break;
            case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_PEDESTRIAN:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN));
              break;
            default:
              object.classification.push_back(makeObjectClassification(
                autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN));
              break;
          }
        }
        if (not is_ego) {
          simulation_interface::toMsg(s.bounding_box().dimensions(), object.shape.dimensions);
          geometry_msgs::msg::Pose pose;
          simulation_interface::toMsg(s.pose(), pose);
          object.kinematics.pose_with_covariance.pose = pose;
          object.kinematics.pose_with_covariance.covariance = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                               0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                               0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
          simulation_interface::toMsg(
            s.action_status().twist(), object.kinematics.twist_with_covariance.twist);
          object.shape.type = object.shape.BOUNDING_BOX;
          msg.objects.emplace_back(object);
        }
      }
    }
    publisher_ptr_->publish(msg);
  }
}
}  // namespace simple_sensor_simulator
