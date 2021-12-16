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
template <>
void DetectionSensor<autoware_perception_msgs::msg::DynamicObjectArray>::update(
  const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp, const std::vector<std::string> & detected_objects)
{
  if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
    autoware_perception_msgs::msg::DynamicObjectArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    last_update_stamp_ = current_time;
    for (const auto & s : status) {
      auto result = std::find(detected_objects.begin(), detected_objects.end(), s.name());
      if (result != detected_objects.end()) {
        autoware_perception_msgs::msg::DynamicObject object;
        bool is_ego = false;
        switch (s.type()) {
          case traffic_simulator_msgs::EntityType::EGO:
            is_ego = true;
            break;
          case traffic_simulator_msgs::EntityType::VEHICLE:
            object.semantic.type = object.semantic.CAR;
            object.semantic.confidence = 1;
            break;
          case traffic_simulator_msgs::EntityType::PEDESTRIAN:
            object.semantic.type = object.semantic.PEDESTRIAN;
            object.semantic.confidence = 1;
            break;
          case traffic_simulator_msgs::EntityType::MISC_OBJECT:
            break;
          default:
            throw SimulationRuntimeError("unsupported entity type!");
            break;
        }
        if (!is_ego) {
          boost::uuids::uuid base =
            boost::uuids::string_generator()("0123456789abcdef0123456789abcdef");
          boost::uuids::name_generator gen(base);
          boost::uuids::uuid uuid = gen(s.name());
          std::copy(uuid.begin(), uuid.end(), object.id.uuid.begin());
          simulation_interface::toMsg(s.bounding_box().dimensions(), object.shape.dimensions);
          geometry_msgs::msg::Pose pose;
          simulation_interface::toMsg(s.pose(), pose);
          object.state.pose_covariance.pose = pose;
          object.state.pose_covariance.covariance = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                     0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
          object.shape.type = object.shape.BOUNDING_BOX;
          object.state.orientation_reliable = true;
          simulation_interface::toMsg(
            s.action_status().twist(), object.state.twist_covariance.twist);
          object.state.twist_reliable = true;
          simulation_interface::toMsg(
            s.action_status().accel(), object.state.acceleration_covariance.accel);
          object.state.acceleration_reliable = true;
          msg.objects.emplace_back(object);
        }
      }
    }
    publisher_ptr_->publish(msg);
  }
}

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
void DetectionSensor<autoware_auto_perception_msgs::msg::PredictedObjects>::update(
  const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp, const std::vector<std::string> & detected_objects)
{
  auto makeObjectClassification = [](const auto & label) {
    autoware_auto_perception_msgs::msg::ObjectClassification object_classification;
    {
      object_classification.label = label;
      object_classification.probability = 1;
    }

    return object_classification;
  };

  if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
    autoware_auto_perception_msgs::msg::PredictedObjects msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    last_update_stamp_ = current_time;
    for (const auto & s : status) {
      auto result = std::find(detected_objects.begin(), detected_objects.end(), s.name());
      if (result != detected_objects.end()) {
        autoware_auto_perception_msgs::msg::PredictedObject object;
        bool is_ego = false;
        switch (s.type()) {
          case traffic_simulator_msgs::EntityType::EGO:
            is_ego = true;
            break;
          case traffic_simulator_msgs::EntityType::VEHICLE:
            object.classification.push_back(makeObjectClassification(
              autoware_auto_perception_msgs::msg::ObjectClassification::CAR));
            break;
          case traffic_simulator_msgs::EntityType::PEDESTRIAN:
            object.classification.push_back(makeObjectClassification(
              autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN));
            break;
          case traffic_simulator_msgs::EntityType::MISC_OBJECT:
            break;
          default:
            throw SimulationRuntimeError("unsupported entity type!");
            break;
        }
        if (not is_ego) {
          simulation_interface::toMsg(s.bounding_box().dimensions(), object.shape.dimensions);
          geometry_msgs::msg::Pose pose;
          simulation_interface::toMsg(s.pose(), pose);
          object.kinematics.initial_pose_with_covariance.pose = pose;
          object.kinematics.initial_pose_with_covariance.covariance = {
            1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
          simulation_interface::toMsg(
            s.action_status().twist(), object.kinematics.initial_twist_with_covariance.twist);
          object.shape.type = object.shape.BOUNDING_BOX;
          msg.objects.emplace_back(object);
        }
      }
    }
    publisher_ptr_->publish(msg);
  }
}
#endif
}  // namespace simple_sensor_simulator
