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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <boost/uuid/string_generator.hpp>
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
auto DetectionSensorBase::getDetectedObjects(
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const
  -> std::vector<std::string>
{
  std::vector<std::string> detected_objects;
  const auto pose = getSensorPose(statuses);
  for (const auto & status : statuses) {
    if (double distance = std::hypot(
          status.pose().position().x() - pose.position().x(),
          status.pose().position().y() - pose.position().y(),
          status.pose().position().z() - pose.position().z());
        status.name() != configuration_.entity() && distance <= configuration_.range()) {
      detected_objects.emplace_back(status.name());
    }
  }
  return detected_objects;
}

auto DetectionSensorBase::getSensorPose(
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> geometry_msgs::Pose
{
  for (const auto & status : statuses) {
    if (
      status.type().type() == traffic_simulator_msgs::EntityType::EGO &&
      status.name() == configuration_.entity()) {
      return status.pose();
    }
  }
  throw SimulationRuntimeError("Detection sensor can be attached only ego entity.");
}

template <>
auto DetectionSensor<autoware_auto_perception_msgs::msg::DetectedObjects>::applyPositionNoise(
  autoware_auto_perception_msgs::msg::DetectedObject detected_object)
  -> autoware_auto_perception_msgs::msg::DetectedObject
{
  auto position_noise_distribution =
    std::normal_distribution<>(0.0, configuration_.pos_noise_stddev());
  detected_object.kinematics.pose_with_covariance.pose.position.x +=
    position_noise_distribution(random_engine_);
  detected_object.kinematics.pose_with_covariance.pose.position.y +=
    position_noise_distribution(random_engine_);
  return detected_object;
}

unique_identifier_msgs::msg::UUID generateUUIDMsg(const std::string & input)
{
  boost::uuids::string_generator generate_uuid;

  const auto uuid = generate_uuid(input);

  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(uuid.begin(), uuid.end(), uuid_msg.uuid.begin());

  return uuid_msg;
}

template <>
auto DetectionSensor<autoware_auto_perception_msgs::msg::DetectedObjects>::update(
  const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & statuses,
  const rclcpp::Time & stamp, const std::vector<std::string> & lidar_detected_entity) -> void
{
  auto makeObjectClassification = [](const auto & label) {
    autoware_auto_perception_msgs::msg::ObjectClassification object_classification;
    object_classification.label = label;
    object_classification.probability = 1;
    return object_classification;
  };
  if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
    const std::vector<std::string> detected_objects{
      configuration_.filter_by_range() ? getDetectedObjects(statuses) : lidar_detected_entity};
    autoware_auto_perception_msgs::msg::DetectedObjects msg;
    autoware_auto_perception_msgs::msg::TrackedObjects ground_truth_msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    last_update_stamp_ = current_time;
    for (const auto & status : statuses) {
      if (
        std::find(detected_objects.begin(), detected_objects.end(), status.name()) !=
          detected_objects.end() and
        status.type().type() != traffic_simulator_msgs::EntityType_Enum::EntityType_Enum_EGO) {
        autoware_auto_perception_msgs::msg::DetectedObject object;
        switch (status.subtype().value()) {
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
            object.kinematics.orientation_availability =
              autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
            break;
          case traffic_simulator_msgs::EntitySubtype_Enum::EntitySubtype_Enum_BICYCLE:
            object.classification.push_back(makeObjectClassification(
              autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE));
            object.kinematics.orientation_availability =
              autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
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
        simulation_interface::toMsg(status.bounding_box().dimensions(), object.shape.dimensions);
        geometry_msgs::msg::Pose pose;
        simulation_interface::toMsg(status.pose(), pose);
        auto rotation = quaternion_operation::getRotationMatrix(pose.orientation);
        geometry_msgs::msg::Point center_point;
        simulation_interface::toMsg(status.bounding_box().center(), center_point);
        Eigen::Vector3d center(center_point.x, center_point.y, center_point.z);
        center = rotation * center;
        pose.position.x = pose.position.x + center.x();
        pose.position.y = pose.position.y + center.y();
        pose.position.z = pose.position.z + center.z();
        object.kinematics.pose_with_covariance.pose = pose;
        object.kinematics.pose_with_covariance.covariance = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                             0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                             0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
        simulation_interface::toMsg(
          status.action_status().twist(), object.kinematics.twist_with_covariance.twist);
        object.shape.type = object.shape.BOUNDING_BOX;

        msg.objects.push_back(object);

        // ref: https://github.com/autowarefoundation/autoware.universe/blob/main/common/perception_utils/src/conversion.cpp
        static auto toTrackedObject =
          [&](
            const std::string & name,
            const autoware_auto_perception_msgs::msg::DetectedObject & detected_object)
          -> autoware_auto_perception_msgs::msg::TrackedObject {
          autoware_auto_perception_msgs::msg::TrackedObject tracked_object;
          tracked_object.existence_probability = detected_object.existence_probability;

          tracked_object.classification = detected_object.classification;

          tracked_object.kinematics.pose_with_covariance =
            detected_object.kinematics.pose_with_covariance;
          tracked_object.kinematics.twist_with_covariance =
            detected_object.kinematics.twist_with_covariance;
          tracked_object.kinematics.orientation_availability =
            detected_object.kinematics.orientation_availability;

          tracked_object.shape = detected_object.shape;
          tracked_object.object_id = generateUUIDMsg(name);

          return tracked_object;
        };

        ground_truth_msg.objects.push_back(toTrackedObject(status.name(), object));
      }
    }

    queue_objects_.push(std::make_pair(msg, current_time));
    autoware_auto_perception_msgs::msg::DetectedObjects delayed_objects;
    if (current_time - queue_objects_.front().second >= configuration_.object_recognition_delay()) {
      delayed_objects = queue_objects_.front().first;
      queue_objects_.pop();
    }

    autoware_auto_perception_msgs::msg::DetectedObjects noised_msg;
    noised_msg.header = delayed_objects.header;
    noised_msg.objects.reserve(delayed_objects.objects.size());
    for (const auto & object : delayed_objects.objects) {
      if (auto probability_of_lost = std::uniform_real_distribution();
          probability_of_lost(random_engine_) > configuration_.probability_of_lost()) {
        noised_msg.objects.push_back(applyPositionNoise(object));
      }
    }

    publisher_ptr_->publish(noised_msg);

    if (configuration_.enable_ground_truth_delay()) {
      static rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
        publisher = std::dynamic_pointer_cast<
          rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>>(
          ground_truth_publisher_base_ptr_);
      publisher->publish(ground_truth_msg);
    }
  }
}
}  // namespace simple_sensor_simulator
