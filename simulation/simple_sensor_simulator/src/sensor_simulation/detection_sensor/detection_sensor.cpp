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
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <geometry/vector3/hypot.hpp>
#include <memory>
#include <random>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace simple_sensor_simulator
{
auto distance(const geometry_msgs::Pose & pose1, const geometry_msgs::Pose & pose2)
{
  return std::hypot(
    pose1.position().x() - pose2.position().x(),  //
    pose1.position().y() - pose2.position().y(),  //
    pose1.position().z() - pose2.position().z());
}

auto DetectionSensorBase::isEgoEntityStatusToWhichThisSensorIsAttached(
  const traffic_simulator_msgs::EntityStatus & status) const -> bool
{
  return status.name() == configuration_.entity() and
         status.type().type() == traffic_simulator_msgs::EntityType::EGO;
}

auto DetectionSensorBase::findEgoEntityStatusToWhichThisSensorIsAttached(
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const
  -> std::vector<traffic_simulator_msgs::EntityStatus>::const_iterator
{
  if (auto iter = std::find_if(
        statuses.begin(), statuses.end(),
        [this](const auto & status) {
          return isEgoEntityStatusToWhichThisSensorIsAttached(status);
        });
      iter != statuses.end()) {
    return iter;
  } else {
    throw SimulationRuntimeError("Detection sensor can be attached only ego entity.");
  }
}

template <typename To, typename... From>
auto make(From &&...) -> To;

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> unique_identifier_msgs::msg::UUID
{
  static auto generate_uuid = boost::uuids::name_generator(boost::uuids::random_generator()());
  const auto uuid = generate_uuid(status.name());
  unique_identifier_msgs::msg::UUID message;
  std::copy(uuid.begin(), uuid.end(), message.uuid.begin());
  return message;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::ObjectClassification
{
  auto object_classification = autoware_perception_msgs::msg::ObjectClassification();

  object_classification.label = [&]() {
    switch (status.subtype().value()) {
      case traffic_simulator_msgs::EntitySubtype::CAR:
        return autoware_perception_msgs::msg::ObjectClassification::CAR;
      case traffic_simulator_msgs::EntitySubtype::TRUCK:
        return autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      case traffic_simulator_msgs::EntitySubtype::BUS:
        return autoware_perception_msgs::msg::ObjectClassification::BUS;
      case traffic_simulator_msgs::EntitySubtype::TRAILER:
        return autoware_perception_msgs::msg::ObjectClassification::TRAILER;
      case traffic_simulator_msgs::EntitySubtype::MOTORCYCLE:
        return autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      case traffic_simulator_msgs::EntitySubtype::BICYCLE:
        return autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
      case traffic_simulator_msgs::EntitySubtype::PEDESTRIAN:
        return autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      default:
        return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
  }();

  object_classification.probability = 1;

  return object_classification;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> geometry_msgs::msg::Pose
{
  auto pose = geometry_msgs::msg::Pose();
  simulation_interface::toMsg(status.pose(), pose);

  auto center_point = geometry_msgs::msg::Point();
  simulation_interface::toMsg(status.bounding_box().center(), center_point);

  Eigen::Vector3d center = quaternion_operation::getRotationMatrix(pose.orientation) *
                           Eigen::Vector3d(center_point.x, center_point.y, center_point.z);

  pose.position.x = pose.position.x + center.x();
  pose.position.y = pose.position.y + center.y();
  pose.position.z = pose.position.z + center.z();

  return pose;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status) -> geometry_msgs::msg::Twist
{
  auto twist = geometry_msgs::msg::Twist();
  simulation_interface::toMsg(status.action_status().twist(), twist);
  return twist;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::DetectedObjectKinematics
{
  auto kinematics = autoware_perception_msgs::msg::DetectedObjectKinematics();

  kinematics.pose_with_covariance.pose = make<geometry_msgs::msg::Pose>(status);

  // clang-format off
  kinematics.pose_with_covariance.covariance = {
    /*
       Row-major representation of the 6x6 covariance matrix. The orientation
       parameters use a fixed-axis representation. In order, the parameters
       are: (x, y, z, rotation about X axis, rotation about Y axis, rotation
       about Z axis)
    */
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
  };
  // clang-format on

  kinematics.twist_with_covariance.twist = make<geometry_msgs::msg::Twist>(status);

  kinematics.orientation_availability = [&]() {
    switch (status.subtype().value()) {
      case traffic_simulator_msgs::EntitySubtype::BICYCLE:
      case traffic_simulator_msgs::EntitySubtype::MOTORCYCLE:
        return autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      default:
        return autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
    }
  }();

  return kinematics;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::Shape
{
  auto shape = autoware_perception_msgs::msg::Shape();
  simulation_interface::toMsg(status.bounding_box().dimensions(), shape.dimensions);
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  return shape;
}

template <>
auto make(const traffic_simulator_msgs::EntityStatus & status)
  -> autoware_perception_msgs::msg::DetectedObject
{
  auto detected_object = autoware_perception_msgs::msg::DetectedObject();
  detected_object.classification.push_back(
    make<autoware_perception_msgs::msg::ObjectClassification>(status));
  detected_object.kinematics =
    make<autoware_perception_msgs::msg::DetectedObjectKinematics>(status);
  detected_object.shape = make<autoware_perception_msgs::msg::Shape>(status);
  return detected_object;
}

template <>
auto make(
  const traffic_simulator_msgs::EntityStatus & status,
  const autoware_perception_msgs::msg::DetectedObject & detected_object)
  -> autoware_perception_msgs::msg::TrackedObject
{
  // ref: https://github.com/autowarefoundation/autoware.universe/blob/main/common/perception_utils/src/conversion.cpp
  auto tracked_object = autoware_perception_msgs::msg::TrackedObject();
  // clang-format off
  tracked_object.object_id                           = make<unique_identifier_msgs::msg::UUID>(status);
  tracked_object.existence_probability               = detected_object.existence_probability;
  tracked_object.classification                      = detected_object.classification;
  tracked_object.kinematics.orientation_availability = detected_object.kinematics.orientation_availability;
  tracked_object.kinematics.pose_with_covariance     = detected_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance    = detected_object.kinematics.twist_with_covariance;
  tracked_object.shape                               = detected_object.shape;
  // clang-format on
  return tracked_object;
};

struct DefaultNoiseApplicator
{
  const double current_simulation_time;

  const rclcpp::Time & current_ros_time;

  const traffic_simulator_msgs::EntityStatus & ego_entity_status;

  std::default_random_engine & random_engine;

  const simulation_api_schema::DetectionSensorConfiguration & detection_sensor_configuration;

  explicit DefaultNoiseApplicator(
    double current_simulation_time, const rclcpp::Time & current_ros_time,
    const traffic_simulator_msgs::EntityStatus & ego_entity_status,
    std::default_random_engine & random_engine,
    const simulation_api_schema::DetectionSensorConfiguration & detection_sensor_configuration)
  : current_simulation_time(current_simulation_time),
    current_ros_time(current_ros_time),
    ego_entity_status(ego_entity_status),
    random_engine(random_engine),
    detection_sensor_configuration(detection_sensor_configuration)
  {
  }

  DefaultNoiseApplicator(const DefaultNoiseApplicator &) = delete;

  DefaultNoiseApplicator(DefaultNoiseApplicator &&) = delete;

  auto operator=(const DefaultNoiseApplicator &) = delete;

  auto operator=(DefaultNoiseApplicator &&) = delete;

  auto operator()(autoware_perception_msgs::msg::DetectedObjects detected_objects)
    -> decltype(auto)
  {
    auto position_noise_distribution =
      std::normal_distribution<>(0.0, detection_sensor_configuration.pos_noise_stddev());

    for (auto && detected_object : detected_objects.objects) {
      detected_object.kinematics.pose_with_covariance.pose.position.x +=
        position_noise_distribution(random_engine);
      detected_object.kinematics.pose_with_covariance.pose.position.y +=
        position_noise_distribution(random_engine);
    }

    detected_objects.objects.erase(
      std::remove_if(
        detected_objects.objects.begin(), detected_objects.objects.end(),
        [this](auto &&) {
          return std::uniform_real_distribution()(random_engine) <
                 detection_sensor_configuration.probability_of_lost();
        }),
      detected_objects.objects.end());

    return detected_objects;
  }
};

struct EllipseBasedNoiseApplicator : public DefaultNoiseApplicator
{
  using DefaultNoiseApplicator::DefaultNoiseApplicator;
  const std::vector<double> ellipse_y_radius_values = {10, 20, 40, 60, 80, 120, 150, 180, 1000};
  const double ellipse_normalized_x_radius_tp_rate = 0.6;
  const double ellipse_normalized_x_radius_distance_mean = 1.8;
  const double ellipse_normalized_x_radius_distance_std = 1.8;
  const double ellipse_normalized_x_radius_yaw_mean = 0.6;
  const double ellipse_normalized_x_radius_yaw_std = 1.6;
  const std::vector<double> tp_rate_values = {0.92, 0.77, 0.74, 0.66, 0.57, 0.28, 0.09, 0.03, 0.00};
  const std::vector<double> distance_mean_values = {0.25, 0.27, 0.44, 0.67, 1.00, 3.00, 4.09, 3.40, 0.00};
  const std::vector<double> distance_std_values = {0.35, 0.54, 0.83, 1.14, 1.60, 3.56, 4.31, 3.61, 0.00};
  const std::vector<double> yaw_mean_values = {0.01, 0.01, 0.00, 0.03, 0.04, 0.00, 0.01, -1.93, 0.00};
  const std::vector<double> yaw_std_values = {0.45, 0.76, 0.79, 0.68, 0.76, 1.59, 1.71, 2.55, 0.00};

  double calculateEllipseDistance(double x, double y, double normalized_x_radius)
  {
    return std::hypot(x / normalized_x_radius, y);
  }

  double findValue(const std::vector<double>& radius_values, const std::vector<double>& target_values, double dist) {
      for (int i = 0; i < radius_values.size(); i++) {
          if (dist < radius_values[i]) {
              return target_values[i];
          }
      }
      return 0.0;
  }

  auto operator()(autoware_perception_msgs::msg::DetectedObjects detected_objects)
    -> decltype(auto)
  {
    double x, y, angle;
    double dist_tp_rate, dist_distance_mean, dist_distance_std, dist_yaw_mean, dist_yaw_std;

    std::vector<bool> is_masked(detected_objects.objects.size(), false);

    for (std::size_t i = 0; i < detected_objects.objects.size(); ++i) {
      auto & detected_object = detected_objects.objects[i];
      auto & pose = detected_object.kinematics.pose_with_covariance.pose;

      // Calculate the normalized distance from the object to the ego vehicle.
      x = pose.position.x - ego_entity_status.pose().position().x();
      y = pose.position.y - ego_entity_status.pose().position().y();
      angle = std::atan2(y, x);

      dist_tp_rate = calculateEllipseDistance(x, y, ellipse_normalized_x_radius_tp_rate);
      dist_distance_mean = calculateEllipseDistance(x, y, ellipse_normalized_x_radius_distance_mean);
      dist_distance_std = calculateEllipseDistance(x, y, ellipse_normalized_x_radius_distance_std);
      dist_yaw_mean = calculateEllipseDistance(x, y, ellipse_normalized_x_radius_yaw_mean);
      dist_yaw_std = calculateEllipseDistance(x, y, ellipse_normalized_x_radius_yaw_std);

      // find noise parameters
      double tp_rate = findValue(ellipse_y_radius_values, tp_rate_values, dist_tp_rate);
      double distance_mean = findValue(ellipse_y_radius_values, distance_mean_values, dist_distance_mean);
      double distance_std = findValue(ellipse_y_radius_values, distance_std_values, dist_distance_std);
      double yaw_mean = findValue(ellipse_y_radius_values, yaw_mean_values, dist_yaw_mean);
      double yaw_std = findValue(ellipse_y_radius_values, yaw_std_values, dist_yaw_std);

      is_masked[i] = std::uniform_real_distribution<double>()(random_engine) > tp_rate;


      // Apply noise
      if (is_masked[i])
        continue;

      double dis_offset = std::normal_distribution<double>(distance_mean, distance_std)(random_engine);
      pose.position.x += dis_offset * std::cos(angle);
      pose.position.y += dis_offset * std::sin(angle);

      double yaw_offset = std::normal_distribution<double>(yaw_mean, yaw_std)(random_engine);
      tf2::Quaternion obj_orientation;
      tf2::convert(pose.orientation, obj_orientation);
      obj_orientation *= tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_offset);
      tf2::convert(obj_orientation, pose.orientation);
    }

    // random mask
    int idx = 0;
    detected_objects.objects.erase(
      std::remove_if(
        detected_objects.objects.begin(), detected_objects.objects.end(),
        [&](auto &&) {
          return is_masked[idx++];
        }),
      detected_objects.objects.end());

    return detected_objects;
  }
};

template <>
auto DetectionSensor<autoware_perception_msgs::msg::DetectedObjects>::update(
  const double current_simulation_time,
  const std::vector<traffic_simulator_msgs::EntityStatus> & statuses,
  const rclcpp::Time & current_ros_time, const std::vector<std::string> & lidar_detected_entities)
  -> void
{
  if (
    current_simulation_time - previous_simulation_time_ - configuration_.update_duration() >=
    -0.002) {
    previous_simulation_time_ = current_simulation_time;

    autoware_perception_msgs::msg::DetectedObjects detected_objects;
    detected_objects.header.stamp = current_ros_time;
    detected_objects.header.frame_id = "map";

    autoware_perception_msgs::msg::TrackedObjects ground_truth_objects;
    ground_truth_objects.header = detected_objects.header;

    const auto ego_entity_status = findEgoEntityStatusToWhichThisSensorIsAttached(statuses);

    auto is_in_range = [&](const auto & status) {
      return not isEgoEntityStatusToWhichThisSensorIsAttached(status) and
             distance(status.pose(), ego_entity_status->pose()) <= configuration_.range() and
             (configuration_.detect_all_objects_in_range() or
              std::find(
                lidar_detected_entities.begin(), lidar_detected_entities.end(), status.name()) !=
                lidar_detected_entities.end());
    };

    for (const auto & status : statuses) {
      if (is_in_range(status)) {
        const auto detected_object =
          make<autoware_perception_msgs::msg::DetectedObject>(status);
        detected_objects.objects.push_back(detected_object);
        ground_truth_objects.objects.push_back(
          make<autoware_perception_msgs::msg::TrackedObject>(status, detected_object));
      }
    }

    if (detected_objects_queue.emplace(detected_objects, current_simulation_time);
        current_simulation_time - detected_objects_queue.front().second >=
        configuration_.object_recognition_delay()) {
      auto apply_noise = EllipseBasedNoiseApplicator(
        current_simulation_time, current_ros_time, *ego_entity_status, random_engine_,
        configuration_);
      detected_objects_publisher->publish(apply_noise(detected_objects_queue.front().first));
      detected_objects_queue.pop();
    }

    if (ground_truth_objects_queue.emplace(ground_truth_objects, current_simulation_time);
        current_simulation_time - ground_truth_objects_queue.front().second >=
        configuration_.object_recognition_ground_truth_delay()) {
      ground_truth_objects_publisher->publish(ground_truth_objects_queue.front().first);
      ground_truth_objects_queue.pop();
    }
  }
}
}  // namespace simple_sensor_simulator
