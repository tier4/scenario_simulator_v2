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
DetectionSensor::DetectionSensor(
  const double current_time,
  const simulation_api_schema::DetectionSensorConfiguration & configuration,
  std::shared_ptr<rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>>
    publisher_ptr)
: configuration_(configuration), publisher_ptr_(publisher_ptr)
{
  last_update_stamp_ = current_time;
}

void DetectionSensor::update(
  double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp, const std::vector<std::string> & detected_objects)
{
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
        switch (s.type()) {
          case traffic_simulator_msgs::EntityType::EGO:
            is_ego = true;
            break;
          case traffic_simulator_msgs::EntityType::VEHICLE:
            object.classification.type = object.classification.CAR;
            object.classification.confidence = 1;
            break;
          case traffic_simulator_msgs::EntityType::PEDESTRIAN:
            object.classification.type = object.classification.PEDESTRIAN;
            object.classification.confidence = 1;
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
          object.kinematics.pose_covariance.pose = pose;
          object.kinematics.pose_covariance.covariance = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                          0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                          0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
          object.shape.type = object.shape.BOUNDING_BOX;
          object.kinematics.orientation_reliable = true;
          simulation_interface::toMsg(
            s.action_status().twist(), object.kinematics.twist_covariance.twist);
          object.kinematics.twist_reliable = true;
          simulation_interface::toMsg(
            s.action_status().accel(), object.kinematics.acceleration_covariance.accel);
          object.kinematics.acceleration_reliable = true;
          msg.objects.emplace_back(object);
        }
      }
    }
    publisher_ptr_->publish(msg);
  }
}
}  // namespace simple_sensor_simulator
