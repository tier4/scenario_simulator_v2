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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random_test_runner/lanelet_utils.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>

traffic_simulator::CanonicalizedEntityStatus getCanonicalizedEntityStatus(
  const double time, const double twist_x, const double x, const double y = 0.0,
  const double z = 0.0, const std::string & name = "name")
{
  static const std::string path =
    ament_index_cpp::get_package_share_directory("random_test_runner") + "/map/lanelet2_map.osm";
  static const auto origin = geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
                               .latitude(35.61836750154)
                               .longitude(139.78066608243)
                               .altitude(0.0);
  static const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);

  traffic_simulator::EntityStatus entity_status;
  entity_status.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  entity_status.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  entity_status.time = time;
  entity_status.name = name;

  traffic_simulator_msgs::msg::ActionStatus action_status;
  action_status.current_action = std::string("action");
  action_status.twist.linear =
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(twist_x).y(0.0).z(0.0);
  action_status.accel = geometry_msgs::msg::Accel();
  entity_status.action_status = action_status;
  entity_status.pose.position.x = x;
  entity_status.pose.position.y = y;
  entity_status.pose.position.z = z;
  /// @note set invalid LaneletPose so pass std::nullopt
  return traffic_simulator::CanonicalizedEntityStatus(entity_status, std::nullopt);
}

inline traffic_simulator_msgs::msg::LaneletPose makeLaneletPose(
  const long int lane_id, const double s)
{
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lane_id;
  lanelet_pose.s = s;
  return lanelet_pose;
}

LaneletUtils & getLaneletUtils()
{
  static const std::string path =
    ament_index_cpp::get_package_share_directory("random_test_runner") + "/map/lanelet2_map.osm";
  static LaneletUtils utils(path);
  return utils;
}

std::shared_ptr<LaneletUtils> getLaneletUtilsPtr()
{
  static const std::string path =
    ament_index_cpp::get_package_share_directory("random_test_runner") + "/map/lanelet2_map.osm";
  static const auto utils = std::make_shared<LaneletUtils>(path);
  return utils;
}

NPCDescription makeNPCDescription(
  const std::string name, const double speed,
  const traffic_simulator_msgs::msg::LaneletPose lanelet_pose)
{
  NPCDescription description;
  description.name = name;
  description.speed = speed;
  description.start_position = lanelet_pose;
  return description;
}
