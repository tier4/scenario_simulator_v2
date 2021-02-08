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

#include <scenario_simulator/raycast/lidar_simulation.hpp>
#include <scenario_simulator/raycast/raycaster.hpp>
#include <xmlrpc_interface/conversions.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <vector>

namespace scenario_simulator
{
LidarSimulation::LidarSimulation()
{
}

LidarSimulation::~LidarSimulation()
{
}

void LidarSimulation::raycast(
  const simulation_api_schema::LidarConfiguration & configuration,
  const std::vector<openscenario_msgs::EntityStatus> & status,
  const rclcpp::Time & stamp)
{
  Raycaster raycaster;
  for (const auto s : status) {
    geometry_msgs::msg::Pose pose;
    xmlrpc_interface::toMsg(s.pose(), pose);
    const auto bbox = s.bounding_box();
    auto rotation =
      quaternion_operation::getRotationMatrix(quaternion_operation::conjugate(pose.orientation));
    geometry_msgs::msg::Point center_point;
    xmlrpc_interface::toMsg(s.bounding_box().center(), center_point);
    Eigen::Vector3d center(center_point.x, center_point.y, center_point.z);
    center = rotation * center;
    pose.position.x = pose.position.x + center.x();
    pose.position.y = pose.position.y + center.y();
    pose.position.z = pose.position.z + center.z();
    raycaster.addPrimitive<scenario_simulator::primitives::Box>(
      s.name(),
      s.bounding_box().dimensions().x(),
      s.bounding_box().dimensions().y(),
      s.bounding_box().dimensions().z(),
      pose);
  }
  // raycaster.raycast(configuration.frame_id(), stamp, );
}
}  // namespace scenario_simulator
