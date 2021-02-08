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

namespace scenario_simulator
{
LidarSimulation::LidarSimulation()
{
}

LidarSimulation::~LidarSimulation()
{
}

void LidarSimulation::addConfiguration(
  const simulation_api_schema::LidarConfiguration & configuration)
{
  lidar_models_.emplace_back(configuration);
}

void LidarSimulation::raycast(const std::vector<openscenario_msgs::EntityStatus> & status)
{
  Raycaster raycaster;
  for (const auto s : status) {
    geometry_msgs::msg::Pose pose;
    xmlrpc_interface::toMsg(s.pose(), pose);
    const auto bbox = s.bounding_box();
    raycaster.addPrimitive<scenario_simulator::primitives::Box>(
      s.name(),
      s.bounding_box().dimensions().x(),
      s.bounding_box().dimensions().y(),
      s.bounding_box().dimensions().z(),
      pose);
  }
}
}  // namespace scenario_simulator
