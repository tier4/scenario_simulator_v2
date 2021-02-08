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

#ifndef SCENARIO_SIMULATOR__RAYCAST__LIDAR_SIMULATION_HPP_
#define SCENARIO_SIMULATOR__RAYCAST__LIDAR_SIMULATION_HPP_

#include <simulation_api_schema.pb.h>

#include <vector>

namespace scenario_simulator
{
class LidarSimulation
{
public:
  LidarSimulation();
  ~LidarSimulation();
  void raycast(const std::vector<openscenario_msgs::EntityStatus> & status);
  void addConfiguration(const simulation_api_schema::LidarConfiguration & configuration);

private:
  std::vector<simulation_api_schema::LidarConfiguration> lidar_models_;
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__RAYCAST__LIDAR_SIMULATION_HPP_
