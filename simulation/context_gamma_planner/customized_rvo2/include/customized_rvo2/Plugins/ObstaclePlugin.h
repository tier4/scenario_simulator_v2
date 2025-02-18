// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CUSTOMIZED_RVO2__PLUGINS__GLOBAL_OBSTACLE_PLUGIN_H
#define CUSTOMIZED_RVO2__PLUGINS__GLOBAL_OBSTACLE_PLUGIN_H

#include "customized_rvo2/KdTree.h"
#include "customized_rvo2/Obstacle.h"
#include "customized_rvo2/Plugins/OrcaPlugin.h"

class ObstaclePlugin : public OrcaObstaclePluginBase
{
public:
  using ObstacleNeighborList = std::vector<std::pair<float, RVO::Obstacle::SharedPtr>>;
  ObstaclePlugin() : OrcaObstaclePluginBase("ObstaclePlugin") {}
  std::vector<RVO::Line> calcOrcaLines(std::shared_ptr<RVO::Agent> agent) override;
  void processObstacles() { obstacle_kdtree_.buildObstacleTree(obstacles_); }
  std::vector<RVO::Vector2> getObstacleLines();
  size_t addObstacle(const std::vector<RVO::Vector2> & vertices, bool process_obstacles);
  void deleteObstacles();

private:
  std::vector<std::shared_ptr<RVO::Obstacle>> obstacles_;
  RVO::ObstacleKdTree obstacle_kdtree_;
};

#endif  // CUSTOMIZED_RVO2__PLUGINS__GLOBAL_OBSTACLE_PLUGIN_H
