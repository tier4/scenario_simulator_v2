// Copyright 2022 Tier IV, Inc All rights reserved.
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

#ifndef CUSTOMIZED_RVO2__VISUALIZEMARKER_HPP_
#define CUSTOMIZED_RVO2__VISUALIZEMARKER_HPP_

#include "customized_rvo2/RVOSimulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace RVO
{
/**
 *
 */
class VisualizeMarker
{
public:
  void visualizeAgents(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeObstacles(visualization_msgs::msg::MarkerArray & marker_array, RVOSimulator & sim_);
  void visualizeObstacleORCALine(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeAgentORCALine(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeVelocitySpace(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeAgentVelocity(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeAgentPrefVelocity(
    visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_);
  void visualizeAllMarkers(
    visualization_msgs::msg::MarkerArray & marker_array, RVOSimulator & sim_);
  void visualizeNextGoal(
    visualization_msgs::msg::MarkerArray & marker_array,
    const geometry_msgs::msg::Point & goal_point);
  void visualizeGoalPoses(
    visualization_msgs::msg::MarkerArray & marker_array,
    const std::vector<geometry_msgs::msg::Pose> & goal_poses);
};
}  // namespace RVO

#endif  // CUSTOMIZED_RVO2__VISUALIZEMARKER_HPP_
