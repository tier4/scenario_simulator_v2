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

#include "customized_rvo2/Plugins/AgentPlugin.h"

#include "customized_rvo2/Agent.hpp"

using namespace RVO;
/* Compute linear constraints on dynamic Agents */
std::vector<RVO::Line> AgentPlugin::calcOrcaLines(
  RVO::Agent::SharedPtr agent, std::shared_ptr<RVO::AgentKdTree> agent_kdtree, double time_step_s)
{
  std::vector<RVO::Line> orca_lines;
  if (!is_active_) {
    return orca_lines;
  }

  auto orcaLines = std::make_shared<std::vector<Line>>();
  const float inv_time_horizon = 1.0f / agent->getAgentConfig().time_horizon;
  Vector2 position = agent->getCenterPosition();
  Vector2 velocity = agent->getVelocity();

  AgentNeighborList neighbor_list =
    agent_kdtree->computeAgentNeighbors(position, agent->getAgentConfig().neighbor_dist);
  filterAgentNeighbors(agent, neighbor_list);
  /* Create agent ORCA lines. */
  // Collision avoidance between Agents
  for (auto neighbor : neighbor_list) {
    const auto other = neighbor.second;
    const Vector2 relative_position = other->getCenterPosition() - position;
    const Vector2 relative_velocity = velocity - other->getVelocity();
    const float dist_sq = absSq(relative_position);

    //Calculating the radius of an ellipse
    const float relative_angle = std::atan2(relative_position.y(), relative_position.x());
    /// @note: The radius of the ellipse is calculated from the relative angle of the other agent.
    auto getRadius = [](const Agent::SharedPtr agent, float relative_angle) {
      const float relative_tan = std::tan(relative_angle - agent->getOrientationYaw());
      return agent->getAgentConfig().radius * agent->getAgentConfig().ellipticity *
             std::sqrt(
               (1 + relative_tan * relative_tan) /
               (agent->getAgentConfig().ellipticity * agent->getAgentConfig().ellipticity +
                relative_tan * relative_tan));
    };
    const float agent_radius = getRadius(agent, relative_angle);
    const float other_radius = getRadius(other, M_PI + relative_angle);

    /// @note: Calculate as a circle when the distance is far, and as an ellipse when the distance is near.
    ///        If ellipses are used in all areas of the calculation, the radius changes abruptly
    ///        as the agents pass each other, and the path of movement is not smooth.
    const float max_combined_radius =
      agent->getAgentConfig().radius + other->getAgentConfig().radius;
    const float min_combined_radius = agent_radius + other_radius;
    float combined_radius = min_combined_radius;
    if (dist_sq > max_combined_radius) {
      combined_radius = max_combined_radius;
    } else {
      combined_radius =
        std::max(0.5f * (dist_sq - min_combined_radius) + min_combined_radius, min_combined_radius);
    }
    const float combined_radius_sq = sqr(combined_radius);

    Line line;
    Vector2 u;
    if (dist_sq >= combined_radius_sq) {
      /* When there is no conflict between itself and the target Dynamic Agent */
      /* No collision. */
      const Vector2 w = relative_velocity - inv_time_horizon * relative_position;
      /* Vector from cutoff center to relative velocity. */
      const float w_length_sq = absSq(w);
      /* Inner product of the dynamic Agent of itself and the target */
      const float dot_product1 = w * relative_position;

      if (dot_product1 < 0.0f && sqr(dot_product1) > combined_radius_sq * w_length_sq) {
        /* Project on cut-off circle. */
        const float w_length = std::sqrt(w_length_sq);
        const Vector2 unit_w = w / w_length;

        line.direction = Vector2(unit_w.y(), -unit_w.x());
        u = (combined_radius * inv_time_horizon - w_length) * unit_w;
      } else {
        /* Project on legs. */
        const float leg = std::sqrt(dist_sq - combined_radius_sq);

        if (det(relative_position, w) > 0.0f) {
          /* Project on left leg. */
          line.direction =
            Vector2(
              relative_position.x() * leg - relative_position.y() * combined_radius,
              relative_position.x() * combined_radius + relative_position.y() * leg) /
            dist_sq;
        } else {
          /* Project on right leg. */
          line.direction =
            -Vector2(
              relative_position.x() * leg + relative_position.y() * combined_radius,
              -relative_position.x() * combined_radius + relative_position.y() * leg) /
            dist_sq;
        }

        const float dot_product2 = relative_velocity * line.direction;

        u = dot_product2 * line.direction - relative_velocity;
      }
    } else {
      /* When there is a conflict between itself and the target Dynamic Agent */
      /* Collision. Project on cut-off circle of time timeStep. */
      if (time_step_s <= std::numeric_limits<double>::epsilon()) {
        THROW_SIMULATION_ERROR("Time step is too small : ", time_step_s, "[s]");
      }
      const float inv_time_step = 1.0f / time_step_s;

      /* Vector from cutoff center to relative velocity. */
      const Vector2 w = relative_velocity - inv_time_step * relative_position;

      const float w_length = abs(w);
      const Vector2 unit_w = w / w_length;
      line.direction = Vector2(unit_w.y(), -unit_w.x());
      u = (combined_radius * inv_time_step - w_length) * unit_w;
    }

    line.point = velocity + 0.5f * u;
    /*****************************************/
    orca_lines.push_back(line);
    /*****************************************/
  }

  return orca_lines;
}

void AgentPlugin::filterAgentNeighbors(
  RVO::Agent::SharedPtr agent, AgentNeighborList & neighbor_list)
{
  // filter by ignore_list_
  const auto & ignore_list = agent->getIgnoreList();
  neighbor_list.erase(
    std::remove_if(
      neighbor_list.begin(), neighbor_list.end(),
      [ignore_list](const std::pair<float, Agent::SharedPtr> & neighbor) -> bool {
        return std::any_of(
          ignore_list.begin(), ignore_list.end(), [neighbor](const uint8_t & entity_type) {
            return entity_type == neighbor.second->getAgentConfig().entity_type;
          });
      }),
    neighbor_list.end());

  // filter with agent->getAgentConfig().max_neighbors
  if (neighbor_list.size() > agent->getAgentConfig().max_neighbors) {
    // sort neighbors by distance (nearer first)
    std::sort(
      neighbor_list.begin(), neighbor_list.end(),
      [](
        const std::pair<float, Agent::SharedPtr> & lhs,
        const std::pair<float, Agent::SharedPtr> & rhs) { return lhs.first < rhs.first; });
    // erase more neighbors than agent->getAgentConfig().max_neighbors
    neighbor_list.erase(
      neighbor_list.begin() + agent->getAgentConfig().max_neighbors, neighbor_list.end());
  }

  neighbor_list.erase(std::remove_if(
    neighbor_list.begin(), neighbor_list.end(),
    [agent](const auto & x) -> bool { return x.second->name == agent->name; }));
}
