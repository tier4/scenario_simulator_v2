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

#include "customized_rvo2/Plugins/ObstaclePlugin.h"

#include "customized_rvo2/Agent.hpp"

using namespace RVO;
std::vector<RVO::Line> ObstaclePlugin::calcOrcaLines(RVO::Agent::SharedPtr agent)
{
  std::vector<RVO::Line> orca_lines;
  if (!is_active_) {
    return orca_lines;
  }
  Vector2 position = agent->getCenterPosition();
  Vector2 velocity = agent->getVelocity();
  float neighbor_dist = agent->getAgentConfig().neighbor_dist;
  ObstacleNeighborList neighbor_list =
    obstacle_kdtree_.computeObstacleNeighbors(position, neighbor_dist);
  const float invTimeHorizonObst = 1.0f / agent->getAgentConfig().time_horizon_obst;

  /* Create obstacle ORCA lines. */
  // Obstacle Lines
  for (size_t i = 0; i < neighbor_list.size(); ++i) {
    Obstacle::SharedPtr obstacle1 = neighbor_list[i].second;
    Obstacle::SharedPtr obstacle2 = obstacle1->next_obstacle_;

    const Vector2 relativePosition1 = obstacle1->point_ - position;
    const Vector2 relativePosition2 = obstacle2->point_ - position;
    const Vector2 position12 = obstacle2->point_ - obstacle1->point_;

    // Calculating the radius of an ellipse
    const float agent_angle = agent->getOrientationYaw();
    Vector2 relativePosition;
    if (dot(-relativePosition1, position12) < 0) {
      relativePosition = relativePosition1;
    } else if (dot(-relativePosition2, -position12) < 0) {
      relativePosition = relativePosition2;
    } else {
      const double norm13 = dot(position12, -relativePosition1) / abs(position12);
      relativePosition = relativePosition1 + normalize(position12) * norm13;
    }

    const float relative_angle = std::atan2(relativePosition.y(), relativePosition.x());
    const float agent_relative_tan = std::tan(relative_angle - agent_angle);
    const float agent_radius =
      agent->getAgentConfig().radius * agent->getAgentConfig().ellipticity *
      std::sqrt(
        (1 + agent_relative_tan * agent_relative_tan) /
        (agent->getAgentConfig().ellipticity * agent->getAgentConfig().ellipticity +
         agent_relative_tan * agent_relative_tan));
    //std::cout << "obstacle_radius: " << agent_radius << std::endl;
    /*
     * Check if velocity obstacle of obstacle is already taken care of by
     * previously constructed obstacle ORCA lines.
     */
    bool alreadyCovered = false;

    for (size_t j = 0; j < orca_lines.size(); ++j) {
      if (
        det(
          invTimeHorizonObst * relativePosition1 - orca_lines.at(j).point,
          orca_lines.at(j).direction) -
            invTimeHorizonObst * agent_radius >=
          -RVO_EPSILON &&
        det(
          invTimeHorizonObst * relativePosition2 - orca_lines.at(j).point,
          orca_lines.at(j).direction) -
            invTimeHorizonObst * agent_radius >=
          -RVO_EPSILON) {
        alreadyCovered = true;
        break;
      }
    }

    if (alreadyCovered) {
      continue;
    }

    /* Not yet covered. Check for collisions. */

    const float distSq1 = absSq(relativePosition1);
    const float distSq2 = absSq(relativePosition2);

    const float radiusSq = sqr(agent_radius);

    const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
    const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
    const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

    Line line;

    if (s < 0.0f && distSq1 <= radiusSq) {
      /* Collision with left vertex. Ignore if non-convex. */
      if (obstacle1->is_convex_) {
        line.point = Vector2(0.0f, 0.0f);
        line.direction = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
        /*****************************************/
        orca_lines.push_back(line);
        /*****************************************/
      }

      continue;
    } else if (s > 1.0f && distSq2 <= radiusSq) {
      /* Collision with right vertex. Ignore if non-convex
       * or if it will be taken care of by neighboring obstacle */
      if (obstacle2->is_convex_ && det(relativePosition2, obstacle2->unit_dir_) >= 0.0f) {
        line.point = Vector2(0.0f, 0.0f);
        line.direction = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
        /*****************************************/
        orca_lines.push_back(line);
        /*****************************************/
      }

      continue;
    } else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
      /* Collision with obstacle segment. */
      line.point = Vector2(0.0f, 0.0f);
      line.direction = -obstacle1->unit_dir_;
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    }

    /*
     * No collision.
     * Compute legs. When obliquely viewed, both legs can come from a single
     * vertex. Legs extend cut-off line when non-convex vertex.
     */

    Vector2 leftLegDirection, rightLegDirection;

    if (s < 0.0f && distSqLine <= radiusSq) {
      /*
       * Obstacle viewed obliquely so that left vertex
       * defines velocity obstacle.
       */
      if (!obstacle1->is_convex_) {
        /* Ignore obstacle. */
        continue;
      }

      obstacle2 = obstacle1;

      const float leg1 = std::sqrt(distSq1 - radiusSq);
      leftLegDirection = Vector2(
                           relativePosition1.x() * leg1 - relativePosition1.y() * agent_radius,
                           relativePosition1.x() * agent_radius + relativePosition1.y() * leg1) /
                         distSq1;
      rightLegDirection = Vector2(
                            relativePosition1.x() * leg1 + relativePosition1.y() * agent_radius,
                            -relativePosition1.x() * agent_radius + relativePosition1.y() * leg1) /
                          distSq1;
    } else if (s > 1.0f && distSqLine <= radiusSq) {
      /*
       * Obstacle viewed obliquely so that
       * right vertex defines velocity obstacle.
       */
      if (!obstacle2->is_convex_) {
        /* Ignore obstacle. */
        continue;
      }

      obstacle1 = obstacle2;

      const float leg2 = std::sqrt(distSq2 - radiusSq);
      leftLegDirection = Vector2(
                           relativePosition2.x() * leg2 - relativePosition2.y() * agent_radius,
                           relativePosition2.x() * agent_radius + relativePosition2.y() * leg2) /
                         distSq2;
      rightLegDirection = Vector2(
                            relativePosition2.x() * leg2 + relativePosition2.y() * agent_radius,
                            -relativePosition2.x() * agent_radius + relativePosition2.y() * leg2) /
                          distSq2;
    } else {
      /* Usual situation. */
      if (obstacle1->is_convex_) {
        const float leg1 = std::sqrt(distSq1 - radiusSq);
        leftLegDirection = Vector2(
                             relativePosition1.x() * leg1 - relativePosition1.y() * agent_radius,
                             relativePosition1.x() * agent_radius + relativePosition1.y() * leg1) /
                           distSq1;
      } else {
        /* Left vertex non-convex; left leg extends cut-off line. */
        leftLegDirection = -obstacle1->unit_dir_;
      }

      if (obstacle2->is_convex_) {
        const float leg2 = std::sqrt(distSq2 - radiusSq);
        rightLegDirection =
          Vector2(
            relativePosition2.x() * leg2 + relativePosition2.y() * agent_radius,
            -relativePosition2.x() * agent_radius + relativePosition2.y() * leg2) /
          distSq2;
      } else {
        /* Right vertex non-convex; right leg extends cut-off line. */
        rightLegDirection = obstacle1->unit_dir_;
      }
    }

    /*
     * Legs can never point into neighboring edge when convex vertex,
     * take cutoff-line of neighboring edge instead. If velocity projected on
     * "foreign" leg, no constraint is added.
     */

    const Obstacle::SharedPtr leftNeighbor = obstacle1->prev_obstacle_;

    bool isLeftLegForeign = false;
    bool isRightLegForeign = false;

    if (obstacle1->is_convex_ && det(leftLegDirection, -leftNeighbor->unit_dir_) >= 0.0f) {
      /* Left leg points into obstacle. */
      leftLegDirection = -leftNeighbor->unit_dir_;
      isLeftLegForeign = true;
    }

    if (obstacle2->is_convex_ && det(rightLegDirection, obstacle2->unit_dir_) <= 0.0f) {
      /* Right leg points into obstacle. */
      rightLegDirection = obstacle2->unit_dir_;
      isRightLegForeign = true;
    }

    /* Compute cut-off centers. */
    const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position);
    const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position);
    const Vector2 cutoffVec = rightCutoff - leftCutoff;

    /* Project current velocity on velocity obstacle. */

    /* Check if current velocity is projected on cutoff circles. */
    const float t =
      (obstacle1 == obstacle2 ? 0.5f : ((velocity - leftCutoff) * cutoffVec) / absSq(cutoffVec));
    const float tLeft = ((velocity - leftCutoff) * leftLegDirection);
    const float tRight = ((velocity - rightCutoff) * rightLegDirection);

    if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
      /* Project on left cut-off circle. */
      const Vector2 unitW = normalize(velocity - leftCutoff);

      line.direction = Vector2(unitW.y(), -unitW.x());
      line.point = leftCutoff + agent_radius * invTimeHorizonObst * unitW;
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    } else if (t > 1.0f && tRight < 0.0f) {
      /* Project on right cut-off circle. */
      const Vector2 unitW = normalize(velocity - rightCutoff);

      line.direction = Vector2(unitW.y(), -unitW.x());
      line.point = rightCutoff + agent_radius * invTimeHorizonObst * unitW;
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    }

    /*
     * Project on left leg, right leg, or cut-off line, whichever is closest
     * to velocity.
     */
    const float distSqCutoff =
      ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2)
         ? std::numeric_limits<float>::infinity()
         : absSq(velocity - (leftCutoff + t * cutoffVec)));
    const float distSqLeft =
      ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity()
                      : absSq(velocity - (leftCutoff + tLeft * leftLegDirection)));
    const float distSqRight =
      ((tRight < 0.0f) ? std::numeric_limits<float>::infinity()
                       : absSq(velocity - (rightCutoff + tRight * rightLegDirection)));

    if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
      /* Project on cut-off line. */
      line.direction = -obstacle1->unit_dir_;
      line.point = leftCutoff + agent_radius * invTimeHorizonObst *
                                  Vector2(-line.direction.y(), line.direction.x());
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    } else if (distSqLeft <= distSqRight) {
      /* Project on left leg. */
      if (isLeftLegForeign) {
        continue;
      }

      line.direction = leftLegDirection;
      line.point = leftCutoff + agent_radius * invTimeHorizonObst *
                                  Vector2(-line.direction.y(), line.direction.x());
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    } else {
      /* Project on right leg. */
      if (isRightLegForeign) {
        continue;
      }

      line.direction = -rightLegDirection;
      line.point = rightCutoff + agent_radius * invTimeHorizonObst *
                                   Vector2(-line.direction.y(), line.direction.x());
      /*****************************************/
      orca_lines.push_back(line);
      /*****************************************/
      continue;
    }
  }
  return orca_lines;
}

size_t ObstaclePlugin::addObstacle(
  const std::vector<RVO::Vector2> & vertices, bool process_obstacles = true)
{
  if (vertices.size() < 2) {
    return RVO::RVO_ERROR;
  }

  const size_t obstacleNo = obstacles_.size();

  for (size_t i = 0; i < vertices.size(); ++i) {
    auto obstacle = std::make_shared<RVO::Obstacle>();
    obstacle->point_ = vertices[i];

    if (i != 0) {
      obstacle->prev_obstacle_ = obstacles_.back();
      obstacle->prev_obstacle_->next_obstacle_ = obstacle;
    }

    if (i == vertices.size() - 1) {
      obstacle->next_obstacle_ = obstacles_[obstacleNo];
      obstacle->next_obstacle_->prev_obstacle_ = obstacle;
    }

    obstacle->unit_dir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

    if (vertices.size() == 2) {
      obstacle->is_convex_ = true;
    } else {
      obstacle->is_convex_ =
        (leftOf(
           vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i],
           vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
    }

    obstacle->id_ = obstacles_.size();

    obstacles_.emplace_back(obstacle);
  }

  if (process_obstacles) {
    processObstacles();
  }

  return obstacleNo;
}

std::vector<RVO::Vector2> ObstaclePlugin::getObstacleLines()
{
  std::vector<RVO::Vector2> lines;
  for (size_t index = 0; index < obstacles_.size(); index++) {
    auto vertex = obstacles_[index];
    lines.emplace_back(vertex->point_);
    lines.emplace_back(vertex->next_obstacle_->point_);
  }
  return lines;
}

void ObstaclePlugin::deleteObstacles()
{
  if (obstacles_.size() > 0) {
    obstacle_kdtree_.deleteObstacleTree();
    obstacles_.clear();
  }
}
