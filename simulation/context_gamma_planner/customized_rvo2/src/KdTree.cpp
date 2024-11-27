/*
 * KdTree.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 *
 * modified by Kotaro Yoshimoto <kotaro.yoshimoto@tier4.jp>
 */

#include "customized_rvo2/KdTree.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "customized_rvo2/Agent.hpp"
#include "customized_rvo2/Obstacle.h"

namespace RVO
{
AgentKdTree::AgentKdTree() {}
void AgentKdTree::rebuildAgentTree(
  std::unordered_map<std::string, std::shared_ptr<Agent>> & all_agents)
{
  if (!agents_.empty()) agents_.clear();
  if (!agent_tree_.empty()) agent_tree_.clear();
  agents_.reserve(all_agents.size());
  for (auto agent : all_agents) {
    agents_.emplace_back(agent.second);
  }
  agent_tree_.resize(2 * agents_.size() - 1);

  if (!agents_.empty()) {
    buildAgentTreeRecursive(0, agents_.size(), 0);
  }
}
/*
void AgentKdTree::updateAgentTree(std::unordered_map<std::string, std::shared_ptr<Agent>> & all_agents)
{
  if (agents_.size() < all_agents.size()) {
    for (size_t i = agents_.size(); i < all_agents.size(); ++i) {
      agents_.push_back(all_agents[i]);
    }

    agent_tree_.resize(2 * agents_.size() - 1);
  }

  if (!agents_.empty()) {
    buildAgentTreeRecursive(0, agents_.size(), 0);
  }
}*/
void AgentKdTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
{
  agent_tree_[node].begin = begin;
  agent_tree_[node].end = end;
  agent_tree_[node].min_x = agent_tree_[node].max_x = agents_[begin]->getCenterPosition().x();
  agent_tree_[node].min_y = agent_tree_[node].max_y = agents_[begin]->getCenterPosition().y();

  for (size_t i = begin + 1; i < end; ++i) {
    agent_tree_[node].max_x =
      std::max(agent_tree_[node].max_x, agents_[i]->getCenterPosition().x());
    agent_tree_[node].min_x =
      std::min(agent_tree_[node].min_x, agents_[i]->getCenterPosition().x());
    agent_tree_[node].max_y =
      std::max(agent_tree_[node].max_y, agents_[i]->getCenterPosition().y());
    agent_tree_[node].min_y =
      std::min(agent_tree_[node].min_y, agents_[i]->getCenterPosition().y());
  }

  if (end - begin > MAX_LEAF_SIZE) {
    /* No leaf node. */
    const bool is_vertical =
      (agent_tree_[node].max_x - agent_tree_[node].min_x >
       agent_tree_[node].max_y - agent_tree_[node].min_y);
    const auto splitValue = [&]() -> float {
      if (is_vertical) {
        return 0.5f * (agent_tree_[node].max_x + agent_tree_[node].min_x);
      }
      return 0.5f * (agent_tree_[node].max_y + agent_tree_[node].min_y);
    };
    size_t left = begin;
    size_t right = end;

    while (left < right) {
      auto verticalPosition = [&](size_t index) {
        if (is_vertical) {
          return agents_[index]->getCenterPosition().x();
        }
        return agents_[index]->getCenterPosition().y();
      };
      while (left < right && verticalPosition(left) < splitValue()) {
        ++left;
      }
      while (right > left && verticalPosition(right - 1) >= splitValue()) {
        --right;
      }
      if (left < right) {
        std::swap(agents_[left], agents_[right - 1]);
        ++left;
        --right;
      }
    }

    if (left == begin) {
      ++left;
      ++right;
    }

    agent_tree_[node].left = node + 1;
    agent_tree_[node].right = node + 2 * (left - begin);

    buildAgentTreeRecursive(begin, left, agent_tree_[node].left);
    buildAgentTreeRecursive(left, end, agent_tree_[node].right);
  }
}
std::vector<std::pair<float, std::shared_ptr<Agent>>> AgentKdTree::computeAgentNeighbors(
  const RVO::Vector2 position, const float range) const
{
  float range_sq = sqr(range);
  std::vector<std::pair<float, std::shared_ptr<Agent>>> ret;
  queryAgentTreeRecursive(position, range_sq, 0, ret);
  return ret;
}
void AgentKdTree::queryAgentTreeRecursive(
  RVO::Vector2 position, float & range_sq, size_t node,
  std::vector<std::pair<float, std::shared_ptr<Agent>>> & ret) const
{
  if (agent_tree_[node].end - agent_tree_[node].begin <= MAX_LEAF_SIZE) {
    for (size_t i = agent_tree_[node].begin; i < agent_tree_[node].end; ++i) {
      ret.emplace_back(std::make_pair(range_sq, agents_[i]));
    }
  } else {
    const float dist_sq_left =
      sqr(std::max(0.0f, agent_tree_[agent_tree_[node].left].min_x - position.x())) +
      sqr(std::max(0.0f, position.x() - agent_tree_[agent_tree_[node].left].max_x)) +
      sqr(std::max(0.0f, agent_tree_[agent_tree_[node].left].min_y - position.y())) +
      sqr(std::max(0.0f, position.y() - agent_tree_[agent_tree_[node].left].max_y));

    const float dist_sq_right =
      sqr(std::max(0.0f, agent_tree_[agent_tree_[node].right].min_x - position.x())) +
      sqr(std::max(0.0f, position.x() - agent_tree_[agent_tree_[node].right].max_x)) +
      sqr(std::max(0.0f, agent_tree_[agent_tree_[node].right].min_y - position.y())) +
      sqr(std::max(0.0f, position.y() - agent_tree_[agent_tree_[node].right].max_y));

    if (dist_sq_left < dist_sq_right) {
      if (dist_sq_left < range_sq) {
        queryAgentTreeRecursive(position, range_sq, agent_tree_[node].left, ret);

        if (dist_sq_right < range_sq) {
          queryAgentTreeRecursive(position, range_sq, agent_tree_[node].right, ret);
        }
      }
    } else {
      if (dist_sq_right < range_sq) {
        queryAgentTreeRecursive(position, range_sq, agent_tree_[node].right, ret);

        if (dist_sq_left < range_sq) {
          queryAgentTreeRecursive(position, range_sq, agent_tree_[node].left, ret);
        }
      }
    }
  }
}

ObstacleKdTree::ObstacleKdTree() : obstacle_tree_(NULL) {}

void ObstacleKdTree::buildObstacleTree(std::vector<Obstacle::SharedPtr> & obstacles)
{
  deleteObstacleTree();
  delete_flag_ = true;
  std::vector<Obstacle::SharedPtr> obstacle_root_node;
  std::copy(obstacles.begin(), obstacles.end(), std::back_inserter(obstacle_root_node));
  obstacle_tree_ = buildObstacleTreeRecursive(obstacle_root_node, obstacles);
}
ObstacleKdTree::ObstacleTreeNode * ObstacleKdTree::buildObstacleTreeRecursive(
  std::vector<Obstacle::SharedPtr> & obstacles_node,
  std::vector<Obstacle::SharedPtr> & obstacles_original)
{
  if (obstacles_node.empty()) {
    return NULL;
  } else {
    ObstacleTreeNode * const node = new ObstacleTreeNode;

    size_t optimal_split = 0;
    size_t min_left = obstacles_node.size();
    size_t min_right = obstacles_node.size();

    for (size_t i = 0; i < obstacles_node.size(); ++i) {
      size_t left_size = 0;
      size_t right_size = 0;

      Obstacle::SharedPtr obstacle_i1 = obstacles_node.at(i);
      auto obstacle_i2 = obstacles_node.at(i)->next_obstacle_;

      /* Compute optimal split node. */
      for (size_t j = 0; j < obstacles_node.size(); ++j) {
        if (i == j) {
          continue;
        }

        const auto obstacle_j1 = obstacles_node[j];
        const auto obstacle_j2 = obstacle_j1->next_obstacle_;

        const float j1_left_of_i =
          leftOf(obstacle_i1->point_, obstacle_i2->point_, obstacle_j1->point_);
        const float j2_left_of_i =
          leftOf(obstacle_i1->point_, obstacle_i2->point_, obstacle_j2->point_);

        if (j1_left_of_i >= -RVO_EPSILON && j2_left_of_i >= -RVO_EPSILON) {
          ++left_size;
        } else if (j1_left_of_i <= RVO_EPSILON && j2_left_of_i <= RVO_EPSILON) {
          ++right_size;
        } else {
          ++left_size;
          ++right_size;
        }

        if (
          std::make_pair(std::max(left_size, right_size), std::min(left_size, right_size)) >=
          std::make_pair(std::max(min_left, min_right), std::min(min_left, min_right))) {
          break;
        }
      }

      if (
        std::make_pair(std::max(left_size, right_size), std::min(left_size, right_size)) <
        std::make_pair(std::max(min_left, min_right), std::min(min_left, min_right))) {
        min_left = left_size;
        min_right = right_size;
        optimal_split = i;
      }
    }

    /* Build split node. */
    std::vector<Obstacle::SharedPtr> left_obstacles(min_left);
    std::vector<Obstacle::SharedPtr> right_obstacles(min_right);

    size_t left_counter = 0;
    size_t right_counter = 0;
    const size_t i = optimal_split;

    auto obstacle_i1 = obstacles_node[i];
    auto obstacle_i2 = obstacle_i1->next_obstacle_;

    for (size_t j = 0; j < obstacles_node.size(); ++j) {
      if (i == j) {
        continue;
      }

      const auto obstacle_j1 = obstacles_node[j];
      const auto obstacle_j2 = obstacle_j1->next_obstacle_;

      const float j1_left_of_i =
        leftOf(obstacle_i1->point_, obstacle_i2->point_, obstacle_j1->point_);
      const float j2_left_of_i =
        leftOf(obstacle_i1->point_, obstacle_i2->point_, obstacle_j2->point_);

      if (j1_left_of_i >= -RVO_EPSILON && j2_left_of_i >= -RVO_EPSILON) {
        left_obstacles[left_counter++] = obstacles_node[j];
      } else if (j1_left_of_i <= RVO_EPSILON && j2_left_of_i <= RVO_EPSILON) {
        right_obstacles[right_counter++] = obstacles_node[j];
      } else {
        /* Split obstacle j. */
        const float t =
          det(
            obstacle_i2->point_ - obstacle_i1->point_, obstacle_j1->point_ - obstacle_i1->point_) /
          det(obstacle_i2->point_ - obstacle_i1->point_, obstacle_j1->point_ - obstacle_j2->point_);

        const Vector2 split_point =
          obstacle_j1->point_ + t * (obstacle_j2->point_ - obstacle_j1->point_);

        auto new_obstacle = std::make_shared<Obstacle>();
        new_obstacle->point_ = split_point;
        new_obstacle->prev_obstacle_ = obstacle_j1;
        new_obstacle->next_obstacle_ = obstacle_j2;
        new_obstacle->is_convex_ = true;
        new_obstacle->unit_dir_ = obstacle_j1->unit_dir_;

        new_obstacle->id_ = obstacles_original.size();

        obstacles_original.push_back(new_obstacle);

        obstacle_j1->next_obstacle_ = new_obstacle;
        obstacle_j2->prev_obstacle_ = new_obstacle;

        if (j1_left_of_i > 0.0f) {
          left_obstacles[left_counter++] = obstacle_j1;
          right_obstacles[right_counter++] = new_obstacle;
        } else {
          right_obstacles[right_counter++] = obstacle_j1;
          left_obstacles[left_counter++] = new_obstacle;
        }
      }
    }

    node->obstacle = obstacle_i1;
    node->left = buildObstacleTreeRecursive(left_obstacles, obstacles_original);
    node->right = buildObstacleTreeRecursive(right_obstacles, obstacles_original);
    return node;
  }
}
std::vector<std::pair<float, Obstacle::SharedPtr>> ObstacleKdTree::computeObstacleNeighbors(
  RVO::Vector2 position, float & range) const
{
  float range_sq = sqr(range);
  std::vector<std::pair<float, Obstacle::SharedPtr>> ret;
  queryObstacleTreeRecursive(position, range_sq, obstacle_tree_, ret);
  return ret;
}
void ObstacleKdTree::deleteObstacleTree()
{
  if (delete_flag_) {
    deleteObstacleTree(obstacle_tree_);
    delete_flag_ = false;
  }
}
void ObstacleKdTree::deleteObstacleTree(ObstacleTreeNode * node)
{
  if (node != NULL) {
    deleteObstacleTree(node->left);
    deleteObstacleTree(node->right);
    delete node;
  }
}
void ObstacleKdTree::queryObstacleTreeRecursive(
  RVO::Vector2 position, float & range_sq, const ObstacleTreeNode * node,
  std::vector<std::pair<float, Obstacle::SharedPtr>> & ret) const
{
  if (node == NULL) {
    return;
  } else {
    const auto obstacle1 = node->obstacle;
    const auto obstacle2 = obstacle1->next_obstacle_;

    const float agent_left_of_line = leftOf(obstacle1->point_, obstacle2->point_, position);

    queryObstacleTreeRecursive(
      position, range_sq, (agent_left_of_line >= 0.0f ? node->left : node->right), ret);

    const float dist_sq_line =
      sqr(agent_left_of_line) / absSq(obstacle2->point_ - obstacle1->point_);

    if (dist_sq_line < range_sq) {
      if (agent_left_of_line < 0.0f) {
        /*
         * Try obstacle at this node only if agent is on right side of
         * obstacle (and can see obstacle).
         */
        ret.emplace_back(std::make_pair(range_sq, node->obstacle));
      }

      /* Try other side of line. */
      queryObstacleTreeRecursive(
        position, range_sq, (agent_left_of_line >= 0.0f ? node->right : node->left), ret);
    }
  }
}
bool ObstacleKdTree::queryVisibility(const Vector2 & q1, const Vector2 & q2, float radius) const
{
  return queryVisibilityRecursive(q1, q2, radius, obstacle_tree_);
}

bool ObstacleKdTree::queryVisibilityRecursive(
  const Vector2 & q1, const Vector2 & q2, float radius, const ObstacleTreeNode * node) const
{
  if (node == NULL) {
    return true;
  } else {
    const auto obstacle1 = node->obstacle;
    const auto obstacle2 = obstacle1->next_obstacle_;

    const float q1_left_of_i = leftOf(obstacle1->point_, obstacle2->point_, q1);
    const float q2_left_of_i = leftOf(obstacle1->point_, obstacle2->point_, q2);
    const float inv_length_i = 1.0f / absSq(obstacle2->point_ - obstacle1->point_);

    if (q1_left_of_i >= 0.0f && q2_left_of_i >= 0.0f) {
      return queryVisibilityRecursive(q1, q2, radius, node->left) &&
             ((sqr(q1_left_of_i) * inv_length_i >= sqr(radius) &&
               sqr(q2_left_of_i) * inv_length_i >= sqr(radius)) ||
              queryVisibilityRecursive(q1, q2, radius, node->right));
    } else if (q1_left_of_i <= 0.0f && q2_left_of_i <= 0.0f) {
      return queryVisibilityRecursive(q1, q2, radius, node->right) &&
             ((sqr(q1_left_of_i) * inv_length_i >= sqr(radius) &&
               sqr(q2_left_of_i) * inv_length_i >= sqr(radius)) ||
              queryVisibilityRecursive(q1, q2, radius, node->left));
    } else if (q1_left_of_i >= 0.0f && q2_left_of_i <= 0.0f) {
      /* One can see through obstacle from left to right. */
      return queryVisibilityRecursive(q1, q2, radius, node->left) &&
             queryVisibilityRecursive(q1, q2, radius, node->right);
    } else {
      const float point1_left_of_q = leftOf(q1, q2, obstacle1->point_);
      const float point2_left_of_q = leftOf(q1, q2, obstacle2->point_);
      const float inv_length_q = 1.0f / absSq(q2 - q1);

      return point1_left_of_q * point2_left_of_q >= 0.0f &&
             sqr(point1_left_of_q) * inv_length_q > sqr(radius) &&
             sqr(point2_left_of_q) * inv_length_q > sqr(radius) &&
             queryVisibilityRecursive(q1, q2, radius, node->left) &&
             queryVisibilityRecursive(q1, q2, radius, node->right);
    }
  }
}
}  // namespace RVO
