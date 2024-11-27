/*
 * KdTree.h
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

#ifndef CUSTOMIZED_RVO2__KDTREE_H_
#define CUSTOMIZED_RVO2__KDTREE_H_

/**
 * \file       KdTree.h
 * \brief      Contains the KdTree class.
 */

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Math.h"
#include "Obstacle.h"

namespace RVO
{
class Agent;

/**
 * \brief      Defines <i>k</i>d-trees for agents and static obstacles in the
 *             simulation.
 */

class AgentKdTree
{
public:
  class AgentTreeNode
  {
  public:
    size_t begin;
    size_t end;
    size_t left;
    float max_x;
    float max_y;
    float min_x;
    float min_y;
    size_t right;
  };
  AgentKdTree();

  /**
   * @brief This function clears the existing agent tree and rebuilds it based on the new list of agents.
   * @param all_agents A reference to an unordered map containing all the agents.
   */
  void rebuildAgentTree(std::unordered_map<std::string, std::shared_ptr<Agent>> & all_agents);
  //void updateAgentTree(std::unordered_map<std::string, std::shared_ptr<Agent>> & all_agents);

  /**
   * @brief builds the agent tree for the given range of agents.
   * @param begin The index of the first agent in the range.
   * @param end The index of the last agent in the range (exclusive).
   * @param node The index of the current node in the agent tree.
   */
  void buildAgentTreeRecursive(size_t begin, size_t end, size_t node);

  /**
   * @brief Computes the neighboring agents within a specified range of a given position.
   * @param position The position of the agent.
   * @param range The range within which to search for neighboring agents.
   * @return A vector of pairs, where each pair contains the distance to the agent and a shared pointer to the agent.
   */
  std::vector<std::pair<float, std::shared_ptr<Agent>>> computeAgentNeighbors(
    const RVO::Vector2 position, const float range) const;

  /**
   * @brief queries the agent tree to find agents within a specified range from a given position.
   * @param position The position from which to query.
   * @param range_sq The squared range within which to search for agents.
   * @param node The index of the current node in the agent tree.
   * @param ret A vector of pairs containing the squared range and shared pointers to the found agents.
   */
  void queryAgentTreeRecursive(
    RVO::Vector2 position, float & range_sq, size_t node,
    std::vector<std::pair<float, std::shared_ptr<Agent>>> & ret) const;

  std::vector<std::shared_ptr<Agent>> agents_;
  std::vector<AgentTreeNode> agent_tree_;

  static const size_t MAX_LEAF_SIZE = 10;

  friend class Agent;
};

class ObstacleKdTree
{
public:
  class ObstacleTreeNode
  {
  public:
    ObstacleTreeNode * left;
    std::shared_ptr<Obstacle> obstacle;
    ObstacleTreeNode * right;
  };

  ObstacleKdTree();

  /**
   * @brief Builds the obstacle tree using the given list of obstacles.
   * @param obstacles The list of obstacles to build the tree from.
   */
  void buildObstacleTree(std::vector<Obstacle::SharedPtr> & obstacles);

  /**
   * @brief Deletes the obstacle tree.
   * @note The obstacle tree must be created before calling this function.
   */
  void deleteObstacleTree();

  /**
   * @brief the obstacle neighbors within a specified range from a given position.
   * @param position The position from which to compute the obstacle neighbors.
   * @param range The range within which to search for obstacle neighbors.
   * @return A vector of pairs, where each pair contains the distance to an obstacle neighbor and a shared pointer to the obstacle.
   */
  std::vector<std::pair<float, Obstacle::SharedPtr>> computeObstacleNeighbors(
    RVO::Vector2 position, float & range) const;

private:
  ObstacleTreeNode * buildObstacleTreeRecursive(
    std::vector<Obstacle::SharedPtr> & obstacles_node,
    std::vector<Obstacle::SharedPtr> & obstacles_original);
  void queryObstacleTreeRecursive(
    RVO::Vector2 position, float & range_sq, const ObstacleTreeNode * node,
    std::vector<std::pair<float, Obstacle::SharedPtr>> & ret) const;

  void deleteObstacleTree(ObstacleTreeNode * node);

  bool queryVisibility(const Vector2 & q1, const Vector2 & q2, float radius) const;

  bool queryVisibilityRecursive(
    const Vector2 & q1, const Vector2 & q2, float radius, const ObstacleTreeNode * node) const;
  ObstacleTreeNode * obstacle_tree_;
  static const size_t MAX_LEAF_SIZE = 10;

  friend class Agent;
  bool delete_flag_ = false;
};

}  // namespace RVO

#endif  // CUSTOMIZED_RVO2__KDTREE_H_
