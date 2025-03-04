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

#ifndef CUSTOMIZED_RVO2__AGENT_HPP_
#define CUSTOMIZED_RVO2__AGENT_HPP_
#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <utility>
#include <vector>

#include "customized_rvo2/KdTree.h"
#include "customized_rvo2/Plugins/OrcaPlugin.h"
#include "geometry_msgs/msg/pose.hpp"

namespace RVO
{
struct AgentConfig
{
  /// @brief Type of entity (VEHICLE or PEDESTRIAN)
  uint8_t entity_type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  /// @brief long radius
  float radius = 2.0f;
  /// @brief  ellipticity = short radius / long radius (0 < ellipticity <= 1.0)
  ///         short radius = ellipticity * long radius
  float ellipticity = 1.0f;
  /// @brief  vehicle base_link to circle center
  float base_link_to_center = 0.0f;
  /// @brief  Agent's maximum speed [m/s]
  float max_speed = 2.0f;
  /// @brief  Agent's maximum acceleration [m/s^2]
  float max_acceleration = 100.0f;
  /// @brief  Agent's maximum deceleration [m/s^2]
  float max_deceleration = 100.0f;
  /// @brief  Distance of obstacles to be considered in the calculation
  float neighbor_dist = 15.0f;
  /// @brief  Number of obstacles considered in the calculation
  size_t max_neighbors = 10;
  /// @brief  Time for dynamic obstacles to be included in the prediction calculation
  float time_horizon = 5.0f;
  /// @brief  Time for static obstacles to be included in the prediction calculation
  float time_horizon_obst = 5.0f;
};

struct AgentPose
{
  Vector2 position;
  geometry_msgs::msg::Quaternion orientation;
};

class RVOSimulator;
class Agent
{
public:
  /**
   * @brief Constructor for the Agent class.
   * 
   * @param name The name of the agent.
   * @param pos The initial position of the agent.
   * @param config The configuration settings for the agent.
   * @param orientation The initial orientation of the agent.
   */
  explicit Agent(
    const std::string & name, const RVO::Vector2 & pos, const AgentConfig & config = AgentConfig(),
    double orientation = 0.0);

  /**
   * @brief Constructor for the Agent class.
   * 
   * @param name The name of the agent.
   * @param pose The initial pose of the agent.
   * @param config The configuration for the agent.
   */
  explicit Agent(
    const std::string & name, const geometry_msgs::msg::Pose & pose,
    const AgentConfig & config = AgentConfig());

  const std::string name;
  using SharedPtr = std::shared_ptr<Agent>;

  /**
   * @brief Adds a list of waypoints to the agent's existing waypoints.
   *
   * @param wps The list of waypoints to be added.
   */
  void addWaypoint(const RVO::Vector2 & wp) { waypoints_.emplace_back(wp); }

  /**
   * @brief Updates the front waypoint of the agent.
   * 
   * If the agent's waypoints list is empty, the given waypoint is added to the list.
   * Otherwise, the first waypoint in the list is updated with the given waypoint.
   * 
   * @param wp The new front waypoint.
   */
  void updateFrontWaypoint(const RVO::Vector2 & wp);
  void addWaypoints(const std::vector<RVO::Vector2> & wps);

  /**
   * @brief Updates the agent's simple navigation based on the current waypoints.
   * 
   * This function calculates the goal vector by subtracting the agent's current position from the first waypoint.
   * If the squared magnitude of the goal vector is greater than 1.0, it is normalized.
   * If there are more than one waypoints, the first waypoint is removed from the list.
   * The preferred velocity is then calculated by multiplying the goal vector with the maximum speed.
   */
  void updateSimpleNavigation();

  /**
   * @brief Updates the agent's state based on the given ORCA lines and time step.
   * 
   * This function calculates the preferred velocity for the agent using the updateSimpleNavigation() function.
   * It then calculates the ORCA lines for obstacles and other agents using the provided vectors.
   * The ORCA lines are used to compute the new velocity for the agent.
   * The agent's state is updated based on the new velocity and the given time step.
   * 
   * @param orca_lines_obst The ORCA lines for obstacles.
   * @param orca_lines_agent The ORCA lines for other agents.
   * @param time_step_s The time step in seconds.
   * @param stop_velocity_threshold [m/s] The threshold for stopping velocity.
   *                                If the velocity is less than this value, the velocity is set to 0.
   *                                The default value is small enough to prevent chattering.
   */
  void update(
    const std::vector<Line> & orca_lines_obst, const std::vector<Line> & orca_lines_agent,
    double time_step_s, double stop_velocity_threshold = 0.1);

  /**
   * @brief the new velocity for an agent based on the given agent lines, obstacle lines, and number of obstacle lines.
   *
   * @param agent_lines The vector of agent lines.
   * @param obstacle_lines The vector of obstacle lines.
   * @param numObstLines The number of obstacle lines.
   * @return The computed velocity as a Vector2 object.
   */
  Vector2 computeNewVelocity(
    const std::vector<Line> & agent_lines, const std::vector<Line> & obstacle_lines,
    int numObstLines) const;

  // getter setter
  const Vector2 & getPrefVelocity() const;
  const size_t & getId() const;
  void setId(const size_t & id);
  /// @brief  Returns the base_link position of the agent.
  Vector2 getPosition() const;
  /// @brief set the base_link position of the agent.
  void setPosition(const Vector2 & position);
  const Vector2 & getCenterPosition() const;
  const Vector2 & getVelocity() const;
  void setVelocity(const Vector2 & velocity) { velocity_ = velocity; }
  float getOrientationYaw() const;
  const geometry_msgs::msg::Quaternion & getOrientation() const;
  void setOrientation(const float & orientation);
  void setOrientation(const geometry_msgs::msg::Quaternion & quat);
  void setPose(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose getPose() const;
  void setMaxSpeed(const float & max_speed);
  const float & getMaxSpeed() const;
  void setMaxAcceleration(const float & max_acceleration);
  const float & getMaxAcceleration() const;
  void setMaxDeceleration(const float & max_deceleration);
  const float & getMaxDeceleration() const;
  const AgentConfig & getAgentConfig() const;
  std::vector<Vector2> getObstacleLines();
  void addIgnoreList(uint8_t entity_type) { ignore_list_.emplace_back(entity_type); };
  const auto & getIgnoreList() { return ignore_list_; }

  /// @brief A vector of RVO::Line objects representing obstacle ORCA lines.(static obstacles)
  std::vector<RVO::Line> obst_orca_lines_;

  /// @brief A vector of RVO::Line objects representing agent ORCA lines.(dynamic obstacles)
  std::vector<RVO::Line> agent_orca_lines_;
  bool is_available_ = true;

  /**
   * @brief Prints the status of the agent.
   * 
   * This function prints the name, type, position, yaw angle, and velocity of the agent.
   */
  void printStatus();

private:
  std::deque<RVO::Vector2> waypoints_;

  AgentConfig config_;
  AgentPose pose_;
  Vector2 new_velocity_;
  Vector2 pref_velocity_;
  Vector2 velocity_;
  size_t id_;
  std::vector<uint8_t> ignore_list_;
};
}  // namespace RVO
#endif  // CUSTOMIZED_RVO2__AGENT_HPP_
