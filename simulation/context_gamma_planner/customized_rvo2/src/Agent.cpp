/*
 * Agent.cpp
 * copied and modified from AgentImpl.cpp in RVO2 Library
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

#include "customized_rvo2/Agent.hpp"

#include <limits>
#include <memory>
#include <vector>

#include "customized_rvo2/KdTree.h"
#include "customized_rvo2/Plugins/AgentPlugin.h"
#include "customized_rvo2/Plugins/ObstaclePlugin.h"

namespace RVO
{
const size_t & Agent::getId() const { return this->id_; }

void Agent::setId(const size_t & id) { id_ = id; }

Vector2 Agent::getPosition() const
{
  return pose_.position + Vector2(-config_.base_link_to_center, 0.0).rotate(getOrientationYaw());
}
void Agent::setPosition(const Vector2 & position)
{
  pose_.position = position + Vector2(config_.base_link_to_center, 0.0).rotate(getOrientationYaw());
}
const Vector2 & Agent::getCenterPosition() const { return pose_.position; }
const Vector2 & Agent::getVelocity() const { return velocity_; }
float Agent::getOrientationYaw() const
{
  Eigen::Quaterniond eigen_quat(
    pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  return eigen_quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];
}
const geometry_msgs::msg::Quaternion & Agent::getOrientation() const { return pose_.orientation; }
void Agent::setOrientation(const float & orientation)
{
  Eigen::Quaterniond eigen_quat(Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ()));
  pose_.orientation.x = eigen_quat.x();
  pose_.orientation.y = eigen_quat.y();
  pose_.orientation.z = eigen_quat.z();
  pose_.orientation.w = eigen_quat.w();
}
void Agent::setOrientation(const geometry_msgs::msg::Quaternion & quat)
{
  pose_.orientation = quat;
}

void Agent::setPose(const geometry_msgs::msg::Pose & pose)
{
  setPosition(RVO::Vector2(pose.position.x, pose.position.y));
  setOrientation(pose.orientation);
}
geometry_msgs::msg::Pose Agent::getPose() const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = getPosition().x();
  pose.position.y = getPosition().y();
  pose.orientation = getOrientation();
  return pose;
}

void Agent::setMaxSpeed(const float & max_speed) { config_.max_speed = max_speed; }
const float & Agent::getMaxSpeed() const { return config_.max_speed; }
void Agent::setMaxAcceleration(const float & max_acceleration)
{
  config_.max_acceleration = max_acceleration;
}
const float & Agent::getMaxAcceleration() const { return config_.max_acceleration; }
void Agent::setMaxDeceleration(const float & max_deceleration)
{
  config_.max_deceleration = max_deceleration;
}
const float & Agent::getMaxDeceleration() const { return config_.max_deceleration; }

const AgentConfig & Agent::getAgentConfig() const { return config_; }

/**
 * @brief Adds a list of waypoints to the agent's existing waypoints.
 *
 * @param wps The list of waypoints to be added.
 */
void Agent::addWaypoints(const std::vector<RVO::Vector2> & wps)
{
  waypoints_.insert(waypoints_.end(), wps.begin(), wps.end());
}

/**
 * @brief Updates the front waypoint of the agent.
 * 
 * If the agent's waypoints list is empty, the given waypoint is added to the list.
 * Otherwise, the first waypoint in the list is updated with the given waypoint.
 * 
 * @param wp The new front waypoint.
 */
void Agent::updateFrontWaypoint(const RVO::Vector2 & wp)
{
  if (waypoints_.empty()) {
    waypoints_.emplace_back(wp);
  } else {
    waypoints_.front() = wp;
  }
}

/**
 * @brief the new velocity for an agent based on the given agent lines, obstacle lines, and number of obstacle lines.
 *
 * @param agent_lines The vector of agent lines.
 * @param obstacle_lines The vector of obstacle lines.
 * @param numObstLines The number of obstacle lines.
 * @return The computed velocity as a Vector2 object.
 */
Vector2 Agent::computeNewVelocity(
  const std::vector<Line> & agent_lines, const std::vector<Line> & obstacle_lines,
  const int numObstLines) const
{
  Vector2 calc_velocity;
  std::vector<Line> lines;
  lines.reserve(agent_lines.size() + obstacle_lines.size());
  std::copy(agent_lines.begin(), agent_lines.end(), std::back_inserter(lines));
  std::copy(obstacle_lines.begin(), obstacle_lines.end(), std::back_inserter(lines));

  size_t lineFail = linearProgram2(lines, config_.max_speed, pref_velocity_, false, calc_velocity);

  if (lineFail < lines.size()) {
    linearProgram3(lines, lineFail, numObstLines, config_.max_speed, calc_velocity);
  }
  return calc_velocity;
}

/**
 * @brief Updates the agent's simple navigation based on the current waypoints.
 * 
 * This function calculates the goal vector by subtracting the agent's current position from the first waypoint.
 * If the squared magnitude of the goal vector is greater than 1.0, it is normalized.
 * If there are more than one waypoints, the first waypoint is removed from the list.
 * The preferred velocity is then calculated by multiplying the goal vector with the maximum speed.
 */
void Agent::updateSimpleNavigation()
{
  RVO::Vector2 goal_vector = waypoints_.front() - pose_.position;

  if (RVO::absSq(goal_vector) > 1.0f) {
    goal_vector = RVO::normalize(goal_vector);
  } else if (waypoints_.size() > 1) {
    waypoints_.pop_front();
  }
  //goal_vector = RVO::normalize(goal_vector);
  pref_velocity_ = goal_vector * config_.max_speed;
}

/**
 * @brief Constructor for the Agent class.
 * 
 * @param name The name of the agent.
 * @param pos The initial position of the agent.
 * @param config The configuration settings for the agent.
 * @param orientation The initial orientation of the agent.
 */
Agent::Agent(
  const std::string & name, const RVO::Vector2 & pos, const AgentConfig & config,
  const double orientation)
: name(name), config_(config)
{
  setPosition(pos);
  setVelocity(RVO::Vector2(0.f, 0.f));
  setOrientation(orientation);
  addWaypoint(this->pose_.position);
}

/**
 * @brief Constructor for the Agent class.
 * 
 * @param name The name of the agent.
 * @param pose The initial pose of the agent.
 * @param config The configuration for the agent.
 */
Agent::Agent(
  const std::string & name, const geometry_msgs::msg::Pose & pose, const AgentConfig & config)
: name(name), config_(config)
{
  setPosition(RVO::Vector2(pose.position.x, pose.position.y));
  setVelocity(RVO::Vector2(0.f, 0.f));
  setOrientation(pose.orientation);
  addWaypoint(this->pose_.position);
}

const Vector2 & Agent::getPrefVelocity() const { return this->pref_velocity_; }

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
 * @param stop_velocity_threshold The threshold for stopping velocity.
 */
void Agent::update(
  const std::vector<Line> & orca_lines_obst, const std::vector<Line> & orca_lines_agent,
  double time_step_s, double stop_velocity_threshold)
{
  updateSimpleNavigation();

  new_velocity_ = computeNewVelocity(orca_lines_agent, orca_lines_obst, orca_lines_obst.size());
  // update state
  if (RVO::absSq(new_velocity_) < stop_velocity_threshold) {
    velocity_ = new_velocity_;
  } else if (abs(new_velocity_) < abs(velocity_) - config_.max_deceleration * time_step_s) {
    velocity_ =
      (abs(velocity_) - config_.max_deceleration * time_step_s) * normalize(new_velocity_);
  } else if (abs(velocity_) + config_.max_acceleration * time_step_s < abs(new_velocity_)) {
    velocity_ =
      (abs(velocity_) + config_.max_acceleration * time_step_s) * normalize(new_velocity_);
  } else {
    velocity_ = new_velocity_;
  }
  pose_.position += velocity_ * time_step_s;
  if (RVO::absSq(velocity_) > stop_velocity_threshold) {
    setOrientation(std::atan2(velocity_.y(), velocity_.x()));
  }
}

/**
 * @brief Prints the status of the agent.
 * 
 * This function prints the name, type, position, yaw angle, and velocity of the agent.
 */
void Agent::printStatus()
{
  std::cout << "##### " << name << " #####" << std::endl;
  std::cout << "name : " << name << std::endl;
  std::cout << "type : " << config_.entity_type << std::endl;
  std::cout << "position : " << pose_.position.x() << ", " << pose_.position.y() << std::endl;
  std::cout << "yaw angle : " << getOrientationYaw() << "[rad] ("
            << getOrientationYaw() * 180.0 / M_PI << " [deg])" << std::endl;
  std::cout << "velocity : " << velocity_.x() << ", " << velocity_.y() << std::endl;
  std::cout << "##### " << name << " #####" << std::endl;
}

}  // namespace RVO
