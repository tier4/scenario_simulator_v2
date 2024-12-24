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
#include "customized_rvo2/RVOSimulator.hpp"

#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace RVO
{
RVOSimulator::RVOSimulator(float time_step_s)
{
  setTimeStep(time_step_s);
  kdtree_ = std::make_shared<RVO::AgentKdTree>();

  obstacle_orca_plugin_ = std::make_shared<ObstaclePlugin>();
  addObstacleORCAPlugin(obstacle_orca_plugin_);

  agent_plugin_ = std::make_shared<AgentPlugin>();
  addAgentORCAPlugin(agent_plugin_);
}

void RVOSimulator::update()
{
  if (agents_.size() != 0) {
    kdtree_->rebuildAgentTree(agents_);
  }

  for (auto agent : agents_) {
    updateAgent(agent.second);
  }
  global_time_ += time_step_;
}

void RVOSimulator::update(Agent::SharedPtr agent)
{
  if (agents_.size() != 0) {
    kdtree_->rebuildAgentTree(agents_);
  }

  updateAgent(agent);
  global_time_ += time_step_;
}

void RVOSimulator::updateAgent(Agent::SharedPtr agent)
{
  agent->obst_orca_lines_.clear();
  for (auto & plugin : orca_obstacle_plugins_) {
    if (!plugin->isActive()) {
      continue;
    }
    auto lines = plugin->calcOrcaLines(agent);
    for (auto line : lines) {
      agent->obst_orca_lines_.emplace_back(line);
    }
  }
  agent->agent_orca_lines_.clear();
  for (auto & plugin : orca_agent_plugins_) {
    if (!plugin->isActive()) {
      continue;
    }
    auto lines = plugin->calcOrcaLines(agent, kdtree_, time_step_);
    for (auto line : lines) {
      agent->agent_orca_lines_.emplace_back(line);
    }
  }
  agent->update(agent->obst_orca_lines_, agent->agent_orca_lines_, time_step_);
}
const std::shared_ptr<Agent> RVOSimulator::getAgent(const std::string & name) const
{
  if (agents_.find(name) != agents_.end()) {
    return agents_.at(name);
  }
  std::cout << "cannot find an agent named " << name << std::endl;
  return nullptr;
  //  THROW_SIMULATION_ERROR("agent : ", name, " does not exist.");
}

void RVOSimulator::addAgent(const std::shared_ptr<Agent> agent)
{
  if (agents_.find(agent->name) != agents_.end()) {
    THROW_SIMULATION_ERROR("agent : ", agent->name, " already exist.");
  }
  agent->setId(agents_.size());
  agents_.emplace(agent->name, agent);
}

bool RVOSimulator::agentExist(const std::string & name) const
{
  return agents_.find(name) != agents_.end();
}

float RVOSimulator::getGlobalTime() const { return global_time_; }

size_t RVOSimulator::addGlobalObstacle(
  const std::vector<Vector2> & obstacle, bool process_obstacles)
{
  if (obstacle.size() < 2) {
    THROW_SIMULATION_ERROR("obstacle should be consisted of at least three points");
    if (obstacle.size() < 2) {
      return RVO::RVO_ERROR;
    }
  }
  return obstacle_orca_plugin_->addObstacle(obstacle, process_obstacles);
}

void RVOSimulator::addGlobalObstacles(
  const std::vector<std::vector<Vector2> > & obstacles, bool process_obstacles)
{
  for (const auto & obstacle : obstacles) {
    addGlobalObstacle(obstacle, process_obstacles);
  }
}

void RVOSimulator::deleteObstacles() { obstacle_orca_plugin_->deleteObstacles(); }

void RVOSimulator::markUnavailableAllAgent()
{
  for (auto agent : agents_) {
    agent.second->is_available_ = false;
  }
}

void RVOSimulator::deleteUnavailableAgent()
{
  /*
  agents_.erase(
    std::remove_if(
      agents_.begin(), agents_.end(), [](const auto & agent) { return !agent.second->is_available_; }),
    agents_.end());
  */
  for (auto it = agents_.begin(); it != agents_.end();) {
    if (!it->second->is_available_) {
      it = agents_.erase(it);
    } else {
      ++it;
    }
  }
}

AgentConfig RVOSimulator::createVehicleConfig(
  const traffic_simulator_msgs::msg::VehicleParameters & parameters) const
{
  RVO::AgentConfig agent_config;
  agent_config.radius = parameters.bounding_box.dimensions.x / 2.0;
  agent_config.ellipticity =
    parameters.bounding_box.dimensions.y / parameters.bounding_box.dimensions.x;
  agent_config.base_link_to_center = parameters.bounding_box.center.x;
  agent_config.entity_type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  agent_config.max_speed = parameters.performance.max_speed;
  agent_config.max_acceleration = parameters.performance.max_acceleration;
  agent_config.max_deceleration = parameters.performance.max_deceleration;
  // Parameters are set to large values to allow the car to maintain a distance while driving.
  // Reduced value to consider only obstacles in the vicinity.
  agent_config.neighbor_dist = 10.0f;
  agent_config.time_horizon = 10.0f;
  agent_config.time_horizon_obst = 1.0f;
  return agent_config;
}

AgentConfig RVOSimulator::createPedestrianConfig(
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters) const
{
  RVO::AgentConfig agent_config;
  agent_config.radius = parameters.bounding_box.dimensions.x / 2.0;
  agent_config.ellipticity =
    parameters.bounding_box.dimensions.y / parameters.bounding_box.dimensions.x;
  agent_config.base_link_to_center = parameters.bounding_box.center.x;
  agent_config.entity_type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  agent_config.max_speed = 1.0f;
  agent_config.neighbor_dist = 4.0f;
  // The original parameters were too large, and even irrelevant places such as adjacent lanes
  // were included in the constraints, so we made them smaller.
  agent_config.time_horizon = 5.0f;
  agent_config.time_horizon_obst = 2.0f;
  return agent_config;
}

}  // namespace RVO
