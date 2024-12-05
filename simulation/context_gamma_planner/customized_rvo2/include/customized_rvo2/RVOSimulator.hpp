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

#ifndef CUSTOMIZED_RVO2__RVOSIMULATOR_HPP_
#define CUSTOMIZED_RVO2__RVOSIMULATOR_HPP_

#include <memory>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <unordered_map>
#include <vector>

#include "customized_rvo2/Agent.hpp"
#include "customized_rvo2/Plugins/AgentPlugin.h"
#include "customized_rvo2/Plugins/ObstaclePlugin.h"

namespace RVO
{
/**
 *
 */
class RVOSimulator
{
public:
  explicit RVOSimulator(float time_step_s = 0.025f);
  void update();
  void update(Agent::SharedPtr agent);
  void setTimeStep(float time_step_s) { time_step_ = time_step_s; }

#define FORWARD_TO_AGENT(NAME) \
  decltype(auto) NAME(const std::string name) const { return getAgent(name)->NAME(); }
  FORWARD_TO_AGENT(getPosition);
  FORWARD_TO_AGENT(getId);
  FORWARD_TO_AGENT(getAgentConfig);
  FORWARD_TO_AGENT(getObstacleLines);

  void addOrcaLinePlugin(OrcaPluginBase::SharedPtr plugin) { orca_plugins_.emplace_back(plugin); }
  void addAgent(const std::shared_ptr<Agent> agent);
  const std::unordered_map<std::string, std::shared_ptr<Agent>> & getAgents() const
  {
    return agents_;
  }
  const std::shared_ptr<Agent> getAgent(const std::string & name) const;
  float getGlobalTime() const;

  /// @brief Add static obstacle to rvo simulator
  /// @param obstacle Static obstacle polygons
  /// @param process_obstacles Build a kdtree from static obstacles
  ///                          It is recommended to basically set it to false and run it
  ///                          only once after all obstacles have been added,
  ///                          as the process takes a long time.
  /// @return obstacle id
  size_t addGlobalObstacle(const std::vector<Vector2> & obstacle, bool process_obstacles = true);

  /// @brief Add static obstacles to rvo simulator
  /// @param obstacles Static obstacle polygons list
  /// @param process_obstacles Build a kdtree from static obstacles
  ///                          It is recommended to basically set it to false and run it
  ///                          only once after all obstacles have been added,
  ///                          as the process takes a long time.
  void addGlobalObstacles(
    const std::vector<std::vector<Vector2>> & obstacles, bool process_obstacles = false);
  bool agentExist(const std::string & name) const;
  void addObstacleORCAPlugin(OrcaObstaclePluginBase::SharedPtr plugin)
  {
    orca_obstacle_plugins_.emplace_back(plugin);
  }
  void addAgentORCAPlugin(OrcaAgentPluginBase::SharedPtr plugin)
  {
    orca_agent_plugins_.emplace_back(plugin);
  }

  /// @brief Build a kdtree from static obstacles
  void processObstacles() { obstacle_orca_plugin_->processObstacles(); }
  std::vector<Vector2> getObstacleLines() { return obstacle_orca_plugin_->getObstacleLines(); }
  void markUnavailableAllAgent();
  void deleteUnavailableAgent();
  void setAgentMaxSpeed(const std::string & name, double speed)
  {
    agents_[name]->setMaxSpeed(speed);
  }
  void deleteObstacles();

  AgentConfig createVehicleConfig(
    const traffic_simulator_msgs::msg::VehicleParameters & parameters) const;
  AgentConfig createPedestrianConfig(
    const traffic_simulator_msgs::msg::PedestrianParameters & parameters) const;

protected:
  void updateAgent(Agent::SharedPtr agent);
  std::vector<OrcaPluginBase::SharedPtr> orca_plugins_;
  std::shared_ptr<RVO::AgentKdTree> kdtree_;
  std::unordered_map<std::string, std::shared_ptr<Agent>> agents_;
  std::vector<OrcaAgentPluginBase::SharedPtr> orca_agent_plugins_;
  std::vector<OrcaObstaclePluginBase::SharedPtr> orca_obstacle_plugins_;
  std::shared_ptr<ObstaclePlugin> obstacle_orca_plugin_;
  std::shared_ptr<AgentPlugin> agent_plugin_;
  float time_step_;
  float global_time_;
};
}  // namespace RVO

#endif  // CUSTOMIZED_RVO2__RVOSIMULATOR_HPP_
