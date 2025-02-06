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

#ifndef CUSTOMIZED_RVO2__ORCA_PLUGIN_HPP_
#define CUSTOMIZED_RVO2__ORCA_PLUGIN_HPP_

#include <memory>

#include "customized_rvo2/Math.h"

namespace RVO
{
class Agent;
class AgentKdTree;
}  // namespace RVO

class OrcaPluginBase
{
public:
  using SharedPtr = std::shared_ptr<OrcaPluginBase>;
  OrcaPluginBase(std::string name) : name_(name) {}
  const std::string & getName() const { return name_; }
  enum class PluginType {
    AGENT,
    OBSTACLE,
  };
  virtual PluginType getType() = 0;
  void activate() { is_active_ = true; }
  void deactivate() { is_active_ = false; }
  bool isActive() const { return is_active_; }

protected:
  std::string name_;
  bool is_active_ = true;
};

class OrcaAgentPluginBase : public OrcaPluginBase
{
public:
  using SharedPtr = std::shared_ptr<OrcaAgentPluginBase>;
  OrcaAgentPluginBase(std::string name) : OrcaPluginBase(name) {}
  virtual std::vector<RVO::Line> calcOrcaLines(
    std::shared_ptr<RVO::Agent> agent, std::shared_ptr<RVO::AgentKdTree>, double time_step_s) = 0;
  PluginType getType() override { return PluginType::AGENT; }
};

class OrcaObstaclePluginBase : public OrcaPluginBase
{
public:
  using SharedPtr = std::shared_ptr<OrcaObstaclePluginBase>;
  OrcaObstaclePluginBase(std::string name) : OrcaPluginBase(name) {}
  virtual std::vector<RVO::Line> calcOrcaLines(std::shared_ptr<RVO::Agent> agent) = 0;
  PluginType getType() override { return PluginType::OBSTACLE; }
};
#endif  // CUSTOMIZED_RVO2__ORCA_PLUGIN_HPP_
