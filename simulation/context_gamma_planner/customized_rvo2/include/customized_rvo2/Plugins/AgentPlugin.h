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

#ifndef CUSTOMIZED_RVO2__PLUGINS__AGENT_PLUGIN_H
#define CUSTOMIZED_RVO2__PLUGINS__AGENT_PLUGIN_H

#include "OrcaPlugin.h"

class AgentPlugin : public OrcaAgentPluginBase
{
public:
  AgentPlugin() : OrcaAgentPluginBase("AgentPlugin") {}
  std::vector<RVO::Line> calcOrcaLines(
    std::shared_ptr<RVO::Agent> agent, std::shared_ptr<RVO::AgentKdTree> agent_kdtree,
    double time_step_s) override;
  using AgentNeighborList = std::vector<std::pair<float, std::shared_ptr<RVO::Agent>>>;
  void filterAgentNeighbors(std::shared_ptr<RVO::Agent> agent, AgentNeighborList & neighbor_list);
};

#endif  // CUSTOMIZED_RVO2__PLUGINS__AGENT_PLUGIN_H
