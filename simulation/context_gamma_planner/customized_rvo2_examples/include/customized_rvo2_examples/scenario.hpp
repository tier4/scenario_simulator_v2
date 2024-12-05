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

#ifndef CUSTOMIZED_RVO2_EXAMPLES__SCENARIO_HPP_
#define CUSTOMIZED_RVO2_EXAMPLES__SCENARIO_HPP_

#include <memory>
#include <vector>

#include "customized_rvo2/Agent.hpp"
#include "customized_rvo2/RVOSimulator.hpp"

namespace RVO
{
/**
 *
 */
struct Scenario
{
public:
  /**
   *
   * @param obstacle
   */
  void addGlobalObstacle(std::vector<Vector2> obstacle)
  {
    auto obstacle_ptr = std::make_shared<std::vector<Vector2>>(obstacle);
    addGlobalObstacle(obstacle_ptr);
  }
  /**
   *
   * @param obstacle
   */
  void addGlobalObstacle(std::shared_ptr<std::vector<Vector2>> obstacle)
  {
    obstacles_.emplace_back(obstacle);
  }
  /**
   *
   * @param agent
   */
  void addAgent(std::shared_ptr<Agent> agent) { agents_.emplace_back(agent); }
  /**
   *
   * @param sim
   */
  void setupScenario(RVOSimulator * sim)
  {
    for (auto obstacle : obstacles_) {
      sim->addGlobalObstacle(*obstacle, false);
    }
    sim->processObstacles();

    for (auto & agent : agents_) {
      sim->addAgent(agent);
    }
  }

  void setupScenario(std::shared_ptr<RVOSimulator> sim)
  {
    for (auto obstacle : obstacles_) {
      sim->addGlobalObstacle(*obstacle, false);
    }
    sim->processObstacles();

    for (auto & agent : agents_) {
      sim->addAgent(agent);
    }
  }

  /**
   *
   * @param sim
   */
  void setupScenario(RVOSimulator & sim) { setupScenario(&sim); }

private:
  std::vector<std::shared_ptr<std::vector<Vector2>>> obstacles_;
  std::vector<std::shared_ptr<Agent>> agents_;
};
}  // namespace RVO
#endif  // CUSTOMIZED_RVO2_EXAMPLES__SCENARIO_HPP_
