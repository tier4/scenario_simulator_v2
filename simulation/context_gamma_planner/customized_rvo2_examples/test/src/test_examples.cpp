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

#include <gtest/gtest.h>

#include <customized_rvo2/RVOSimulator.hpp>
//#include <scenario_simulator_exception/exception.hpp>
#include "customized_rvo2_examples/example_scenarios.hpp"
#include "customized_rvo2_examples/scenario.hpp"

class TestExamples : public ::testing::Test
{
protected:
  virtual void SetUp() { simulator_ = std::make_unique<RVO::RVOSimulator>(0.025f); }
  virtual void TearDown() { simulator_.release(); }

public:
  std::unique_ptr<RVO::RVOSimulator> simulator_;
};

TEST_F(TestExamples, agent_ignore)
{
  RVO::Scenario scenario;
  setupAgentIgnoreScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, block)
{
  RVO::Scenario scenario;
  setupBlockScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}
TEST_F(TestExamples, circle)
{
  RVO::Scenario scenario;
  setupCircleScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, curve)
{
  RVO::Scenario scenario;
  setupCurveScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, intersection)
{
  RVO::Scenario scenario;
  setupIntersectionScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, multiple_agent_avoid)
{
  RVO::Scenario scenario;
  setupMultipleAgentAvoidScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, single_agent_avoid)
{
  RVO::Scenario scenario;
  setupSingleAgentAvoidScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, road)
{
  RVO::Scenario scenario;
  setupRoadScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 10) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, critical_corner_case)
{
  RVO::Scenario scenario;
  setupCriticalCornerCaseScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 20) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, grid_obstacle)
{
  RVO::Scenario scenario;
  setupGridObstacleScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    while (simulator_->getGlobalTime() <= 20) {
      simulator_->update();
    }
  };
  EXPECT_NO_THROW(update());
}

TEST_F(TestExamples, dynamic_velocity)
{
  RVO::Scenario scenario;
  setupDynamicVelocityScenario(scenario);
  scenario.setupScenario(simulator_.get());
  auto update = [this]() {
    simulator_->getAgents().at("agent1")->setMaxSpeed(5.0);
    EXPECT_DOUBLE_EQ(simulator_->getAgents().at("agent1")->getMaxSpeed(), 5.0);

    while (simulator_->getGlobalTime() <= 20) {
      const float max_speed = simulator_->getGlobalTime();
      simulator_->getAgents().at("agent1")->setMaxSpeed(max_speed);
      simulator_->update();
      const auto current_speed = simulator_->getAgents().at("agent1")->getVelocity();
      EXPECT_LE(
        std::sqrt(current_speed.x() * current_speed.x() + current_speed.y() * current_speed.y()),
        max_speed + 1e-3);
    }
  };
  EXPECT_NO_THROW(update());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
