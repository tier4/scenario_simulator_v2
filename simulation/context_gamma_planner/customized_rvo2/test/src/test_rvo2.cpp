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
#include <scenario_simulator_exception/exception.hpp>

class Rvo2Test : public ::testing::Test
{
protected:
  virtual void SetUp() { simulator_ = std::make_unique<RVO::RVOSimulator>(0.025f); }
  virtual void TearDown() { simulator_.release(); }

public:
  std::unique_ptr<RVO::RVOSimulator> simulator_;
};

bool checkVector2(const RVO::Vector2 & vec0, const RVO::Vector2 & vec1, float tolerance = 0.01)
{
  float diff = std::hypot(vec1.x() - vec0.x(), vec1.y() - vec0.y());
  if (diff <= tolerance) {
    return true;
  }
  std::cout << "vec0:" << vec0.x() << "," << vec0.y() << std::endl;
  std::cout << "vec1:" << vec1.x() << "," << vec1.y() << std::endl;
  return false;
}

TEST_F(Rvo2Test, UpdateEmpty) { simulator_->update(); }

TEST_F(Rvo2Test, addAgent)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  simulator_->addAgent(agent);
}

TEST_F(Rvo2Test, addAgentWithSameName)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  EXPECT_NO_THROW(simulator_->addAgent(agent));
  EXPECT_THROW(simulator_->addAgent(agent), common::SimulationError);
}

//TEST_F(Rvo2Test, getAgent)
//{
//  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
//  simulator_->addAgent(agent);
//  EXPECT_NO_THROW(simulator_->getAgent("agent"));
//  EXPECT_STREQ(simulator_->getAgent("agent")->name.c_str(), "agent");
//  EXPECT_THROW(simulator_->getAgent("expect_fail"), common::SimulationError);
//}

//TEST_F(Rvo2Test, getId)
//{
//  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
//  simulator_->addAgent(agent);
//  EXPECT_EQ(simulator_->getId("agent"), 0);
//  EXPECT_THROW(simulator_->getId("expect_fail"), common::SimulationError);
//}

TEST_F(Rvo2Test, getIdAgent)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  EXPECT_EQ(agent->getId(), 0);
}

TEST_F(Rvo2Test, setId)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  simulator_->addAgent(agent);
  EXPECT_EQ(simulator_->getId("agent"), 0);
  agent->setId(2);
  EXPECT_EQ(simulator_->getId("agent"), 2);
}

TEST_F(Rvo2Test, getPosition)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0));
  simulator_->addAgent(agent);
  EXPECT_FLOAT_EQ(simulator_->getPosition("agent").x(), 0.0);
  EXPECT_FLOAT_EQ(simulator_->getPosition("agent").y(), 1.0);
}

TEST_F(Rvo2Test, getOrientation)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0));
  simulator_->addAgent(agent);
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 1.0;
  orientation.w = 0.0;
  agent->setOrientation(orientation);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getOrientation().x, 0.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getOrientation().y, 0.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getOrientation().z, 1.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getOrientation().w, 0.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getOrientationYaw(), -M_PI);
}

TEST_F(Rvo2Test, getAcceleration)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0));
  simulator_->addAgent(agent);
  agent->setMaxAcceleration(10.0);
  agent->setMaxDeceleration(-10.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getMaxAcceleration(), 10.0);
  EXPECT_FLOAT_EQ(simulator_->getAgent("agent")->getMaxDeceleration(), -10.0);
}

TEST_F(Rvo2Test, getPositionAgent)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0));
  EXPECT_FLOAT_EQ(agent->getPosition().x(), 0.0);
  EXPECT_FLOAT_EQ(agent->getPosition().y(), 1.0);
}

//TEST(Rvo2Test, constructors){
//  RVO::AgentConfig config;
//  EXPECT_FLOAT_EQ(RVO::Agent("agent1", RVO::Vector2(0, 1.0)).getPosition().y(),1.0);
////  EXPECT_EQ(std::make_shared<RVO::Agent>("agent2", RVO::Vector2(0, 1.0),config)->getId(),0);
//}

TEST_F(Rvo2Test, getAgentConfig)
{
  RVO::AgentConfig config;
  config.max_speed = 1000.0;
  std::shared_ptr<RVO::Agent> agent =
    std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0), config);
  simulator_->addAgent(agent);
  EXPECT_FLOAT_EQ(simulator_->getAgentConfig("agent").max_speed, 1000.0);
}

TEST_F(Rvo2Test, getAgentConfigAgent)
{
  RVO::AgentConfig config;
  config.max_speed = 1000.0;
  std::shared_ptr<RVO::Agent> agent =
    std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 1.0), config);
  EXPECT_FLOAT_EQ(agent->getAgentConfig().max_speed, 1000.0);
}

TEST_F(Rvo2Test, addWaypoint)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  agent->addWaypoint(RVO::Vector2(1, 0));
  simulator_->addAgent(agent);
}

TEST_F(Rvo2Test, addWaypoints)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  agent->addWaypoints({RVO::Vector2(1, 0), RVO::Vector2(2, 0)});
  simulator_->addAgent(agent);
}

TEST_F(Rvo2Test, agentExists)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  EXPECT_FALSE(simulator_->agentExist("agent"));
  EXPECT_FALSE(simulator_->agentExist("expect_false"));
  simulator_->addAgent(agent);
  EXPECT_TRUE(simulator_->agentExist("agent"));
  EXPECT_FALSE(simulator_->agentExist("expect_false"));
}

TEST_F(Rvo2Test, addObstacle)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  std::vector<RVO::Vector2> wrong_obstacle = {RVO::Vector2(-1, -1)};
  EXPECT_THROW(simulator_->addGlobalObstacle(wrong_obstacle, false), common::SimulationError);
}

TEST_F(Rvo2Test, getObstacleLines)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  simulator_->addAgent(agent);
  simulator_->addGlobalObstacle(
    {RVO::Vector2(-1, -1), RVO::Vector2(-1, 1), RVO::Vector2(-1, 2)}, true);
  EXPECT_EQ(simulator_->getObstacleLines().size(), static_cast<size_t>(6));
}

TEST_F(Rvo2Test, goForward)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  agent->addWaypoint(RVO::Vector2(1, 0));
  simulator_->addAgent(agent);
  const auto position = simulator_->getAgents().at("agent")->getPosition();
  EXPECT_TRUE(checkVector2(position, RVO::Vector2(0, 0)));
  while (simulator_->getGlobalTime() <= 10) {
    simulator_->update();
  }
  const auto position_end = simulator_->getAgents().at("agent")->getPosition();
  EXPECT_TRUE(checkVector2(position_end, RVO::Vector2(1.0, 0)));
}

TEST_F(Rvo2Test, goForwardWithStraightLane)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  agent->addWaypoint(RVO::Vector2(1, 0));
  simulator_->addAgent(agent);
  simulator_->addGlobalObstacle({RVO::Vector2(-1, 1), RVO::Vector2(2, 1)});
  simulator_->addGlobalObstacle({RVO::Vector2(-1, 1), RVO::Vector2(2, 1)});
  const auto position = simulator_->getAgents().at("agent")->getPosition();
  EXPECT_TRUE(checkVector2(position, RVO::Vector2(0, 0)));
  while (simulator_->getGlobalTime() <= 10) {
    simulator_->update();
  }
  const auto position_end = simulator_->getAgents().at("agent")->getPosition();
  EXPECT_TRUE(checkVector2(position_end, RVO::Vector2(1.0, 0)));
}

TEST_F(Rvo2Test, passIntersection)
{
  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 20.0f;
  auto agent1 = std::make_shared<RVO::Agent>("agent1", RVO::Vector2(20.0f, 0.1f), conf);
  agent1->addWaypoint(RVO::Vector2(-20.0f, 0.0f));
  simulator_->addAgent(agent1);

  auto agent2 = std::make_shared<RVO::Agent>("agent2", RVO::Vector2(-20.0f, -0.1f), conf);
  agent2->addWaypoint(RVO::Vector2(20.0f, 0.0f));
  simulator_->addAgent(agent2);

  std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

  obstacle1.push_back(RVO::Vector2(4.0f, 6.0f));
  obstacle1.push_back(RVO::Vector2(6.0f, 4.0f));
  obstacle1.push_back(RVO::Vector2(20.0f, 4.0f));
  obstacle1.push_back(RVO::Vector2(20.0f, 20.0f));
  obstacle1.push_back(RVO::Vector2(4.0f, 20.0f));

  obstacle2.push_back(RVO::Vector2(4.0f, -20.0f));
  obstacle2.push_back(RVO::Vector2(20.0f, -20.0f));
  obstacle2.push_back(RVO::Vector2(20.0f, -4.0f));
  obstacle2.push_back(RVO::Vector2(6.0f, -4.0f));
  obstacle2.push_back(RVO::Vector2(4.0f, -6.0f));

  obstacle3.push_back(RVO::Vector2(-4.0f, -6.0f));
  obstacle3.push_back(RVO::Vector2(-6.0f, -4.0f));
  obstacle3.push_back(RVO::Vector2(-20.0f, -4.0f));
  obstacle3.push_back(RVO::Vector2(-20.0f, -20.0f));
  obstacle3.push_back(RVO::Vector2(-4.0f, -20.0f));

  obstacle4.push_back(RVO::Vector2(-4.0f, 20.0f));
  obstacle4.push_back(RVO::Vector2(-20.0f, 20.0f));
  obstacle4.push_back(RVO::Vector2(-20.0f, 4.0f));
  obstacle4.push_back(RVO::Vector2(-6.0f, 4.0f));
  obstacle4.push_back(RVO::Vector2(-4.0f, 6.0f));

  simulator_->addGlobalObstacle(obstacle1);
  simulator_->addGlobalObstacle(obstacle2);
  simulator_->addGlobalObstacle(obstacle3);
  simulator_->addGlobalObstacle(obstacle4);

  // distance : 40[m]
  // max speed : 2.0[m/s]
  // => expected : over 20[s]
  while (simulator_->getGlobalTime() <= 25) {
    simulator_->update();
  }

  EXPECT_TRUE(
    checkVector2(simulator_->getAgents().at("agent1")->getPosition(), RVO::Vector2(-20, 0)));
  EXPECT_TRUE(
    checkVector2(simulator_->getAgents().at("agent2")->getPosition(), RVO::Vector2(20, 0)));
}

TEST_F(Rvo2Test, checkAgentIgnoreFunction)
{
  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 20.0f;
  conf.entity_type = traffic_simulator_msgs::msg::EntityType::VEHICLE;

  auto vehicle_no_avoid =
    std::make_shared<RVO::Agent>("vehicle_no_avoid", RVO::Vector2(2.f, 1.5f), conf);
  vehicle_no_avoid->addWaypoint({22.0f, 1.5f});
  vehicle_no_avoid->addIgnoreList(traffic_simulator_msgs::msg::EntityType::PEDESTRIAN);
  simulator_->addAgent(vehicle_no_avoid);

  auto vehicle_pedestrian_avoid =
    std::make_shared<RVO::Agent>("vehicle_pedestrian_avoid", RVO::Vector2(-2.f, 1.5f), conf);
  vehicle_pedestrian_avoid->addWaypoint({18.0f, 1.5f});
  simulator_->addAgent(vehicle_pedestrian_avoid);

  conf.entity_type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  auto pedestrian = std::make_shared<RVO::Agent>("pedestrian", RVO::Vector2(10.f, 2.f), conf);
  pedestrian->addIgnoreList(traffic_simulator_msgs::msg::EntityType::VEHICLE);
  pedestrian->addWaypoint({10.f, 2.f});
  simulator_->addAgent(pedestrian);

  bool has_pass_through_pedestrian[2] = {false, false};
  while (simulator_->getGlobalTime() <= 25) {
    simulator_->update();
    // check each agent to pass through the pedestrian
    if (absSq(vehicle_no_avoid->getPosition() - pedestrian->getPosition()) < 1.0f) {
      has_pass_through_pedestrian[0] = true;
    }
    if (absSq(vehicle_pedestrian_avoid->getPosition() - pedestrian->getPosition()) < 1.0f) {
      has_pass_through_pedestrian[1] = true;
    }
  }
  // no avoidance
  EXPECT_TRUE(has_pass_through_pedestrian[0]);
  // with avoidance
  EXPECT_FALSE(has_pass_through_pedestrian[1]);
}

TEST_F(Rvo2Test, obstaclePluginGetType)
{
  ObstaclePlugin plugin;
  EXPECT_EQ(plugin.getType(), OrcaPluginBase::PluginType::OBSTACLE);
}

TEST_F(Rvo2Test, agentPluginGetType)
{
  AgentPlugin plugin;
  EXPECT_EQ(plugin.getType(), OrcaPluginBase::PluginType::AGENT);
}

TEST_F(Rvo2Test, critical_corner_case)
{
  float center_x = 0;
  float center_y = 0;
  float inside_radius = 100;
  float outside_radius = inside_radius + 10.0f;
  auto pos1 = RVO::Vector2(inside_radius + 4.0f, 0.0f);
  auto pos2 = RVO::Vector2(
    center_x + (inside_radius + 4.0f) * std::cos(M_PI_2),
    center_y + (inside_radius + 4.0f) * std::sin(M_PI_2));

  RVO::AgentConfig conf;
  //   If radius is set to 2.0, the agent cannot avoid the obstacle and stops.
  // If radius is set to 1.0 and smaller, and neighbor_dist is set to 6 or higher, the agent can go over the obstacle, but contact occurs.
  conf.radius = 1.0f;  // fail 2.0f
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 6.0f;  // fail 5.0f, success 6.0f 10.0f
  conf.time_horizon = 10.0f;
  conf.time_horizon_obst = 10.0f;
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1, conf);
  simulator_->addAgent(agent1);

  std::vector<RVO::Vector2> curve_outside, curve_inside, obstacle;
  for (float theta = 0; theta < M_PI_2; theta += 0.01f) {
    curve_inside.push_back(RVO::Vector2(
      center_x + inside_radius * std::cos(theta), center_y + inside_radius * std::sin(theta)));
    curve_outside.push_back(RVO::Vector2(
      center_x + outside_radius * std::cos(M_PI_2 - theta),
      center_y + outside_radius * std::sin(M_PI_2 - theta)));

    agent1->addWaypoint(RVO::Vector2(
      center_x + (inside_radius + 4.0f) * std::cos(theta),
      center_y + (inside_radius + 4.0f) * std::sin(theta)));
  }
  agent1->addWaypoint(pos1);

  curve_inside.push_back(RVO::Vector2(center_x, center_y));
  curve_outside.push_back(RVO::Vector2(center_x + outside_radius, center_y + outside_radius));

  obstacle.push_back(RVO::Vector2(
    center_x + inside_radius * std::cos(M_PI / 5) + 3.0f,
    center_y + inside_radius * std::sin(M_PI / 5) + 3.0f));
  obstacle.push_back(RVO::Vector2(
    center_x + inside_radius * std::cos(M_PI / 5), center_y + inside_radius * std::sin(M_PI / 5)));

  simulator_->addGlobalObstacle(curve_inside);
  simulator_->addGlobalObstacle(curve_outside);
  simulator_->addGlobalObstacle(obstacle);

  bool has_collided = false;
  bool has_reached_goal = false;
  auto obstacle_pos = RVO::Vector2(
    center_x + inside_radius * std::cos(M_PI / 5) + 1.5f,
    center_y + inside_radius * std::sin(M_PI / 5) + 1.5f);
  while (simulator_->getGlobalTime() <= 150) {
    simulator_->update();

    if (absSq(agent1->getPosition() - pos2) < 1.0f) {
      has_reached_goal = true;
    }

    if (absSq(agent1->getPosition() - obstacle_pos) < conf.radius) {
      has_collided = true;
    }
  }
  EXPECT_TRUE(has_reached_goal);
  EXPECT_FALSE(has_collided);
}
/*
TEST_F(Rvo2Test, change_agent_speed)
{
  std::shared_ptr<RVO::Agent> agent = std::make_shared<RVO::Agent>("agent", RVO::Vector2(0, 0));
  simulator_->addAgent(agent);
  auto plugin = std::make_shared<context_gamma_planner::PedestrianPlugin>();
  plugin->setTargetSpeed(15);
  simulator_->addAgentORCAPlugin(plugin);
  plugin->setTargetSpeed(15);
  EXPECT_DOUBLE_EQ(simulator_->getAgent("agent")->getAgentConfig().max_speed, 15);
}*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
