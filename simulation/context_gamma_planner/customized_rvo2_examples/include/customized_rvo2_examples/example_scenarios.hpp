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

#ifndef CUSTOMIZED_RVO2_EXAMPLES__EXAMPLE_SCENARIO_HPP_
#define CUSTOMIZED_RVO2_EXAMPLES__EXAMPLE_SCENARIO_HPP_

#include "customized_rvo2_examples/scenario.hpp"

void setupAgentIgnoreScenario(RVO::Scenario & scenario)
{
  std::vector<RVO::Vector2> road_upper_bound, road_lower_bound;
  road_upper_bound.push_back(RVO::Vector2(0.0f, 3.0f));
  road_upper_bound.push_back(RVO::Vector2(20.0f, 3.0f));
  road_upper_bound.push_back(RVO::Vector2(20.0f, 3.1f));
  road_upper_bound.push_back(RVO::Vector2(0.0f, 3.1f));

  road_lower_bound.push_back(RVO::Vector2(0.0f, -3.1f));
  road_lower_bound.push_back(RVO::Vector2(20.0f, -3.1f));
  road_lower_bound.push_back(RVO::Vector2(20.0f, -3.0f));
  road_lower_bound.push_back(RVO::Vector2(0.0f, -3.0f));

  scenario.addGlobalObstacle(road_upper_bound);
  scenario.addGlobalObstacle(road_lower_bound);

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
  scenario.addAgent(vehicle_no_avoid);

  auto vehicle_pedestrian_avoid =
    std::make_shared<RVO::Agent>("vehicle_pedestrian_avoid", RVO::Vector2(-2.f, 1.5f), conf);
  vehicle_pedestrian_avoid->addWaypoint({18.0f, 1.5f});
  scenario.addAgent(vehicle_pedestrian_avoid);

  conf.entity_type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  auto pedestrian = std::make_shared<RVO::Agent>("pedestrian", RVO::Vector2(10.f, 2.f), conf);
  pedestrian->addIgnoreList(traffic_simulator_msgs::msg::EntityType::VEHICLE);
  pedestrian->addWaypoint({10.f, 2.f});
  scenario.addAgent(pedestrian);
}

void setupBlockScenario(RVO::Scenario & scenario)
{
  for (size_t i = 0; i < 5; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      auto agent1 = std::make_shared<RVO::Agent>(
        "agent1_" + std::to_string(i * 5 + j), RVO::Vector2(55.0f + i * 10.0f, 55.0f + j * 10.0f));
      agent1->addWaypoint(RVO::Vector2(-75.0f, -75.0f));
      scenario.addAgent(agent1);

      auto agent2 = std::make_shared<RVO::Agent>(
        "agent2_" + std::to_string(i * 5 + j), RVO::Vector2(-55.0f - i * 10.0f, 55.0f + j * 10.0f));
      agent2->addWaypoint(RVO::Vector2(75.0f, -75.0f));
      scenario.addAgent(agent2);

      auto agent3 = std::make_shared<RVO::Agent>(
        "agent3_" + std::to_string(i * 5 + j), RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
      agent3->addWaypoint(RVO::Vector2(-75.0f, 75.0f));
      scenario.addAgent(agent3);

      auto agent4 = std::make_shared<RVO::Agent>(
        "agent4_" + std::to_string(i * 5 + j),
        RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
      agent4->addWaypoint(RVO::Vector2(75.0f, 75.0f));
      scenario.addAgent(agent4);
    }
  }

  /*
     * Add (polygonal) obstacles, specifying their vertices in counterclockwise
     * order.
     */
  std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

  obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
  obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
  obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
  obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

  obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
  obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
  obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
  obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

  obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
  obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
  obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
  obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

  obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
  obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
  obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
  obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

  scenario.addGlobalObstacle(obstacle1);
  scenario.addGlobalObstacle(obstacle2);
  scenario.addGlobalObstacle(obstacle3);
  scenario.addGlobalObstacle(obstacle4);
}

void setupCircleScenario(RVO::Scenario & scenario)
{
  for (float r = 50.0f; r < 200.f; r += 20.f) {
    int n = static_cast<int>(r / 5);
    for (size_t i = 0; i < static_cast<size_t>(n); ++i) {
      auto agent = std::make_shared<RVO::Agent>(
        "agent_" + std::to_string(i) + "_" + std::to_string(r),
        (r + static_cast<float>(i) / n * 10.f) *
          RVO::Vector2(std::cos(i * 2.0f * M_PI / n), std::sin(i * 2.0f * M_PI / n)));
      agent->addWaypoint(-agent->getPosition());
      scenario.addAgent(agent);
    }
  }
}

void setupCurveScenario(RVO::Scenario & scenario)
{
  float center_x = 0;
  float center_y = 0;
  float inside_radius = 100;
  float outside_radius = inside_radius + 10.0f;

  /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */

  auto pos1 = RVO::Vector2(0.0f, inside_radius + 5.0f);
  auto pos2 = RVO::Vector2(inside_radius + 5.0f, 0.0f);
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1);
  agent1->addWaypoint(pos2);
  scenario.addAgent(agent1);

  auto agent2 = std::make_shared<RVO::Agent>("agent2", pos2);
  agent2->addWaypoint(pos1);
  scenario.addAgent(agent2);

  /*
     * Add (polygonal) obstacles, specifying their vertices in counterclockwise
     * order.
     */
  std::vector<RVO::Vector2> curve_outside, curve_inside;

  for (float theta = 0; theta < M_PI_2; theta += 0.01f) {
    curve_inside.push_back(RVO::Vector2(
      center_x + inside_radius * std::cos(theta), center_y + inside_radius * std::sin(theta)));
    curve_outside.push_back(RVO::Vector2(
      center_x + outside_radius * std::cos(M_PI_2 - theta),
      center_y + outside_radius * std::sin(M_PI_2 - theta)));
    agent2->addWaypoint(RVO::Vector2(
      center_x + (outside_radius - 4.0f) * std::cos(M_PI_2 - theta),
      center_y + (outside_radius - 4.0f) * std::sin(M_PI_2 - theta)));
    agent1->addWaypoint(RVO::Vector2(
      center_x + (inside_radius + 4.0f) * std::cos(theta),
      center_y + (inside_radius + 4.0f) * std::sin(theta)));
  }

  curve_inside.push_back(RVO::Vector2(center_x, center_y));

  curve_outside.push_back(RVO::Vector2(center_x + outside_radius, center_y + outside_radius));

  scenario.addGlobalObstacle(curve_inside);
  scenario.addGlobalObstacle(curve_outside);
}

void setupIntersectionScenario(RVO::Scenario & scenario)
{
  /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */

  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 20.0f;
  auto agent1 = std::make_shared<RVO::Agent>("agent1", RVO::Vector2(20.0f, 0.1f), conf);
  agent1->addWaypoint(RVO::Vector2(-20.0f, 0.0f));
  scenario.addAgent(agent1);

  auto agent2 = std::make_shared<RVO::Agent>("agent2", RVO::Vector2(-20.0f, -0.1f), conf);
  agent2->addWaypoint(RVO::Vector2(20.0f, 0.0f));
  scenario.addAgent(agent2);

  // 道路形状（交差点）
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

  scenario.addGlobalObstacle(obstacle1);
  scenario.addGlobalObstacle(obstacle2);
  scenario.addGlobalObstacle(obstacle3);
  scenario.addGlobalObstacle(obstacle4);
}

void setupCriticalCornerCaseScenario(RVO::Scenario & scenario)
{
  float center_x = 0;
  float center_y = 0;
  float inside_radius = 100;
  float outside_radius = inside_radius + 10.0f;
  auto pos1 = RVO::Vector2(inside_radius + 4.0f, 0.0f);

  RVO::AgentConfig conf;
  //   If radius is set to 2.0, the agent cannot avoid the obstacle and stops.
  // If radius is set to 1.0 and smaller, and neighbor_dist is set to 6 or higher, the agent can go over the obstacle, but contact occurs.
  conf.radius = 2.0f;  // fail 2.0f
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 5.0f;  // fail 5.0f, success 6.0f 10.0f
  conf.time_horizon = 10.0f;
  conf.time_horizon_obst = 10.0f;
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1, conf);
  scenario.addAgent(agent1);

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

  // obstacle.push_back(RVO::Vector2(
  //   center_x + inside_radius * std::cos(M_PI / 4), center_y + inside_radius * std::sin(M_PI / 4)));
  // obstacle.push_back(RVO::Vector2(
  //   center_x + inside_radius * std::cos(M_PI / 4) + 3.0f,
  //   center_y + inside_radius * std::sin(M_PI / 4) + 3.0f));
  obstacle.push_back(RVO::Vector2(
    center_x + inside_radius * std::cos(M_PI / 5) + 3.0f,
    center_y + inside_radius * std::sin(M_PI / 5) + 3.0f));
  obstacle.push_back(RVO::Vector2(
    center_x + inside_radius * std::cos(M_PI / 5), center_y + inside_radius * std::sin(M_PI / 5)));
  // obstacle.push_back(RVO::Vector2(
  //   center_x + inside_radius * std::cos(M_PI / 4), center_y + inside_radius * std::sin(M_PI / 4)));

  scenario.addGlobalObstacle(curve_inside);
  scenario.addGlobalObstacle(curve_outside);
  scenario.addGlobalObstacle(obstacle);
}

void setupRoadScenario(RVO::Scenario & scenario)
{
  std::vector<RVO::Vector2> curve_outside, curve_inside;
  RVO::Vector2 pos1(0.f, 105.f);
  RVO::Vector2 pos2(105.f, 0.f);
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1);
  auto agent2 = std::make_shared<RVO::Agent>("agent2", pos2);

  float center_x = 0;
  float center_y = 0;
  float inside_radius = 100;
  float outside_radius = inside_radius + 10.0f;
  float polygon_width = 0.5f;

  for (float theta = 0; theta < M_PI_2; theta += 0.01f) {
    curve_inside.push_back(RVO::Vector2(
      center_x + inside_radius * std::cos(theta), center_y + inside_radius * std::sin(theta)));
    curve_outside.push_back(RVO::Vector2(
      center_x + outside_radius * std::cos(M_PI_2 - theta),
      center_y + outside_radius * std::sin(M_PI_2 - theta)));
    agent1->addWaypoint(RVO::Vector2(
      center_x + (outside_radius - 3.0f) * std::cos(M_PI_2 - theta),
      center_y + (outside_radius - 3.0f) * std::sin(M_PI_2 - theta)));
    agent2->addWaypoint(RVO::Vector2(
      center_x + (inside_radius + 3.0f) * std::cos(theta),
      center_y + (inside_radius + 3.0f) * std::sin(theta)));
  }

  scenario.addAgent(agent1);
  scenario.addAgent(agent2);

  for (float theta = M_PI_2; theta > 0.f; theta -= 0.01f) {
    curve_inside.push_back(RVO::Vector2(
      center_x + (inside_radius - polygon_width) * std::cos(theta),
      center_y + (inside_radius - polygon_width) * std::sin(theta)));
    curve_outside.push_back(RVO::Vector2(
      center_x + (outside_radius + polygon_width) * std::cos(M_PI_2 - theta),
      center_y + (outside_radius + polygon_width) * std::sin(M_PI_2 - theta)));
  }

  scenario.addGlobalObstacle(curve_inside);
  scenario.addGlobalObstacle(curve_outside);
}

void setupMultipleAgentAvoidScenario(RVO::Scenario & scenario)
{
  std::vector<RVO::Vector2> obstacle;
  obstacle.push_back(RVO::Vector2(15.0f, 2.75f));
  obstacle.push_back(RVO::Vector2(11.0f, 2.75f));
  obstacle.push_back(RVO::Vector2(11.1f, 1.00f));
  obstacle.push_back(RVO::Vector2(15.0f, 1.00f));

  scenario.addGlobalObstacle(obstacle);

  std::vector<RVO::Vector2> road_upper_bound, road_lower_bound;
  road_upper_bound.push_back(RVO::Vector2(0.0f, 3.0f));
  road_upper_bound.push_back(RVO::Vector2(20.0f, 3.0f));
  road_upper_bound.push_back(RVO::Vector2(20.0f, 3.1f));
  road_upper_bound.push_back(RVO::Vector2(0.0f, 3.1f));

  road_lower_bound.push_back(RVO::Vector2(0.0f, -3.1f));
  road_lower_bound.push_back(RVO::Vector2(20.0f, -3.1f));
  road_lower_bound.push_back(RVO::Vector2(20.0f, -3.0f));
  road_lower_bound.push_back(RVO::Vector2(0.0f, -3.0f));

  scenario.addGlobalObstacle(road_upper_bound);
  scenario.addGlobalObstacle(road_lower_bound);

  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 20.0f;

  RVO::Vector2 pos1(0.f, 1.5f);
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1, conf);
  agent1->addWaypoint({20.0f, 1.5f});
  scenario.addAgent(agent1);

  RVO::Vector2 pos2(20.f, -1.5f);
  auto agent2 = std::make_shared<RVO::Agent>("agent2", pos2, conf);
  agent2->addWaypoint({0.0f, -1.5f});
  scenario.addAgent(agent2);
}

void setupSingleAgentAvoidScenario(RVO::Scenario & scenario)
{
  RVO::Vector2 pos(0.f, 0.f);
  auto agent = std::make_shared<RVO::Agent>("agent", pos);

  agent->addWaypoint({20.0f, 0.0f});
  agent->addWaypoint({0.0f, 0.0f});

  scenario.addAgent(agent);

  std::vector<RVO::Vector2> obstacle;
  obstacle.push_back(RVO::Vector2(11.0f, -0.1f));
  obstacle.push_back(RVO::Vector2(9.0f, -0.1f));
  obstacle.push_back(RVO::Vector2(9.0f, -3.f));
  obstacle.push_back(RVO::Vector2(11.0f, -3.f));

  scenario.addGlobalObstacle(obstacle);
}

void setupGridObstacleScenario(RVO::Scenario & scenario)
{
  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 1.0;  //4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 10.0f;

  RVO::Vector2 pos1(-1.0f, -1.0f);
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1, conf);
  agent1->addWaypoint({31.0f, 31.0f});
  scenario.addAgent(agent1);
  //scenario.addAgent(agent1);

  RVO::Vector2 pos2(31.0f, -1.0f);
  auto agent2 = std::make_shared<RVO::Agent>("agent2", pos2, conf);
  agent2->addWaypoint({-1.0f, 31.0f});
  scenario.addAgent(agent2);

  RVO::Vector2 pos3(31.0f, 31.0f);
  auto agent3 = std::make_shared<RVO::Agent>("agent3", pos3, conf);
  agent3->addWaypoint({-1.0f, -1.0f});
  scenario.addAgent(agent3);

  RVO::Vector2 pos4(-1.0f, 31.0f);
  auto agent4 = std::make_shared<RVO::Agent>("agent4", pos4, conf);
  agent4->addWaypoint({31.0f, -1.0f});
  scenario.addAgent(agent4);

  float obstacle_size = 2.0;
  float path_width = 5.0;
  float obstacle_origin_x = 0.0;
  float obstacle_origin_y = 0.0;
  int obstacle_num = 5;

  for (int x = 0; x < obstacle_num; x++) {
    for (int y = 0; y < obstacle_num; y++) {
      std::vector<RVO::Vector2> obstacle;
      obstacle.push_back(RVO::Vector2(
        x * (obstacle_origin_x + obstacle_size + path_width),
        obstacle_origin_y + y * (obstacle_size + path_width)));
      obstacle.push_back(RVO::Vector2(
        x * (obstacle_origin_x + obstacle_size + path_width) + obstacle_size,
        obstacle_origin_y + y * (obstacle_size + path_width)));
      obstacle.push_back(RVO::Vector2(
        x * (obstacle_origin_x + obstacle_size + path_width) + obstacle_size,
        obstacle_origin_y + y * (obstacle_size + path_width) + obstacle_size));
      obstacle.push_back(RVO::Vector2(
        x * (obstacle_origin_x + obstacle_size + path_width),
        obstacle_origin_y + y * (obstacle_size + path_width) + obstacle_size));
      //obstacles.push_back(obstacle);
      scenario.addGlobalObstacle(obstacle);
    }
  }
}

void setupDynamicVelocityScenario(RVO::Scenario & scenario)
{
  RVO::AgentConfig conf;
  conf.radius = 1.0f;
  conf.max_speed = 2.0f;
  conf.neighbor_dist = 1.0;  //4.0f;
  conf.time_horizon = 20.0f;
  conf.time_horizon_obst = 10.0f;

  RVO::Vector2 pos1(-0.0f, 0.0f);
  auto agent1 = std::make_shared<RVO::Agent>("agent1", pos1, conf);
  agent1->addWaypoint({20.0f, 20.0f});
  scenario.addAgent(agent1);
}

#endif  // CUSTOMIZED_RVO2_EXAMPLES__EXAMPLE_SCENARIO_HPP_
