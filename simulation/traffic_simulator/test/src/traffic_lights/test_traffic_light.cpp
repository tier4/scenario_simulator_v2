// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>

#include "../expect_eq_macros.hpp"

geometry_msgs::msg::Point getPoint(double val)
{
  geometry_msgs::msg::Point p;
  p.x = val;
  p.y = val;
  p.z = val;
  return p;
}

TEST(TrafficLights, getPositionError)
{
  traffic_simulator::TrafficLight light(0);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightColor::NONE), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightColor::GREEN), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightColor::YELLOW), common::SemanticError);
  EXPECT_THROW(light.getPosition(traffic_simulator::TrafficLightColor::RED), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::NONE), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::LEFT), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::RIGHT), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::STRAIGHT), common::SemanticError);
}

TEST(TrafficLights, getColorPosition)
{
  std::unordered_map<traffic_simulator::TrafficLightColor, geometry_msgs::msg::Point>
    color_positions;
  color_positions[traffic_simulator::TrafficLightColor::GREEN] = getPoint(0);
  color_positions[traffic_simulator::TrafficLightColor::YELLOW] = getPoint(1);
  color_positions[traffic_simulator::TrafficLightColor::RED] = getPoint(2);
  traffic_simulator::TrafficLight light(0, color_positions);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightColor::NONE), common::SemanticError);
  EXPECT_POINT_EQ(light.getPosition(traffic_simulator::TrafficLightColor::GREEN), getPoint(0));
  EXPECT_POINT_EQ(light.getPosition(traffic_simulator::TrafficLightColor::YELLOW), getPoint(1));
  EXPECT_POINT_EQ(light.getPosition(traffic_simulator::TrafficLightColor::RED), getPoint(2));
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::NONE), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::LEFT), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::RIGHT), common::SemanticError);
  EXPECT_THROW(
    light.getPosition(traffic_simulator::TrafficLightArrow::STRAIGHT), common::SemanticError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
