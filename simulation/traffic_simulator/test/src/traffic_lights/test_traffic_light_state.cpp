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
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>

TEST(TrafficLights, makeLampState)
{
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightColor::RED).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::RED);
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightColor::GREEN).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::GREEN);
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightColor::YELLOW).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::AMBER);
  EXPECT_THROW(
    traffic_simulator::makeLampState(traffic_simulator::TrafficLightColor::NONE),
    std::out_of_range);
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightArrow::STRAIGHT).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW);
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightArrow::RIGHT).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW);
  // EXPECT_EQ(
  //   traffic_simulator::makeLampState(traffic_simulator::TrafficLightArrow::LEFT).color,
  //   autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW);
  EXPECT_THROW(
    traffic_simulator::makeLampState(traffic_simulator::TrafficLightArrow::NONE),
    std::out_of_range);
}

TEST(TrafficLights, OPERATOR_INSERT)
{
  std::stringstream ss;
  ss << traffic_simulator::TrafficLightArrow::STRAIGHT;
  EXPECT_STREQ(ss.str().c_str(), "straight");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightArrow::NONE;
  EXPECT_STREQ(ss.str().c_str(), "none");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightArrow::LEFT;
  EXPECT_STREQ(ss.str().c_str(), "left");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightArrow::RIGHT;
  EXPECT_STREQ(ss.str().c_str(), "right");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightColor::NONE;
  EXPECT_STREQ(ss.str().c_str(), "none");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightColor::RED;
  EXPECT_STREQ(ss.str().c_str(), "red");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightColor::GREEN;
  EXPECT_STREQ(ss.str().c_str(), "green");
  ss = std::stringstream();
  ss << traffic_simulator::TrafficLightColor::YELLOW;
  EXPECT_STREQ(ss.str().c_str(), "yellow");
}

TEST(TrafficLights, OPERATOR_INPUT)
{
  traffic_simulator::TrafficLightArrow arrow;
  std::stringstream("straight") >> arrow;
  EXPECT_EQ(arrow, traffic_simulator::TrafficLightArrow::STRAIGHT);
  std::stringstream("none") >> arrow;
  EXPECT_EQ(arrow, traffic_simulator::TrafficLightArrow::NONE);
  std::stringstream("left") >> arrow;
  EXPECT_EQ(arrow, traffic_simulator::TrafficLightArrow::LEFT);
  std::stringstream("right") >> arrow;
  EXPECT_EQ(arrow, traffic_simulator::TrafficLightArrow::RIGHT);
  EXPECT_THROW(std::stringstream("invalid") >> arrow, common::SimulationError);
  traffic_simulator::TrafficLightColor color;
  std::stringstream("red") >> color;
  EXPECT_EQ(color, traffic_simulator::TrafficLightColor::RED);
  std::stringstream("green") >> color;
  EXPECT_EQ(color, traffic_simulator::TrafficLightColor::GREEN);
  std::stringstream("yellow") >> color;
  EXPECT_EQ(color, traffic_simulator::TrafficLightColor::YELLOW);
  std::stringstream("none") >> color;
  EXPECT_EQ(color, traffic_simulator::TrafficLightColor::NONE);
  EXPECT_THROW(std::stringstream("invalid") >> color, common::SimulationError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
