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

#include <boost/lexical_cast.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>

TEST(TrafficLights, OPERATOR_INSERT)
{
  using namespace traffic_simulator;

  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightArrow::STRAIGHT).c_str(), "straight");
  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightArrow::NONE).c_str(), "none");
  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightArrow::LEFT).c_str(), "left");
  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightArrow::RIGHT).c_str(), "right");

  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightColor::RED).c_str(), "red");
  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightColor::GREEN).c_str(), "green");
  EXPECT_STREQ(boost::lexical_cast<std::string>(TrafficLightColor::YELLOW).c_str(), "yellow");
}

TEST(TrafficLights, OPERATOR_INPUT)
{
  using namespace traffic_simulator;

  EXPECT_EQ(boost::lexical_cast<TrafficLightArrow>("straight"), TrafficLightArrow::STRAIGHT);
  EXPECT_EQ(boost::lexical_cast<TrafficLightArrow>("none"), TrafficLightArrow::NONE);
  EXPECT_EQ(boost::lexical_cast<TrafficLightArrow>("left"), TrafficLightArrow::LEFT);
  EXPECT_EQ(boost::lexical_cast<TrafficLightArrow>("right"), TrafficLightArrow::RIGHT);
  EXPECT_THROW(boost::lexical_cast<TrafficLightArrow>("invalid"), common::SimulationError);

  EXPECT_EQ(boost::lexical_cast<TrafficLightColor>("red"), TrafficLightColor::RED);
  EXPECT_EQ(boost::lexical_cast<TrafficLightColor>("green"), TrafficLightColor::GREEN);
  EXPECT_EQ(boost::lexical_cast<TrafficLightColor>("yellow"), TrafficLightColor::YELLOW);
  EXPECT_THROW(boost::lexical_cast<TrafficLightColor>("invalid"), common::SimulationError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
