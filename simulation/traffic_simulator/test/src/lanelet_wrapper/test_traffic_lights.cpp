// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include "test_lanelet_wrapper.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace traffic_simulator::lanelet_wrapper::tests
{
/**
 * @note Test basic functionality.
 * Test traffic light checking correctness with an id of a traffic light.
 */
TEST_F(LaneletWrapperTest_StandardMap, isTrafficLight_trafficLight)
{
  EXPECT_TRUE(traffic_lights::isTrafficLight(34836));
}

/**
 * @note Test basic functionality.
 * Test traffic light checking correctness with an id of not a traffic light.
 */
TEST_F(LaneletWrapperTest_StandardMap, isTrafficLight_notTrafficLight)
{
  EXPECT_FALSE(traffic_lights::isTrafficLight(120659));
}

/**
 * @note Test function behavior when called with an invalid lanelet id.
 */
TEST_F(LaneletWrapperTest_StandardMap, isTrafficLight_invalidId)
{
  EXPECT_FALSE(traffic_lights::isTrafficLight(1000003));
}

/**
 * @note Test basic functionality.
 * Test traffic light relation checking correctness
 * with an id of a lanelet that has a relation with a traffic light.
 */
TEST_F(
  LaneletWrapperTest_StandardMap, isTrafficLightRegulatoryElement_trafficLightRegulatoryElement)
{
  EXPECT_TRUE(traffic_lights::isTrafficLightRegulatoryElement(34806));
}

/**
 * @note Test basic functionality.
 * Test traffic light relation checking correctness
 * with an id of a lanelet that does not have a relation with a traffic light.
 */
TEST_F(
  LaneletWrapperTest_StandardMap, isTrafficLightRegulatoryElement_noTrafficLightRegulatoryElement)
{
  EXPECT_FALSE(traffic_lights::isTrafficLightRegulatoryElement(120659));
}

/**
 * @note Test function behavior when called with an invalid lanelet id.
 */
TEST_F(LaneletWrapperTest_StandardMap, isTrafficLightRegulatoryElement_invalidId)
{
  EXPECT_FALSE(traffic_lights::isTrafficLightRegulatoryElement(1000003));
}
/**
 * @note Test basic functionality.
 * Test traffic lights id obtaining correctness.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightIds_correct)
{
  auto result_traffic_lights = traffic_lights::trafficLightIds();

  std::sort(result_traffic_lights.begin(), result_traffic_lights.end());
  EXPECT_EQ(result_traffic_lights, (lanelet::Ids{34802, 34836}));
}

/**
 * @note Test function behavior when there are no traffic lights on the map.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getTrafficLightIds_noTrafficLight)
{
  EXPECT_EQ(traffic_lights::trafficLightIds().size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test traffic light position obtaining
 * with a traffic light and bulb color specified.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightBulbPosition_correct)
{
  const lanelet::Id light_id = 34802;
  const double epsilon = 0.1;

  {
    const auto return_bulb_position = traffic_lights::trafficLightBulbPosition(light_id, "green");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3761.05, 73755.30, 5.35), epsilon);
  }

  {
    const auto return_bulb_position = traffic_lights::trafficLightBulbPosition(light_id, "yellow");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.60, 73755.07, 5.35), epsilon);
  }

  {
    const auto return_bulb_position = traffic_lights::trafficLightBulbPosition(light_id, "red");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.16, 73754.87, 5.35), epsilon);
  }

  {
    EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(light_id, "pink").has_value());
  }
}

/**
 * @note Test basic functionality.
 * Test traffic light position obtaining
 * with a traffic light and bulb color specified.
 */
TEST_F(LaneletWrapperTest_WithoutLightBulb, getTrafficLightBulbPositionInfer_correct)
{
  const lanelet::Id light_id = 34802;
  const double epsilon = 0.1;

  {
    const auto return_bulb_position =
      traffic_lights::trafficLightBulbPosition(light_id, "green", true);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3761.05, 73755.30, 5.35), epsilon);
  }

  {
    const auto return_bulb_position =
      traffic_lights::trafficLightBulbPosition(light_id, "yellow", true);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.60, 73755.07, 5.35), epsilon);
  }

  {
    const auto return_bulb_position =
      traffic_lights::trafficLightBulbPosition(light_id, "red", true);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.16, 73754.87, 5.35), epsilon);
  }

  {
    EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(light_id, "green").has_value());
  }

  {
    EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(light_id, "yellow").has_value());
  }

  {
    EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(light_id, "red").has_value());
  }

  {
    EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(light_id, "pink").has_value());
  }
}

/**
 * @note Test basic functionality.
 * Test traffic light position obtaining
 * with an id of a traffic light that does not exist
 * - the goal is to test the branch when no traffic light is selected.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightBulbPosition_invalidTrafficLight)
{
  EXPECT_FALSE(traffic_lights::trafficLightBulbPosition(1000003, "red").has_value());
}
/**
 * @note Test basic functionality.
 * Test traffic light ids obtaining correctness
 * with a route that does not have any traffic lights.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightIdsOnPath_noTrafficLights)
{
  EXPECT_EQ(
    traffic_lights::trafficLightIdsOnPath(lanelet::Ids{34579, 34774, 120659, 120660, 34468, 34438})
      .size(),
    static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test traffic light ids obtaining correctness with a route that has some traffic lights.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightIdsOnPath_trafficLights)
{
  auto result_traffic_light_ids = traffic_lights::trafficLightIdsOnPath(
    {34579, 34774, 120659, 120660, 34468, 34438, 34408, 34624, 34630});
  auto actual_traffic_light_ids = lanelet::Ids{34802, 34836};

  std::sort(result_traffic_light_ids.begin(), result_traffic_light_ids.end());
  std::sort(actual_traffic_light_ids.begin(), actual_traffic_light_ids.end());

  EXPECT_EQ(result_traffic_light_ids, actual_traffic_light_ids);
}

/**
 * @note Test function behavior when passed an empty vector of lanelet ids.
 */
TEST_F(LaneletWrapperTest_StandardMap, getTrafficLightIdsOnPath_empty)
{
  EXPECT_EQ(traffic_lights::trafficLightIdsOnPath({}).size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining traffic light stop line ids
 * correctness with a traffic light that has one stop line.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_stopLine)
{
  EXPECT_EQ(traffic_lights::trafficLightStopLineIds(34802), (lanelet::Ids{34805}));
}

/**
 * @note Test basic functionality.
 * Test obtaining traffic light stop line ids
 * correctness with a traffic light that has several stop lines
 * - the goal is to test the scenario where one traffic light has multiple stop lines
 * (e.g. on a road with two parallel lanes with the same direction).
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_severalStopLines)
{
  auto result_stoplines = traffic_lights::trafficLightStopLineIds(34836);
  auto actual_stoplines = lanelet::Ids{120663, 34805};

  std::sort(result_stoplines.begin(), result_stoplines.end());
  std::sort(actual_stoplines.begin(), actual_stoplines.end());

  EXPECT_EQ(result_stoplines, actual_stoplines);
}

/**
 * @note Test function behavior when passed an invalid traffic light id.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_invalidTrafficLightId)
{
  EXPECT_THROW(traffic_lights::trafficLightStopLineIds(1000039), std::runtime_error);
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
