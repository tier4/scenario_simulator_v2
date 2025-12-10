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

#ifndef TRAFFIC_SIMULATOR__TEST__TEST_LANELET_WRAPPER_HPP_
#define TRAFFIC_SIMULATOR__TEST__TEST_LANELET_WRAPPER_HPP_

#include <geometry_msgs/msg/point.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <string>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/distance.hpp>
#include <traffic_simulator/lanelet_wrapper/lane_change.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>
#include <traffic_simulator/lanelet_wrapper/traffic_lights.hpp>

#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

class LaneletWrapperTest_StandardMap : public testing::Test
{
protected:
  LaneletWrapperTest_StandardMap() { activateLaneletWrapper("standard_map"); }
};
class LaneletWrapperTest_WithoutLightBulb : public testing::Test
{
protected:
  LaneletWrapperTest_WithoutLightBulb() { activateLaneletWrapper("minimal_map"); }
};

class LaneletWrapperTest_WithRoadShoulderMap : public testing::Test
{
protected:
  LaneletWrapperTest_WithRoadShoulderMap() { activateLaneletWrapper("with_road_shoulder"); }
};
class LaneletWrapperTest_EmptyMap : public testing::Test
{
protected:
  LaneletWrapperTest_EmptyMap() { activateLaneletWrapper("empty"); }
};
class LaneletWrapperTest_FourTrackHighwayMap : public testing::Test
{
protected:
  LaneletWrapperTest_FourTrackHighwayMap() { activateLaneletWrapper("four_track_highway"); }
};
class LaneletWrapperTest_CrossroadsWithStoplinesMap : public testing::Test
{
protected:
  LaneletWrapperTest_CrossroadsWithStoplinesMap()
  {
    activateLaneletWrapper("crossroads_with_stoplines");
  }
};
class LaneletWrapperTest_KashiwanohaMap : public testing::Test
{
protected:
  LaneletWrapperTest_KashiwanohaMap()
  {
    const auto lanelet_path =
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map/lanelet2_map.osm";
    traffic_simulator::lanelet_map::activate(lanelet_path);
  }
};
class LaneletWrapperTest_IntersectionMap : public testing::Test
{
protected:
  LaneletWrapperTest_IntersectionMap() { activateLaneletWrapper("intersection"); }
};

#endif  // TRAFFIC_SIMULATOR__TEST__TEST_LANELET_WRAPPER_HPP_
