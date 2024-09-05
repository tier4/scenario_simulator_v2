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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class distanceTest_FourTrackHighway : public testing::Test
{
protected:
  distanceTest_FourTrackHighway()
  : hdmap_utils_ptr(std::make_shared<hdmap_utils::HdMapUtils>(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/four_track_highway/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.22312494055522)
        .longitude(138.8024583466017)
        .altitude(0.0)))
  {
  }
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
};

class distanceTest_StandardMap : public testing::Test
{
protected:
  distanceTest_StandardMap()
  : hdmap_utils_ptr(std::make_shared<hdmap_utils::HdMapUtils>(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/standard_map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0)))
  {
  }
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
};

class distanceTest_Intersection : public testing::Test
{
protected:
  distanceTest_Intersection()
  : hdmap_utils_ptr(std::make_shared<hdmap_utils::HdMapUtils>(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/intersection/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.64200728302)
        .longitude(139.74821144562)
        .altitude(0.0)))
  {
  }
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
};

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is impossible to calculate the distance, e.g. not connected lanelets
 * and with allow_lane_change = false.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_impossible_noChange)
{
  const auto pose_from = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);
  {
    const auto result = traffic_simulator::distance::lateralDistance(
      pose_from, pose_to, std::numeric_limits<double>::infinity(), false, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, false, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is possible to calculate the distance
 * and with allow_lane_change = false.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_possible_noChange)
{
  const auto pose_from = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(201L, 0.0, 0.0, hdmap_utils_ptr);
  {
    const auto result = traffic_simulator::distance::lateralDistance(
      pose_from, pose_to, std::numeric_limits<double>::infinity(), false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 0.0, std::numeric_limits<double>::epsilon());
  }
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 0.0, std::numeric_limits<double>::epsilon());
  }
}

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is impossible to calculate the distance, e.g. not connected lanelets
 * and with allow_lane_change = true.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_impossible_change)
{
  const auto pose_from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  {
    const auto result = traffic_simulator::distance::lateralDistance(
      pose_from, pose_to, std::numeric_limits<double>::infinity(), true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is possible to calculate the distance
 * and with allow_lane_change = true.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_possible_change)
{
  const auto pose_from = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);
  constexpr double approx_distance = -3.0;
  constexpr double tolerance = 0.5;
  {
    const auto result = traffic_simulator::distance::lateralDistance(
      pose_from, pose_to, std::numeric_limits<double>::infinity(), true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
}

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is possible to calculate the distance, but matching_distance is too small.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_impossible_matching)
{
  const auto pose_from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, 2.0, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is possible to calculate the distance and matching_distance is large.
 */
TEST_F(distanceTest_FourTrackHighway, lateralDistance_possible_matching)
{
  const auto pose_from = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);

  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, 3.0, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), -3.0, 0.5);
  }
}

constexpr auto convertDegToRad(const double deg) -> double { return deg / 180.0 * M_PI; }
constexpr auto convertRadToDeg(const double rad) -> double { return rad * 180.0 / M_PI; }

auto makePose(const double x, const double y, const double yaw_deg) -> geometry_msgs::msg::Pose
{
  /**
   * @note +x axis is  0 degrees; +y axis is 90 degrees
   */
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(0.0))
    .orientation(math::geometry::convertEulerAngleToQuaternion(
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(
        convertDegToRad(yaw_deg))));
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = false
 * in an impossible scenario, e.g. no path.
 */
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_noAdjacent_noOpposite_noChange_false)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81595.44, 50006.09, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81584.48, 50084.76, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, false, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = false
 * in a scenario that meets those criteria.
 */
TEST_F(distanceTest_StandardMap, longitudinalDistance_noAdjacent_noOpposite_noChange)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(3800.05, 73715.77, 30.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(3841.26, 73748.80, 110.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 60.0, 1.0);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = false
 * in an impossible scenario, e.g. no path.
 */
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_adjacent_noOpposite_noChange_false)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81585.79, 50042.62, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81588.34, 50083.23, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, false, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = false
 * in a scenario that meets those criteria.
 */
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_adjacent_noOpposite_noChange)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81599.02, 50065.76, 280.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81599.61, 50045.16, 280.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 20.0, 1.0);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = true
 * in an impossible scenario, e.g. no path.
 */
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_noAdjacent_noOpposite_change_false)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81595.47, 49982.80, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81599.34, 50022.34, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81612.35, 50015.63, 280.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81612.95, 49991.30, 280.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = true
 * in a scenario that meets those criteria.
 */
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_noAdjacent_noOpposite_change_case0)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81592.96, 49997.94, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81570.56, 50141.75, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 145.0, 1.0);
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81587.31, 50165.57, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81610.25, 49988.59, 100.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 178.0, 1.0);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = true
 * in a scenario that meets those criteria.
 */
TEST_F(distanceTest_Intersection, longitudinalDistance_noAdjacent_noOpposite_change_case1)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86627.71, 44972.06, 340.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86647.23, 44882.51, 240.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_TRUE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86555.38, 45000.88, 340.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86647.23, 44882.51, 240.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_TRUE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86788.82, 44993.77, 210.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86553.48, 44990.56, 150.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_TRUE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86787.85, 44998.44, 210.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86648.95, 44884.04, 240.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_TRUE(result.has_value());
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = true
 * in an impossible scenario, e.g. no path.
 */
TEST_F(distanceTest_Intersection, longitudinalDistance_adjacent_noOpposite_change_false)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86736.13, 44969.63, 210.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86642.95, 44958.78, 340.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86732.06, 44976.58, 210.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86704.59, 44927.32, 340.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = true
 * in a scenario that meets those criteria.
 */
TEST_F(distanceTest_Intersection, longitudinalDistance_adjacent_noOpposite_change)
{
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86637.19, 44967.35, 340.0), false, hdmap_utils_ptr);
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86648.82, 44886.19, 240.0), false, hdmap_utils_ptr);

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 103.0, 1.0);
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86719.94, 44957.20, 210.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(86599.32, 44975.01, 180.0), false, hdmap_utils_ptr);
    ASSERT_TRUE(pose_from.has_value());

    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), 131.0, 1.0);
  }
}
