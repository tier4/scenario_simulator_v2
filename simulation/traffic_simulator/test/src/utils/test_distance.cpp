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
#include <geometry/bounding_box.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <numeric>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

#include "../helper_functions.hpp"

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
TEST_F(distanceTest_FourTrackHighway, longitudinalDistance_noAdjacent_noOpposite_change)
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

/**
 * @note Test equality with math::geometry::getPolygonDistance
 * function result on intersecting bounding boxes.
 */
TEST(distance, boundingBoxDistance_intersection)
{
  const auto pose_from = makePose(100.0, 100.0, 0.0);
  const auto pose_to = makePose(120.0, 100.0, 90.0);
  const auto bounding_box_from = makeCustom2DBoundingBox(30.0, 1.0);
  const auto bounding_box_to = makeCustom2DBoundingBox(1.0, 30.0);

  const auto result_distance = traffic_simulator::distance::boundingBoxDistance(
    pose_from, bounding_box_from, pose_to, bounding_box_to);
  const auto actual_distance =
    math::geometry::getPolygonDistance(pose_from, bounding_box_from, pose_to, bounding_box_to);
  EXPECT_FALSE(result_distance.has_value());
  EXPECT_FALSE(actual_distance.has_value());
}

/**
 * @note Test equality with math::geometry::getPolygonDistance
 * function result on disjoint bounding boxes.
 */
TEST(distance, boundingBoxDistance_disjoint)
{
  const auto pose_from = makePose(100.0, 100.0, 0.0);
  const auto pose_to = makePose(120.0, 100.0, 90.0);
  const auto bounding_box_from = makeCustom2DBoundingBox(1.0, 30.0);
  const auto bounding_box_to = makeCustom2DBoundingBox(30.0, 1.0);

  const auto result_distance = traffic_simulator::distance::boundingBoxDistance(
    pose_from, bounding_box_from, pose_to, bounding_box_to);
  const auto actual_distance =
    math::geometry::getPolygonDistance(pose_from, bounding_box_from, pose_to, bounding_box_to);
  ASSERT_TRUE(result_distance.has_value());
  ASSERT_TRUE(actual_distance.has_value());
  EXPECT_NEAR(
    actual_distance.value(), result_distance.value(), std::numeric_limits<double>::epsilon());
}

/**
 * @note Test calculation correctness with lanelet::Id.
 */
TEST_F(distanceTest_StandardMap, distanceToLeftLaneBound_single)
{
  constexpr lanelet::Id lanelet_id = 34741L;
  constexpr double tolerance = 0.1;
  {
    const auto pose = makePose(3818.33, 73726.18, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 0.5, tolerance);
  }
  {
    const auto pose = makePose(3816.89, 73723.09, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 2.6, tolerance);
  }
  {
    const auto pose = makePose(3813.42, 73721.11, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(3.0, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 2.7, tolerance);
  }
  {
    const auto pose = makePose(3813.42, 73721.11, 120.0);
    const auto bounding_box = makeCustom2DBoundingBox(3.0, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 1.3, tolerance);
  }
  {
    const auto pose = makePose(3810.99, 73721.40, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 1.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 1.4, tolerance);
  }
  {
    const auto pose = makePose(3810.99, 73721.40, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, -1.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 2.4, tolerance);
  }
  {
    const auto pose = makePose(3680.81, 73757.27, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, 34684L, hdmap_utils_ptr);
    EXPECT_NEAR(result, 5.1, tolerance);
  }
  {
    const auto pose = makePose(3692.79, 73753.00, 30.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, 34684L, hdmap_utils_ptr);
    EXPECT_NEAR(result, 7.2, tolerance);
  }
}

/**
 * @note Test calculation correctness with a vector containing multiple lanelets.
 * Test equality with the minimum of distanceToLeftLaneBound results (lanelet::Id overload).
 */
TEST_F(distanceTest_StandardMap, distanceToLeftLaneBound_multipleVector)
{
  const auto lanelet_ids = lanelet::Ids{34603L, 34600L, 34621L, 34741L};
  const auto pose = makePose(3836.16, 73757.99, 120.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  const double actual_distance = std::transform_reduce(
    lanelet_ids.cbegin(), lanelet_ids.cend(), std::numeric_limits<double>::max(),
    [](const double lhs, const double rhs) { return std::min(lhs, rhs); },
    [&pose, &bounding_box, this](const lanelet::Id lanelet_id) {
      return traffic_simulator::distance::distanceToLeftLaneBound(
        pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    });
  const double result_distance = traffic_simulator::distance::distanceToLeftLaneBound(
    pose, bounding_box, lanelet_ids, hdmap_utils_ptr);
  EXPECT_NEAR(actual_distance, result_distance, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result_distance, 1.4, 0.1);
}

/**
 * @note Test calculation correctness with a vector containing a single lanelet.
 * Test equality with the distanceToLeftLaneBound results (lanelet::Id overload).
 */
TEST_F(distanceTest_StandardMap, distanceToLeftLaneBound_singleVector)
{
  constexpr lanelet::Id lanelet_id = 34426L;
  const auto pose = makePose(3693.34, 73738.37, 300.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  const double actual_distance = traffic_simulator::distance::distanceToLeftLaneBound(
    pose, bounding_box, lanelet_id, hdmap_utils_ptr);
  const double result_distance = traffic_simulator::distance::distanceToLeftLaneBound(
    pose, bounding_box, {lanelet_id}, hdmap_utils_ptr);
  EXPECT_NEAR(actual_distance, result_distance, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result_distance, 1.8, 0.1);
}

/**
 * @note Test function behavior with an empty vector.
 */
TEST_F(distanceTest_StandardMap, distanceToLeftLaneBound_emptyVector)
{
  const auto pose = makePose(3825.87, 73773.08, 135.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  EXPECT_THROW(
    traffic_simulator::distance::distanceToLeftLaneBound(
      pose, bounding_box, lanelet::Ids{}, hdmap_utils_ptr),
    common::SemanticError);
}

/**
 * @note Test calculation correctness with lanelet::Id.
 */
TEST_F(distanceTest_Intersection, distanceToRightLaneBound_single)
{
  constexpr lanelet::Id lanelet_id = 660L;
  constexpr double tolerance = 0.1;
  {
    const auto pose = makePose(86651.84, 44941.47, 135.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 4.1, tolerance);
  }
  {
    const auto pose = makePose(86653.05, 44946.74, 135.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 0.6, tolerance);
  }
  {
    const auto pose = makePose(86651.47, 44941.07, 120.0);
    const auto bounding_box = makeCustom2DBoundingBox(3.0, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 4.3, tolerance);
  }
  {
    const auto pose = makePose(86651.47, 44941.07, 210.0);
    const auto bounding_box = makeCustom2DBoundingBox(3.0, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 3.1, tolerance);
  }
  {
    const auto pose = makePose(86644.10, 44951.86, 150.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 1.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 2.0, tolerance);
  }
  {
    const auto pose = makePose(86644.10, 44951.86, 150.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, -1.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 1.1, tolerance);
  }
  {
    const auto pose = makePose(86644.11, 44941.21, 0.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 11.2, tolerance);
  }
  {
    const auto pose = makePose(86656.83, 44946.96, 0.0);
    const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
    const double result = traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    EXPECT_NEAR(result, 2.6, tolerance);
  }
}

/**
 * @note Test calculation correctness with a vector containing multiple lanelets.
 * Test equality with the minimum of distanceToRightLaneBound results (lanelet::Id overload).
 */
TEST_F(distanceTest_Intersection, distanceToRightLaneBound_multipleVector)
{
  const auto lanelet_ids = lanelet::Ids{660L, 663L, 684L, 654L, 686L};
  const auto pose = makePose(86642.05, 44902.61, 60.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  const double actual_distance = std::transform_reduce(
    lanelet_ids.cbegin(), lanelet_ids.cend(), std::numeric_limits<double>::max(),
    [](const double lhs, const double rhs) { return std::min(lhs, rhs); },
    [&pose, &bounding_box, this](const lanelet::Id lanelet_id) {
      return traffic_simulator::distance::distanceToRightLaneBound(
        pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    });
  const double result_distance = traffic_simulator::distance::distanceToRightLaneBound(
    pose, bounding_box, lanelet_ids, hdmap_utils_ptr);
  EXPECT_NEAR(actual_distance, result_distance, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result_distance, 2.7, 0.1);
}

/**
 * @note Test calculation correctness with a vector containing a single lanelet.
 * Test equality with the distanceToRightLaneBound result (lanelet::Id overload).
 */
TEST_F(distanceTest_Intersection, distanceToRightLaneBound_singleVector)
{
  constexpr lanelet::Id lanelet_id = 654L;
  const auto pose = makePose(86702.79, 44929.05, 150.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  const double actual_distance = traffic_simulator::distance::distanceToRightLaneBound(
    pose, bounding_box, lanelet_id, hdmap_utils_ptr);
  const double result_distance = traffic_simulator::distance::distanceToRightLaneBound(
    pose, bounding_box, {lanelet_id}, hdmap_utils_ptr);
  EXPECT_NEAR(actual_distance, result_distance, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(result_distance, 2.4, 0.1);
}

/**
 * @note Test function behavior with an empty vector.
 */
TEST_F(distanceTest_Intersection, distanceToRightLaneBound_emptyVector)
{
  const auto pose = makePose(3825.87, 73773.08, 135.0);
  const auto bounding_box = makeCustom2DBoundingBox(0.1, 0.1, 0.0, 0.0);
  EXPECT_THROW(
    traffic_simulator::distance::distanceToRightLaneBound(
      pose, bounding_box, lanelet::Ids{}, hdmap_utils_ptr),
    common::SemanticError);
}
