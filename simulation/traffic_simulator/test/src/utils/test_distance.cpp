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

/**
 * @note Test if the function correctly uses getLateralDistance. Test with a scenario
 * in which it is impossible to calculate the distance, e.g. not connected lanelets
 * and with allow_lane_change = false.
 */
TEST(distance, lateralDistance_impossible_noChange)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
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
TEST(distance, lateralDistance_possible_noChange)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
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
TEST(distance, lateralDistance_impossible_change)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
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
TEST(distance, lateralDistance_possible_change)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
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
TEST(distance, lateralDistance_impossible_matching)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
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
TEST(distance, lateralDistance_possible_matching)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
  const auto pose_from = traffic_simulator::helper::constructCanonicalizedLaneletPose(
    3002184L, 0.0, 0.0, hdmap_utils_ptr);
  const auto pose_to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(202L, 0.0, 0.0, hdmap_utils_ptr);
  constexpr double approx_distance = -3.0;
  constexpr double tolerance = 0.5;
  {
    const auto result =
      traffic_simulator::distance::lateralDistance(pose_from, pose_to, 3.0, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
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
TEST(distance, longitudinalDistance_noAdjacent_noOpposite_noChange_false)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));

  const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81595.44, 50006.09, 100.0), false, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81584.48, 50084.76, 100.0), false, hdmap_utils_ptr);
  {
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
TEST(distance, longitudinalDistance_noAdjacent_noOpposite_noChange)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/standard_map/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.61836750154)
      .longitude(139.78066608243)
      .altitude(0.0));

  const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(3800.05, 73715.77, 30.0), false, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(3841.26, 73748.80, 110.0), false, hdmap_utils_ptr);
  {
    constexpr double approx_distance = 60.0;
    constexpr double tolerance = 1.0;
    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = false
 * in an impossible scenario, e.g. no path.
 */
TEST(distance, longitudinalDistance_adjacent_noOpposite_noChange_false)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));

  const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81585.79, 50042.62, 100.0), false, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81588.34, 50083.23, 100.0), false, hdmap_utils_ptr);
  {
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
TEST(distance, longitudinalDistance_adjacent_noOpposite_noChange)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));

  const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81599.02, 50065.76, 280.0), false, hdmap_utils_ptr);
  const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
    makePose(81599.61, 50045.16, 280.0), false, hdmap_utils_ptr);
  {
    constexpr double approx_distance = 20.0;
    constexpr double tolerance = 1.0;
    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), true, false, false, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = false,
 * include_opposite_direction = false, allow_lane_change = true
 * in an impossible scenario, e.g. no path.
 */
TEST(distance, longitudinalDistance_noAdjacent_noOpposite_change_false)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));

  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81595.47, 49982.80, 100.0), false, hdmap_utils_ptr);
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81599.34, 50022.34, 100.0), false, hdmap_utils_ptr);
    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    EXPECT_FALSE(result.has_value());
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81612.35, 50015.63, 280.0), false, hdmap_utils_ptr);
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81612.95, 49991.30, 280.0), false, hdmap_utils_ptr);
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
TEST(distance, longitudinalDistance_noAdjacent_noOpposite_change)
{
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/four_track_highway/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.22312494055522)
      .longitude(138.8024583466017)
      .altitude(0.0));
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81592.96, 49997.94, 100.0), false, hdmap_utils_ptr);
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81570.56, 50141.75, 100.0), false, hdmap_utils_ptr);
    constexpr double approx_distance = 145.0;
    constexpr double tolerance = 1.0;
    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
  {
    const auto pose_from = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81587.31, 50165.57, 100.0), false, hdmap_utils_ptr);
    const auto pose_to = traffic_simulator::toCanonicalizedLaneletPose(
      makePose(81610.25, 49988.59, 100.0), false, hdmap_utils_ptr);
    constexpr double approx_distance = 178.0;
    constexpr double tolerance = 1.0;
    const auto result = traffic_simulator::distance::longitudinalDistance(
      pose_from.value(), pose_to.value(), false, false, true, hdmap_utils_ptr);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result.value(), approx_distance, tolerance);
  }
}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = true
 * in an impossible scenario, e.g. no path.
 */
TEST(distance, longitudinalDistance_adjacent_noOpposite_change_false) {}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = false, allow_lane_change = true
 * in a scenario that meets those criteria.
 */
TEST(distance, longitudinalDistance_adjacent_noOpposite_change) {}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = true, allow_lane_change = false
 * in an impossible scenario, e.g. no path.
 */
TEST(distance, longitudinalDistance_adjacent_opposite_noChange_false) {}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = true, allow_lane_change = false
 * in a scenario that meets those criteria.
 */
TEST(distance, longitudinalDistance_adjacent_opposite_noChange) {}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = true, allow_lane_change = true
 * in an impossible scenario, e.g. no path.
 */
TEST(distance, longitudinalDistance_adjacent_opposite_change_false) {}

/**
 * @note Test calculation correctness with include_adjacent_lanelet = true,
 * include_opposite_direction = true, allow_lane_change = true
 * in a scenario that meets those criteria.
 */
TEST(distance, longitudinalDistance_adjacent_opposite_change) {}
