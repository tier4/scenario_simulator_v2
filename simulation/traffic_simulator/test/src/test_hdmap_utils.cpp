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
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

TEST(HdMapUtils, Construct)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);
  ASSERT_NO_THROW(hdmap_utils.toMapBin());
}

TEST(HdMapUtils, MatchToLane)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = 0.0;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(120659, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 120659);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34411, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 34411);
  }
}

TEST(HdMapUtils, AlongLaneletPose)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30)
      .lanelet_id,
    34513);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(
        traffic_simulator::helper::constructLaneletPose(34513, 0, 0),
        hdmap_utils.getLaneletLength(34513) + 10.0)
      .lanelet_id,
    34510);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .s,
    30.0);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .lanelet_id,
    34513);

  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .lanelet_id,
    34684);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .s,
    hdmap_utils.getLaneletLength(34684) - 10.0);
}

TEST(HdMapUtils, RoadShoulder)
{
  std::string path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                     "/map/with_road_shoulder/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);
  const auto next_lanelet_ids = hdmap_utils.getNextLaneletIds(34696);
  EXPECT_EQ(next_lanelet_ids.size(), static_cast<size_t>(1));
  if (next_lanelet_ids.size() == 1) {
    EXPECT_EQ(next_lanelet_ids[0], static_cast<int64_t>(34768));
  }
  const auto previous_lanelet_ids = hdmap_utils.getPreviousLaneletIds(34768);
  EXPECT_EQ(previous_lanelet_ids.size(), static_cast<size_t>(1));
  if (previous_lanelet_ids.size() == 1) {
    EXPECT_EQ(previous_lanelet_ids[0], static_cast<int64_t>(34696));
  }
}

TEST(HdMapUtils, CanonicalizeNegative)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);

  const auto lanelet_len = hdmap_utils.getLaneletLength(34696);
  double canonicalized_lanelet_offset = -5;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34768, canonicalized_lanelet_offset, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34696);
  EXPECT_EQ(canonicalized_lanelet_pose.value().s, lanelet_len + canonicalized_lanelet_offset);
}

TEST(HdMapUtils, CanonicalizePositive)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);

  const auto lanelet_len = hdmap_utils.getLaneletLength(34981);
  double canonicalized_lanelet_offset = 5;
  const auto non_canonicalized_lanelet_pose = traffic_simulator::helper::constructLaneletPose(
    34981, lanelet_len + canonicalized_lanelet_offset, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34585);
  EXPECT_EQ(canonicalized_lanelet_pose.value().s, canonicalized_lanelet_offset);
}

TEST(HdMapUtils, CanonicalizeAllNegative)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);

  double canonicalized_lanelet_offset = -31;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34564, canonicalized_lanelet_offset, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.gelAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, canonicalized_lanelet_offset +
                                        hdmap_utils.getLaneletLength(34564) +
                                        hdmap_utils.getLaneletLength(34570));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34981);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, canonicalized_lanelet_offset +
                                        hdmap_utils.getLaneletLength(34564) +
                                        hdmap_utils.getLaneletLength(34636));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34600);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, canonicalized_lanelet_offset +
                                        hdmap_utils.getLaneletLength(34564) +
                                        hdmap_utils.getLaneletLength(34648));
}

TEST(HdMapUtils, CanonicalizeAllPositive)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  hdmap_utils::HdMapUtils hdmap_utils(path, origin);

  const auto lanelet_len = hdmap_utils.getLaneletLength(34981);
  double canonicalized_lanelet_offset = 25;
  const auto non_canonicalized_lanelet_pose = traffic_simulator::helper::constructLaneletPose(
    34981, lanelet_len + canonicalized_lanelet_offset, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.gelAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s,
    canonicalized_lanelet_offset - hdmap_utils.getLaneletLength(34585));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34564);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s,
    canonicalized_lanelet_offset - hdmap_utils.getLaneletLength(34636));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34630);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s,
    canonicalized_lanelet_offset - hdmap_utils.getLaneletLength(34651));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
