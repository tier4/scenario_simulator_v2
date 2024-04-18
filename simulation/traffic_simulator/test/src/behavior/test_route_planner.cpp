#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(RoutePlanner, getRouteLanelets_empty)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);

  traffic_simulator::RoutePlanner route_planner(hdmap_utils_ptr);
  route_planner.setWaypoints({});

  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose entity_pose(
    traffic_simulator::helper::constructLaneletPose(120659, 1, 0), hdmap_utils_ptr);
  const auto route_lanelets = route_planner.getRouteLanelets(entity_pose, 100.0);

  const auto following_lanelets = hdmap_utils_ptr->getFollowingLanelets(120659, 100.0, true);
  EXPECT_EQ(route_lanelets, following_lanelets);
}
