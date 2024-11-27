// Copyright 2021 Tier IV, Inc All rights reserved.
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

#include <context_gamma_planner/pedestrian_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

class PedestrianPluginTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    plugin = std::make_unique<context_gamma_planner::PedestrianPlugin>();
    plugin->configure(rclcpp::get_logger("gtest"));
  }
  virtual void TearDown() { plugin.release(); }

public:
  std::unique_ptr<context_gamma_planner::PedestrianPlugin> plugin;
};

TEST_F(PedestrianPluginTest, CurrentTime)
{
  plugin->setCurrentTime(3.4);
  EXPECT_DOUBLE_EQ(plugin->getCurrentTime(), 3.4);
}

TEST_F(PedestrianPluginTest, DebugMarker)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.scale.x = 3.0;
  marker.colors.emplace_back(std_msgs::msg::ColorRGBA());
  marker.action = marker.ADD;
  marker.type = marker.TRIANGLE_LIST;
  marker.text = "test";
  markers.emplace_back(marker);
  plugin->setDebugMarker(markers);
  EXPECT_EQ(plugin->getDebugMarker().size(), static_cast<size_t>(1));
  EXPECT_EQ(plugin->getDebugMarker()[0], marker);
}

TEST_F(PedestrianPluginTest, BehaviorParameter)
{
  traffic_simulator_msgs::msg::BehaviorParameter model;
  model.dynamic_constraints.max_acceleration = 31.3;
  model.dynamic_constraints.max_deceleration = 20.1;
  model.lane_change_distance = 30.4;
  model.see_around = false;
  plugin->setBehaviorParameter(model);
  EXPECT_EQ(plugin->getBehaviorParameter(), model);
}

TEST_F(PedestrianPluginTest, HdMapUtils)
{
  std::string path = ament_index_cpp::get_package_share_directory("context_gamma_planner") +
                     "/maps/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
  plugin->setHdMapUtils(hdmap_utils_ptr);
  EXPECT_TRUE(plugin->getHdMapUtils());
}

TEST_F(PedestrianPluginTest, Obstacle)
{
  plugin->setObstacle(std::nullopt);
  EXPECT_TRUE(plugin->getObstacle() == std::nullopt);
  plugin->setObstacle(traffic_simulator_msgs::msg::Obstacle());
  EXPECT_EQ(plugin->getObstacle(), traffic_simulator_msgs::msg::Obstacle());
}

TEST_F(PedestrianPluginTest, PedestrianParameters)
{
  traffic_simulator_msgs::msg::PedestrianParameters param;
  param.bounding_box.center.x = 20.0;
  param.name = "ego";
  plugin->setPedestrianParameters(param);
  EXPECT_EQ(plugin->getPedestrianParameters(), param);
}

TEST_F(PedestrianPluginTest, Requests)
{
  plugin->setRequest(traffic_simulator::behavior::Request::WALK_STRAIGHT);
  EXPECT_EQ(plugin->getRequest(), traffic_simulator::behavior::Request::WALK_STRAIGHT);
}

TEST_F(PedestrianPluginTest, RouteLanelets)
{
  std::vector<lanelet::Id> route = {1, 32, 31};
  plugin->setRouteLanelets(route);
  EXPECT_EQ(plugin->getRouteLanelets()[0], 1);
  EXPECT_EQ(plugin->getRouteLanelets()[1], 32);
  EXPECT_EQ(plugin->getRouteLanelets()[2], 31);
}

TEST_F(PedestrianPluginTest, TargetSpeed)
{
  plugin->setTargetSpeed(std::nullopt);
  EXPECT_EQ(plugin->getTargetSpeed(), std::nullopt);
  plugin->setTargetSpeed(15);
  EXPECT_DOUBLE_EQ(plugin->getTargetSpeed().value(), 15);
}

TEST_F(PedestrianPluginTest, StepTime)
{
  plugin->setStepTime(0.01);
  EXPECT_DOUBLE_EQ(plugin->getStepTime(), 0.01);
}

TEST_F(PedestrianPluginTest, LaneChangeParameters)
{
  plugin->setLaneChangeParameters(traffic_simulator::lane_change::Parameter());
  EXPECT_EQ(
    plugin->getLaneChangeParameters().trajectory_shape,
    traffic_simulator::lane_change::TrajectoryShape::CUBIC);
}

/*
TEST_F(PedestrianPluginTest, TrafficLightManager)
{
  std::string path = ament_index_cpp::get_package_share_directory("context_gamma_planner") +
                     "/maps/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
  const auto manager = std::make_shared<traffic_simulator::TrafficLightManager>(
    hdmap_utils_ptr, nullptr, nullptr, nullptr);
  plugin->setTrafficLightManager(manager);
  EXPECT_EQ(plugin->getTrafficLightManager()->getIds().size(), static_cast<size_t>(2));
}
*/

TEST_F(PedestrianPluginTest, VehicleParameters)
{
  traffic_simulator_msgs::msg::VehicleParameters parameters;
  parameters.bounding_box.center.x = 4.2;
  plugin->setVehicleParameters(parameters);
  EXPECT_EQ(plugin->getVehicleParameters(), parameters);
  EXPECT_DOUBLE_EQ(plugin->getVehicleParameters().bounding_box.center.x, 4.2);
}

TEST_F(PedestrianPluginTest, Waypoint)
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  p1.x = 1;
  waypoints.waypoints.emplace_back(p0);
  waypoints.waypoints.emplace_back(p1);
  plugin->setWaypoints(waypoints);
  EXPECT_EQ(plugin->getWaypoints(), waypoints);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
