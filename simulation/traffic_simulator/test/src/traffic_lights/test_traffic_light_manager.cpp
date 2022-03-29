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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>

TEST(TrafficLightManager, getIds)
{
  const auto node = std::make_shared<rclcpp::Node>("getIds");
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
  traffic_simulator::TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>
    manager(hdmap_utils_ptr, node, "map");
  EXPECT_FALSE(manager.getTrafficLights().find(34836) == std::end(manager.getTrafficLights()));
  EXPECT_FALSE(manager.getTrafficLights().find(34802) == std::end(manager.getTrafficLights()));
  EXPECT_EQ(manager.getTrafficLights().size(), static_cast<std::size_t>(2));
}

TEST(TrafficLightManager, setColor)
{
  const auto node = std::make_shared<rclcpp::Node>("setColor");
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
  traffic_simulator::TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>
    manager(hdmap_utils_ptr, node, "map");
  for (const auto & [id, traffic_light] : manager.getTrafficLights()) {
    EXPECT_EQ(manager.getColor(id), traffic_simulator::TrafficLightColor::NONE);
    manager.setColor(id, traffic_simulator::TrafficLightColor::GREEN);
    EXPECT_EQ(manager.getColor(id), traffic_simulator::TrafficLightColor::GREEN);
    manager.setColor(id, traffic_simulator::TrafficLightColor::YELLOW);
    EXPECT_EQ(manager.getColor(id), traffic_simulator::TrafficLightColor::YELLOW);
    manager.setColor(id, traffic_simulator::TrafficLightColor::RED);
    EXPECT_EQ(manager.getColor(id), traffic_simulator::TrafficLightColor::RED);
    manager.setColor(id, traffic_simulator::TrafficLightColor::NONE);
    EXPECT_EQ(manager.getColor(id), traffic_simulator::TrafficLightColor::NONE);
  }
}

TEST(TrafficLightManager, setArrow)
{
  const auto node = std::make_shared<rclcpp::Node>("setArrow");
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  const auto hdmap_utils_ptr = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
  traffic_simulator::TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>
    manager(hdmap_utils_ptr, node, "map");
  for (const auto & [id, traffic_light] : manager.getTrafficLights()) {
    EXPECT_EQ(manager.getArrow(id), traffic_simulator::TrafficLightArrow::NONE);
    manager.setArrow(id, traffic_simulator::TrafficLightArrow::LEFT);
    EXPECT_EQ(manager.getArrow(id), traffic_simulator::TrafficLightArrow::LEFT);
    manager.setArrow(id, traffic_simulator::TrafficLightArrow::RIGHT);
    EXPECT_EQ(manager.getArrow(id), traffic_simulator::TrafficLightArrow::RIGHT);
    manager.setArrow(id, traffic_simulator::TrafficLightArrow::STRAIGHT);
    EXPECT_EQ(manager.getArrow(id), traffic_simulator::TrafficLightArrow::STRAIGHT);
    manager.setArrow(id, traffic_simulator::TrafficLightArrow::NONE);
    EXPECT_EQ(manager.getArrow(id), traffic_simulator::TrafficLightArrow::NONE);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
