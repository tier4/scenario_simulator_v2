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
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

#include "../expect_eq_macros.hpp"

/// Helper functions
// clang-format off
auto stateFromColor(const std::string & color)   -> std::string { return color + " solidOn circle"; }
auto stateFromStatus(const std::string & status) -> std::string { return "green " + status + " circle"; }
auto stateFromShape(const std::string & shape)   -> std::string { return "green solidOn " + shape; }
// clang-format on
auto getTime(const std_msgs::msg::Header & header) -> unsigned int
{
  static constexpr unsigned int nanosecond_multiplier = static_cast<unsigned int>(1e+9);
  return static_cast<unsigned int>(header.stamp.sec) * nanosecond_multiplier +
         static_cast<unsigned int>(header.stamp.nanosec);
}

class ConventionalTrafficLightsTest : public testing::Test
{
protected:
  const lanelet::Id id = 34836;

  const rclcpp::Node::SharedPtr node_ptr =
    std::make_shared<rclcpp::Node>("ConventionalTrafficLightsTest");

  const std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr =
    std::make_shared<hdmap_utils::HdMapUtils>(
      path, geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
              .latitude(35.61836750154)
              .longitude(139.78066608243)
              .altitude(0.0));

  traffic_simulator::ConventionalTrafficLights lights =
    traffic_simulator::ConventionalTrafficLights(node_ptr, hdmap_utils_ptr);
};

TEST_F(ConventionalTrafficLightsTest, setTrafficLightsColor)
{
  using Color = traffic_simulator::TrafficLight::Color;

  lights.setTrafficLightsColor(id, Color::green);
  EXPECT_FALSE(lights.getTrafficLightsComposedState(id).find("green") == std::string::npos);

  lights.setTrafficLightsColor(id, Color::yellow);
  EXPECT_FALSE(lights.getTrafficLightsComposedState(id).find("yellow") == std::string::npos);

  lights.setTrafficLightsColor(id, Color::red);
  EXPECT_FALSE(lights.getTrafficLightsComposedState(id).find("red") == std::string::npos);

  lights.setTrafficLightsColor(id, Color::white);
  EXPECT_FALSE(lights.getTrafficLightsComposedState(id).find("white") == std::string::npos);
}

TEST_F(ConventionalTrafficLightsTest, setTrafficLightsState_color)
{
  // green
  lights.setTrafficLightsState(id, stateFromColor("green"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("green"));

  lights.setTrafficLightsState(id, stateFromColor("Green"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("green"));

  // red
  lights.setTrafficLightsState(id, stateFromColor("red"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("red"));

  lights.setTrafficLightsState(id, stateFromColor("Red"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("red"));

  // yellow
  lights.setTrafficLightsState(id, stateFromColor("yellow"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("yellow"));

  lights.setTrafficLightsState(id, stateFromColor("Yellow"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("yellow"));

  lights.setTrafficLightsState(id, stateFromColor("amber"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("yellow"));

  // white
  lights.setTrafficLightsState(id, stateFromColor("white"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromColor("white"));
}

TEST_F(ConventionalTrafficLightsTest, setTrafficLightsState_status)
{
  // solid on
  lights.setTrafficLightsState(id, stateFromStatus("solidOn"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("solidOn"));

  // solid off
  lights.setTrafficLightsState(id, stateFromStatus("solidOff"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("solidOff"));

  lights.setTrafficLightsState(id, stateFromStatus("Blank"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("solidOff"));

  lights.setTrafficLightsState(id, stateFromStatus("none"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("solidOff"));

  // flashing
  lights.setTrafficLightsState(id, stateFromStatus("flashing"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("flashing"));

  // unknown
  lights.setTrafficLightsState(id, stateFromStatus("unknown"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromStatus("unknown"));
}

TEST_F(ConventionalTrafficLightsTest, setTrafficLightsState_shape)
{
  lights.setTrafficLightsState(id, stateFromShape("circle"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("circle"));

  lights.setTrafficLightsState(id, stateFromShape("cross"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("cross"));

  lights.setTrafficLightsState(id, stateFromShape("left"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("left"));

  lights.setTrafficLightsState(id, stateFromShape("down"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("down"));

  lights.setTrafficLightsState(id, stateFromShape("up"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("up"));

  lights.setTrafficLightsState(id, stateFromShape("right"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("right"));

  lights.setTrafficLightsState(id, stateFromShape("lowerLeft"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("lowerLeft"));

  lights.setTrafficLightsState(id, stateFromShape("upperLeft"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("upperLeft"));

  lights.setTrafficLightsState(id, stateFromShape("lowerRight"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("lowerRight"));

  lights.setTrafficLightsState(id, stateFromShape("upperRight"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("upperRight"));

  lights.setTrafficLightsState(id, stateFromShape("straight"));
  EXPECT_EQ(lights.getTrafficLightsComposedState(id), stateFromShape("up"));
}

TEST_F(ConventionalTrafficLightsTest, isAnyTrafficLightChanged)
{
  EXPECT_TRUE(lights.isAnyTrafficLightChanged());
}

TEST_F(ConventionalTrafficLightsTest, isRequiredStopTrafficLightState)
{
  lights.setTrafficLightsState(id, stateFromColor("green"));
  EXPECT_FALSE(lights.isRequiredStopTrafficLightState(id));

  lights.setTrafficLightsState(id, stateFromColor("yellow"));
  EXPECT_TRUE(lights.isRequiredStopTrafficLightState(id));

  lights.setTrafficLightsState(id, "yellow flashing circle");
  EXPECT_FALSE(lights.isRequiredStopTrafficLightState(id));

  lights.setTrafficLightsState(id, stateFromColor("red"));
  EXPECT_TRUE(lights.isRequiredStopTrafficLightState(id));
}

TEST_F(ConventionalTrafficLightsTest, compareTrafficLightsState)
{
  lights.setTrafficLightsState(id, stateFromColor("green"));
  EXPECT_TRUE(lights.compareTrafficLightsState(id, stateFromColor("green")));
  EXPECT_TRUE(lights.compareTrafficLightsState(id, stateFromColor("Green")));
  EXPECT_FALSE(lights.compareTrafficLightsState(id, stateFromColor("yellow")));
  EXPECT_FALSE(lights.compareTrafficLightsState(id, stateFromColor("red")));
  EXPECT_FALSE(lights.compareTrafficLightsState(id, stateFromColor("amber")));
  EXPECT_FALSE(lights.compareTrafficLightsState(id, stateFromColor("white")));

  lights.setTrafficLightsState(id, stateFromStatus("Blank"));
  EXPECT_TRUE(lights.compareTrafficLightsState(id, stateFromStatus("solidOff")));
  EXPECT_TRUE(lights.compareTrafficLightsState(id, stateFromStatus("Blank")));
  EXPECT_TRUE(lights.compareTrafficLightsState(id, stateFromStatus("none")));
}

TEST_F(ConventionalTrafficLightsTest, startUpdate)
{
  lights.setTrafficLightsState(id, stateFromColor("red"));

  std::vector<visualization_msgs::msg::MarkerArray> markers;

  lights.startUpdate(20.0);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber =
    node_ptr->create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  // spin for 1 second
  const auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1001);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(node_ptr);
  }

  // verify lambdas
  const auto verify_delete_marker =
    [](const visualization_msgs::msg::Marker & marker, const auto & info = "") {
      EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETEALL) << info;
    };

  const auto verify_add_marker = [](
                                   const visualization_msgs::msg::Marker & marker,
                                   const auto info = "") {
    EXPECT_EQ(marker.ns, "bulb") << info;
    EXPECT_EQ(marker.id, 34836) << info;
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::SPHERE) << info;
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD) << info;

    EXPECT_POINT_NEAR_STREAM(
      marker.pose.position,
      geometry_msgs::build<geometry_msgs::msg::Point>().x(3770.02).y(73738.34).z(5.80), 0.01, info);

    EXPECT_QUATERNION_EQ_STREAM(marker.pose.orientation, geometry_msgs::msg::Quaternion(), info);

    EXPECT_POINT_EQ_STREAM(
      marker.scale, geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.3).y(0.3).z(0.3), info);

    EXPECT_COLOR_RGBA_EQ_STREAM(
      marker.color, std_msgs::build<std_msgs::msg::ColorRGBA>().r(1.0).g(0.0).b(0.0).a(1.0), info);
  };

  std::vector<std_msgs::msg::Header> headers;

  // verify
  EXPECT_EQ(markers.size(), static_cast<std::size_t>(40));
  for (std::size_t i = 0; i < markers.size(); i += 2) {
    {
      const auto & one_marker = markers[i].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_delete_marker(one_marker[0], "marker " + std::to_string(i));
    }
    {
      const auto & one_marker = markers[i + 1].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_add_marker(one_marker[0], "marker " + std::to_string(i + 1));

      headers.push_back(one_marker[0].header);
    }
  }

  // verify message timing
  const double expected_time = 1.0 / 20.0;
  const double actual_time =
    (static_cast<double>(getTime(headers.back()) - getTime(headers.front())) /
     static_cast<double>(headers.size() - 1)) *
    1e-9;
  EXPECT_NEAR(actual_time, expected_time, 1e-4);
}

TEST_F(ConventionalTrafficLightsTest, resetUpdate)
{
  lights.setTrafficLightsState(id, stateFromColor("green"));

  std::vector<visualization_msgs::msg::MarkerArray> markers;

  lights.startUpdate(20.0);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber =
    node_ptr->create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(501);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(node_ptr);
  }
  lights.resetUpdate(4.0);
  end = std::chrono::system_clock::now() + std::chrono::milliseconds(501);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(node_ptr);
  }

  // verify lambdas
  const auto verify_delete_marker =
    [](const visualization_msgs::msg::Marker & marker, const auto & info = "") {
      EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETEALL) << info;
    };

  const auto verify_add_marker = [](
                                   const visualization_msgs::msg::Marker & marker,
                                   const auto info = "") {
    EXPECT_EQ(marker.ns, "bulb") << info;
    EXPECT_EQ(marker.id, 34836) << info;
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::SPHERE) << info;
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD) << info;

    EXPECT_POINT_NEAR_STREAM(
      marker.pose.position,
      geometry_msgs::build<geometry_msgs::msg::Point>().x(3770.02).y(73738.34).z(5.80), 0.01, info);

    EXPECT_QUATERNION_EQ_STREAM(marker.pose.orientation, geometry_msgs::msg::Quaternion(), info);

    EXPECT_POINT_EQ_STREAM(
      marker.scale, geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.3).y(0.3).z(0.3), info);

    EXPECT_COLOR_RGBA_NEAR_STREAM(
      marker.color, std_msgs::build<std_msgs::msg::ColorRGBA>().r(0.0).g(0.5).b(0.0).a(1.0), 0.01,
      info);
  };

  std::vector<std_msgs::msg::Header> headers;

  // verify
  EXPECT_EQ(markers.size(), static_cast<std::size_t>(24));
  for (std::size_t i = 0; i < markers.size(); i += 2) {
    {
      const auto & one_marker = markers[i].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_delete_marker(one_marker[0], "marker " + std::to_string(i));
    }
    {
      const auto & one_marker = markers[i + 1].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_add_marker(one_marker[0], "marker " + std::to_string(i + 1));

      headers.push_back(one_marker[0].header);
    }
  }

  // verify message timing
  {
    const double expected_time = 1.0 / 20.0;
    const double actual_time =
      (static_cast<double>(getTime(headers.at(9)) - getTime(headers.front())) /
       static_cast<double>(10 - 1)) *
      1e-9;
    EXPECT_NEAR(actual_time, expected_time, 1e-4);
  }
  {
    const double expected_time = 1.0 / 4.0;
    const double actual_time =
      (static_cast<double>(getTime(headers.back()) - getTime(*(headers.rbegin() + 1))) /
       static_cast<double>(headers.size() - 10 - 1)) *
      1e-9;
    EXPECT_NEAR(actual_time, expected_time, 1e-4);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
