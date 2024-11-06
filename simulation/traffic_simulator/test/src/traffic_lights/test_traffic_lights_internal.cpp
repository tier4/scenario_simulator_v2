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
#include <std_msgs/msg/header.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

#include "../expect_eq_macros.hpp"
#include "helper.hpp"

constexpr double timing_eps = 1e-3;
constexpr double frequency_eps = 0.5;

template <typename TrafficLightsT>
class TrafficLightsInternalTest : public testing::Test
{
public:
  const lanelet::Id id = 34836;
  const lanelet::Id signal_id = 34806;

  const rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("TrafficLightsInternalTest");

  const std::string path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                           "/map/standard_map/lanelet2_map.osm";

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr =
    std::make_shared<hdmap_utils::HdMapUtils>(
      path, geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
              .latitude(35.61836750154)
              .longitude(139.78066608243)
              .altitude(0.0));

  std::unique_ptr<TrafficLightsT> lights;

  TrafficLightsInternalTest()
  : lights([this] {
      if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::ConventionalTrafficLights>)
        return std::make_unique<TrafficLightsT>(node_ptr, hdmap_utils_ptr);
      else if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::V2ITrafficLights>)
        return std::make_unique<TrafficLightsT>(node_ptr, hdmap_utils_ptr, "awf/universe");
      else
        throw std::runtime_error("Given TrafficLights type is not supported");
    }())
  {
  }
};

// Alias for declaring types in typed tests
using TrafficLightsTypes =
  testing::Types<traffic_simulator::ConventionalTrafficLights, traffic_simulator::V2ITrafficLights>;

// Test name generator
class TrafficLightsNameGenerator
{
public:
  template <typename TrafficLightsT>
  static std::string GetName(int)
  {
    if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::ConventionalTrafficLights>)
      return "ConventionalTrafficLights";
    if constexpr (std::is_same_v<TrafficLightsT, traffic_simulator::V2ITrafficLights>)
      return "V2ITrafficLights";
  }
};

// Declare typed test suite
TYPED_TEST_SUITE(TrafficLightsInternalTest, TrafficLightsTypes, TrafficLightsNameGenerator);

// Define V2I type for use in tests with V2I traffic lights only
using V2ITrafficLightsTest = TrafficLightsInternalTest<traffic_simulator::V2ITrafficLights>;

TYPED_TEST(TrafficLightsInternalTest, setTrafficLightsColor)
{
  using Color = traffic_simulator::TrafficLight::Color;

  this->lights->setTrafficLightsColor(this->id, Color::green);
  EXPECT_FALSE(
    this->lights->getTrafficLightsComposedState(this->id).find("green") == std::string::npos);

  this->lights->setTrafficLightsColor(this->id, Color::yellow);
  EXPECT_FALSE(
    this->lights->getTrafficLightsComposedState(this->id).find("yellow") == std::string::npos);

  this->lights->setTrafficLightsColor(this->id, Color::red);
  EXPECT_FALSE(
    this->lights->getTrafficLightsComposedState(this->id).find("red") == std::string::npos);

  this->lights->setTrafficLightsColor(this->id, Color::white);
  EXPECT_FALSE(
    this->lights->getTrafficLightsComposedState(this->id).find("white") == std::string::npos);
}

TYPED_TEST(TrafficLightsInternalTest, setTrafficLightsState_color)
{
  // green
  this->lights->setTrafficLightsState(this->id, stateFromColor("green"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("green"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Green"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("green"));

  // red
  this->lights->setTrafficLightsState(this->id, stateFromColor("red"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("red"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Red"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("red"));

  // yellow
  this->lights->setTrafficLightsState(this->id, stateFromColor("yellow"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Yellow"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("amber"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  // white
  this->lights->setTrafficLightsState(this->id, stateFromColor("white"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("white"));
}

TYPED_TEST(TrafficLightsInternalTest, setTrafficLightsState_status)
{
  // solid on
  this->lights->setTrafficLightsState(this->id, stateFromStatus("solidOn"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOn"));

  // solid off
  this->lights->setTrafficLightsState(this->id, stateFromStatus("solidOff"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("Blank"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("none"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  // flashing
  this->lights->setTrafficLightsState(this->id, stateFromStatus("flashing"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("flashing"));

  // unknown
  this->lights->setTrafficLightsState(this->id, stateFromStatus("unknown"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("unknown"));
}

TYPED_TEST(TrafficLightsInternalTest, setTrafficLightsState_shape)
{
  this->lights->setTrafficLightsState(this->id, stateFromShape("circle"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("circle"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("cross"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("cross"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("left"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("left"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("down"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("down"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("up"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("up"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("right"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("right"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("lowerLeft"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("lowerLeft"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("upperLeft"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("upperLeft"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("lowerRight"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("lowerRight"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("upperRight"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("upperRight"));

  this->lights->setTrafficLightsState(this->id, stateFromShape("straight"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromShape("up"));
}

TYPED_TEST(TrafficLightsInternalTest, isAnyTrafficLightChanged)
{
  EXPECT_TRUE(this->lights->isAnyTrafficLightChanged());
}

TYPED_TEST(TrafficLightsInternalTest, isRequiredStopTrafficLightState)
{
  this->lights->setTrafficLightsState(this->id, stateFromColor("green"));
  EXPECT_FALSE(this->lights->isRequiredStopTrafficLightState(this->id));

  this->lights->setTrafficLightsState(this->id, stateFromColor("yellow"));
  EXPECT_TRUE(this->lights->isRequiredStopTrafficLightState(this->id));

  this->lights->setTrafficLightsState(this->id, "yellow flashing circle");
  EXPECT_FALSE(this->lights->isRequiredStopTrafficLightState(this->id));

  this->lights->setTrafficLightsState(this->id, stateFromColor("red"));
  EXPECT_TRUE(this->lights->isRequiredStopTrafficLightState(this->id));
}

TYPED_TEST(TrafficLightsInternalTest, compareTrafficLightsState)
{
  this->lights->setTrafficLightsState(this->id, stateFromColor("green"));
  EXPECT_TRUE(this->lights->compareTrafficLightsState(this->id, stateFromColor("green")));
  EXPECT_TRUE(this->lights->compareTrafficLightsState(this->id, stateFromColor("Green")));
  EXPECT_FALSE(this->lights->compareTrafficLightsState(this->id, stateFromColor("yellow")));
  EXPECT_FALSE(this->lights->compareTrafficLightsState(this->id, stateFromColor("red")));
  EXPECT_FALSE(this->lights->compareTrafficLightsState(this->id, stateFromColor("amber")));
  EXPECT_FALSE(this->lights->compareTrafficLightsState(this->id, stateFromColor("white")));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("Blank"));
  EXPECT_TRUE(this->lights->compareTrafficLightsState(this->id, stateFromStatus("solidOff")));
  EXPECT_TRUE(this->lights->compareTrafficLightsState(this->id, stateFromStatus("Blank")));
  EXPECT_TRUE(this->lights->compareTrafficLightsState(this->id, stateFromStatus("none")));
}

TYPED_TEST(TrafficLightsInternalTest, startUpdate_publishMarkers)
{
  this->lights->setTrafficLightsState(this->id, stateFromColor("red"));

  std::vector<visualization_msgs::msg::MarkerArray> markers;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber =
    this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  const auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1005);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
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
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(headers.size() - 1) /
    static_cast<double>(getTime(headers.back()) - getTime(headers.front())) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TYPED_TEST(TrafficLightsInternalTest, resetUpdate_publishMarkers)
{
  this->lights->setTrafficLightsState(this->id, stateFromColor("green"));

  std::vector<visualization_msgs::msg::MarkerArray> markers, markers_reset;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber =
    this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  subscriber = this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
    "traffic_light/marker", 10,
    [&markers_reset](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
      markers_reset.push_back(*msg_in);
    });

  this->lights->resetUpdate(10.0);
  end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
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

  std::vector<std_msgs::msg::Header> headers, headers_reset;

  // verify
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
  for (std::size_t i = 0; i < markers_reset.size(); i += 2) {
    {
      const auto & one_marker = markers_reset[i].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_delete_marker(one_marker[0], "marker " + std::to_string(i));
    }
    {
      const auto & one_marker = markers_reset[i + 1].markers;
      EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));
      verify_add_marker(one_marker[0], "marker " + std::to_string(i + 1));

      headers_reset.push_back(one_marker[0].header);
    }
  }

  // verify message timing
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(headers.size() - 1) /
      static_cast<double>(getTime(headers.back()) - getTime(headers.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
  {
    const double expected_frequency = 10.0;
    const double actual_frequency =
      static_cast<double>(headers_reset.size() - 1) /
      static_cast<double>(getTime(headers_reset.back()) - getTime(headers_reset.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
}

TYPED_TEST(TrafficLightsInternalTest, generateAutowarePerceptionMsg)
{
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  const auto msg = traffic_simulator::TrafficLightPublisherBase::generateAutowarePerceptionMsg(
    this->node_ptr->get_clock()->now(), *this->lights);

  const double expected_time =
    static_cast<double>(getTime(this->node_ptr->get_clock()->now())) * 1e-9;
  const double actual_time = static_cast<double>(getTime(msg.stamp)) * 1e-9;
  EXPECT_NEAR(actual_time, expected_time, timing_eps);

  EXPECT_EQ(msg.signals.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(msg.signals.front().elements.size(), static_cast<std::size_t>(2));

  EXPECT_EQ(msg.signals[0].traffic_signal_id, this->signal_id);

  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;
  // signals are parsed in reverse order
  EXPECT_EQ(msg.signals[0].elements[0].color, TrafficSignalElement::AMBER);
  EXPECT_EQ(msg.signals[0].elements[0].status, TrafficSignalElement::FLASHING);
  EXPECT_EQ(msg.signals[0].elements[0].shape, TrafficSignalElement::CIRCLE);
  EXPECT_NEAR(msg.signals[0].elements[0].confidence, 0.7, 1e-6);

  EXPECT_EQ(msg.signals[0].elements[1].color, TrafficSignalElement::RED);
  EXPECT_EQ(msg.signals[0].elements[1].status, TrafficSignalElement::SOLID_ON);
  EXPECT_EQ(msg.signals[0].elements[1].shape, TrafficSignalElement::CIRCLE);
  EXPECT_NEAR(msg.signals[0].elements[1].confidence, 0.7, 1e-6);
}

TYPED_TEST(TrafficLightsInternalTest, generateAutowareAutoPerceptionMsg)
{
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  const auto msg = traffic_simulator::TrafficLightPublisherBase::generateAutowareAutoPerceptionMsg(
    this->node_ptr->get_clock()->now(), *this->lights);

  const double expected_time =
    static_cast<double>(getTime(this->node_ptr->get_clock()->now())) * 1e-9;
  const double actual_time = static_cast<double>(getTime(msg.header)) * 1e-9;
  EXPECT_NEAR(actual_time, expected_time, timing_eps);

  EXPECT_EQ(msg.signals.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(msg.signals.front().lights.size(), static_cast<std::size_t>(2));

  EXPECT_EQ(msg.header.frame_id, "camera_link");
  EXPECT_EQ(msg.signals[0].map_primitive_id, this->id);

  using TrafficLight = autoware_auto_perception_msgs::msg::TrafficLight;
  // signals are parsed in reverse order
  EXPECT_EQ(msg.signals[0].lights[0].color, TrafficLight::AMBER);
  EXPECT_EQ(msg.signals[0].lights[0].status, TrafficLight::FLASHING);
  EXPECT_EQ(msg.signals[0].lights[0].shape, TrafficLight::CIRCLE);
  EXPECT_NEAR(msg.signals[0].lights[0].confidence, 0.7, 1e-6);

  EXPECT_EQ(msg.signals[0].lights[1].color, TrafficLight::RED);
  EXPECT_EQ(msg.signals[0].lights[1].status, TrafficLight::SOLID_ON);
  EXPECT_EQ(msg.signals[0].lights[1].shape, TrafficLight::CIRCLE);
  EXPECT_NEAR(msg.signals[0].lights[1].confidence, 0.7, 1e-6);
}

TEST_F(V2ITrafficLightsTest, startUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficSignalArray> signals;

  rclcpp::Subscription<TrafficSignalArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficSignalArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1005);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  // verify message timing
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(stamps.size() - 1) /
    static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;

  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TEST_F(V2ITrafficLightsTest, startUpdate_publishSignalsLegacy)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficSignalArray> signals;

  rclcpp::Subscription<TrafficSignalArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficSignalArray>(
      "/v2x/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  const auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1005);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  // verify message timing
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(stamps.size() - 1) /
    static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TEST_F(V2ITrafficLightsTest, resetUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficSignalArray> signals, signals_reset;

  rclcpp::Subscription<TrafficSignalArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficSignalArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
    "/perception/traffic_light_recognition/external/traffic_signals", 10,
    [&signals_reset](const TrafficSignalArray::SharedPtr msg_in) {
      signals_reset.push_back(*msg_in);
    });

  this->lights->resetUpdate(10.0);
  end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps, stamps_reset;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  for (std::size_t i = 0; i < signals_reset.size(); ++i) {
    stamps_reset.push_back(signals_reset[i].stamp);

    const auto & one_message = signals_reset[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  // verify message timing
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(stamps.size() - 1) /
      static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
  {
    const double expected_frequency = 10.0;
    const double actual_frequency =
      static_cast<double>(stamps_reset.size() - 1) /
      static_cast<double>(getTime(stamps_reset.back()) - getTime(stamps_reset.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
}

TEST_F(V2ITrafficLightsTest, resetUpdate_publishSignalsLegacy)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficSignalArray> signals, signals_reset;

  rclcpp::Subscription<TrafficSignalArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficSignalArray>(
      "/v2x/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
    "/v2x/traffic_signals", 10, [&signals_reset](const TrafficSignalArray::SharedPtr msg_in) {
      signals_reset.push_back(*msg_in);
    });

  this->lights->resetUpdate(10.0);
  end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps, stamps_reset;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  for (std::size_t i = 0; i < signals_reset.size(); ++i) {
    stamps_reset.push_back(signals_reset[i].stamp);

    const auto & one_message = signals_reset[i].signals;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_signal_id, signal_id) << info;

    EXPECT_EQ(one_message[0].elements.size(), static_cast<std::size_t>(2)) << info;

    EXPECT_EQ(one_message[0].elements[0].color, TrafficSignalElement::AMBER) << info;
    EXPECT_EQ(one_message[0].elements[0].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[0].status, TrafficSignalElement::FLASHING) << info;
    EXPECT_NEAR(one_message[0].elements[0].confidence, 0.7, 1e-6);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, 0.7, 1e-6);
  }

  // verify message timing
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(stamps.size() - 1) /
      static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
  {
    const double expected_frequency = 10.0;
    const double actual_frequency =
      static_cast<double>(stamps_reset.size() - 1) /
      static_cast<double>(getTime(stamps_reset.back()) - getTime(stamps_reset.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
}
