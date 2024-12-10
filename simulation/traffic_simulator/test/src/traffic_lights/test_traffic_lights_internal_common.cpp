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

#include <color_names/color_names.hpp>

#include "../expect_eq_macros.hpp"
#include "common_test_fixtures.hpp"
#include "helper.hpp"

constexpr double eps = 1e-6;

constexpr double timing_eps = 1e-3;

// Frequency can fluctuate a bit due to timing, especially when the machine is under heavy load
// so higher tolerance is needed
constexpr double frequency_eps = 0.5;

// Position in tests is defined with precision of 1cm so such tolerance is needed
constexpr double position_eps = 0.01;

using namespace std::chrono_literals;

using TrafficLightsTypes =
  testing::Types<traffic_simulator::ConventionalTrafficLights, traffic_simulator::V2ITrafficLights>;

TYPED_TEST_SUITE(TrafficLightsInternalTest, TrafficLightsTypes, TrafficLightsNameGenerator);

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
  this->lights->setTrafficLightsState(this->id, stateFromColor("green"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("green"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Green"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("green"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("red"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("red"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Red"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("red"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("yellow"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("Yellow"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("amber"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("yellow"));

  this->lights->setTrafficLightsState(this->id, stateFromColor("white"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromColor("white"));
}

TYPED_TEST(TrafficLightsInternalTest, setTrafficLightsState_status)
{
  this->lights->setTrafficLightsState(this->id, stateFromStatus("solidOn"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOn"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("solidOff"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("Blank"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("none"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("solidOff"));

  this->lights->setTrafficLightsState(this->id, stateFromStatus("flashing"));
  EXPECT_EQ(this->lights->getTrafficLightsComposedState(this->id), stateFromStatus("flashing"));

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
  const auto marker_id = this->id;
  constexpr const char * color_name = "red";
  this->lights->setTrafficLightsState(this->id, stateFromColor(color_name));

  std::vector<visualization_msgs::msg::MarkerArray> markers;
  const auto subscriber =
    this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  // start update with 20Hz frequency and subscribe for 1 second
  this->lights->startUpdate(20.0);
  const auto end = std::chrono::system_clock::now() + 1s;
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  const auto verify_delete_marker =
    [](const visualization_msgs::msg::Marker & marker, const auto & info = "") {
      EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETEALL) << info;
    };

  const auto verify_add_marker = [&color_name, &marker_id](
                                   const visualization_msgs::msg::Marker & marker,
                                   const auto info = "") {
    EXPECT_EQ(marker.ns, "bulb") << info;
    EXPECT_EQ(marker.id, marker_id) << info;
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::SPHERE) << info;
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD) << info;

    EXPECT_POINT_NEAR_STREAM(
      marker.pose.position,
      geometry_msgs::build<geometry_msgs::msg::Point>().x(3770.02).y(73738.34).z(5.80),
      position_eps, info);

    EXPECT_QUATERNION_EQ_STREAM(marker.pose.orientation, geometry_msgs::msg::Quaternion(), info);

    EXPECT_POINT_EQ_STREAM(
      marker.scale, geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.3).y(0.3).z(0.3), info);

    EXPECT_COLOR_RGBA_NEAR_STREAM(marker.color, color_names::makeColorMsg(color_name), eps, info);
  };

  // verify contents of messages
  std::vector<std_msgs::msg::Header> headers;
  for (std::size_t i = 0; i < markers.size(); i += 2) {
    // every marker with an even index (i=0,2,4...) is of type ::DELETEALL, others are ::ADD
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

  // verify 20Hz frequency of ::ADD markers
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(headers.size() - 1) /
    static_cast<double>(getTime(headers.back()) - getTime(headers.front())) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TYPED_TEST(TrafficLightsInternalTest, resetUpdate_publishMarkers)
{
  const auto marker_id = this->id;
  constexpr const char * color_name = "green";
  this->lights->setTrafficLightsState(this->id, stateFromColor(color_name));

  std::vector<visualization_msgs::msg::MarkerArray> markers;
  {
    const auto subscriber =
      this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
        "traffic_light/marker", 10,
        [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
          markers.push_back(*msg_in);
        });

    // start update with 20Hz frequency and subscribe for 0.5 second
    this->lights->startUpdate(20.0);
    const auto first_end = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < first_end) {
      rclcpp::spin_some(this->node_ptr);
    }
  }

  std::vector<visualization_msgs::msg::MarkerArray> markers_reset;
  {
    const auto subscriber =
      this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
        "traffic_light/marker", 10,
        [&markers_reset](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
          markers_reset.push_back(*msg_in);
        });

    // reset update to 10Hz frequency and subscribe for 0.5 second
    this->lights->resetUpdate(10.0);
    const auto second_end = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < second_end) {
      rclcpp::spin_some(this->node_ptr);
    }
  }

  const auto verify_delete_marker =
    [](const visualization_msgs::msg::Marker & marker, const auto & info = "") {
      EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETEALL) << info;
    };

  const auto verify_add_marker = [&color_name, &marker_id](
                                   const visualization_msgs::msg::Marker & marker,
                                   const auto info = "") {
    EXPECT_EQ(marker.ns, "bulb") << info;
    EXPECT_EQ(marker.id, marker_id) << info;
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::SPHERE) << info;
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD) << info;

    EXPECT_POINT_NEAR_STREAM(
      marker.pose.position,
      geometry_msgs::build<geometry_msgs::msg::Point>().x(3770.02).y(73738.34).z(5.80),
      position_eps, info);

    EXPECT_QUATERNION_EQ_STREAM(marker.pose.orientation, geometry_msgs::msg::Quaternion(), info);

    EXPECT_POINT_EQ_STREAM(
      marker.scale, geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.3).y(0.3).z(0.3), info);

    EXPECT_COLOR_RGBA_NEAR_STREAM(marker.color, color_names::makeColorMsg(color_name), eps, info);
  };

  // verify contents of messages - before reset
  std::vector<std_msgs::msg::Header> headers;
  for (std::size_t i = 0; i < markers.size(); i += 2) {
    // every marker with an even index (i=0,2,4...) is of type ::DELETEALL, others are ::ADD
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
  // verify 20Hz frequency of ::ADD markers - before reset
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(headers.size() - 1) /
      static_cast<double>(getTime(headers.back()) - getTime(headers.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }

  // verify contents of messages - after reset
  std::vector<std_msgs::msg::Header> headers_reset;
  for (std::size_t i = 0; i < markers_reset.size(); i += 2) {
    // every marker with an even index (i=0,2,4...) is of type ::DELETEALL, others are ::ADD
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
  // verify 10Hz frequency of ::ADD markers - after reset
  {
    const double expected_frequency = 10.0;
    const double actual_frequency =
      static_cast<double>(headers_reset.size() - 1) /
      static_cast<double>(getTime(headers_reset.back()) - getTime(headers_reset.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
}

TYPED_TEST(TrafficLightsInternalTest, generateTrafficSimulatorV1Msg)
{
  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  const auto msg =
    *traffic_simulator::TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::
      generateMessage(
        this->node_ptr->get_clock()->now(), this->lights->generateUpdateTrafficLightsRequest());

  EXPECT_EQ(msg.traffic_lights.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(msg.traffic_lights.front().traffic_light_bulbs.size(), static_cast<std::size_t>(2));
  EXPECT_EQ(msg.traffic_lights[0].lanelet_way_id, this->id);

  using TrafficLightBulbV1 = traffic_simulator_msgs::msg::TrafficLightBulbV1;
  // we use this order, because signals are parsed in reverse (note: AMBER is equal to yellow)
  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[0].color, TrafficLightBulbV1::AMBER);
  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[0].status, TrafficLightBulbV1::FLASHING);
  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[0].shape, TrafficLightBulbV1::CIRCLE);
  EXPECT_NEAR(msg.traffic_lights[0].traffic_light_bulbs[0].confidence, expected_confidence, eps);

  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[1].color, TrafficLightBulbV1::RED);
  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[1].status, TrafficLightBulbV1::SOLID_ON);
  EXPECT_EQ(msg.traffic_lights[0].traffic_light_bulbs[1].shape, TrafficLightBulbV1::CIRCLE);
  EXPECT_NEAR(msg.traffic_lights[0].traffic_light_bulbs[1].confidence, expected_confidence, eps);
}

TYPED_TEST(TrafficLightsInternalTest, generateAutowarePerceptionTrafficSignalMsg)
{
  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  const auto msg =
    *traffic_simulator::TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::
      generateMessage(
        this->node_ptr->get_clock()->now(), this->lights->generateUpdateTrafficLightsRequest());

  const double expected_time =
    static_cast<double>(getTime(this->node_ptr->get_clock()->now())) * 1e-9;
  const double actual_time = static_cast<double>(getTime(msg.stamp)) * 1e-9;
  EXPECT_NEAR(actual_time, expected_time, timing_eps);

  EXPECT_EQ(msg.signals.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(msg.signals.front().elements.size(), static_cast<std::size_t>(2));

  EXPECT_EQ(msg.signals[0].traffic_signal_id, this->signal_id);

  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;
  // we use this order, because signals are parsed in reverse
  EXPECT_EQ(msg.signals[0].elements[0].color, TrafficSignalElement::AMBER);
  EXPECT_EQ(msg.signals[0].elements[0].status, TrafficSignalElement::FLASHING);
  EXPECT_EQ(msg.signals[0].elements[0].shape, TrafficSignalElement::CIRCLE);
  EXPECT_NEAR(msg.signals[0].elements[0].confidence, expected_confidence, eps);

  EXPECT_EQ(msg.signals[0].elements[1].color, TrafficSignalElement::RED);
  EXPECT_EQ(msg.signals[0].elements[1].status, TrafficSignalElement::SOLID_ON);
  EXPECT_EQ(msg.signals[0].elements[1].shape, TrafficSignalElement::CIRCLE);
  EXPECT_NEAR(msg.signals[0].elements[1].confidence, expected_confidence, eps);
}

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
TYPED_TEST(TrafficLightsInternalTest, generateAutowarePerceptionTrafficLightGroupMsg)
{
  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  const auto msg =
    *traffic_simulator::
      TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::generateMessage(
        this->node_ptr->get_clock()->now(), this->lights->generateUpdateTrafficLightsRequest());

  const double expected_time =
    static_cast<double>(getTime(this->node_ptr->get_clock()->now())) * 1e-9;
  const double actual_time = static_cast<double>(getTime(msg.stamp)) * 1e-9;
  EXPECT_NEAR(actual_time, expected_time, timing_eps);

  EXPECT_EQ(msg.traffic_light_groups.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(msg.traffic_light_groups.front().elements.size(), static_cast<std::size_t>(2));

  EXPECT_EQ(msg.traffic_light_groups[0].traffic_light_group_id, this->signal_id);

  using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;
  // we use this order, because signals are parsed in reverse
  EXPECT_EQ(msg.traffic_light_groups[0].elements[0].color, TrafficLightElement::AMBER);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[0].status, TrafficLightElement::FLASHING);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_NEAR(msg.traffic_light_groups[0].elements[0].confidence, expected_confidence, eps);

  EXPECT_EQ(msg.traffic_light_groups[0].elements[1].color, TrafficLightElement::RED);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[1].status, TrafficLightElement::SOLID_ON);
  EXPECT_EQ(msg.traffic_light_groups[0].elements[1].shape, TrafficLightElement::CIRCLE);
  EXPECT_NEAR(msg.traffic_light_groups[0].elements[1].confidence, expected_confidence, eps);
}
#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
