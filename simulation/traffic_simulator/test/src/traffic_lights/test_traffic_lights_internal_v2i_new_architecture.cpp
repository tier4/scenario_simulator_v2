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

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)

/**
 * These tests are almost identical to tests in `test_traffic_lights_internal_v2i.cpp` but they use
 * autoware_perception_msgs::msg::TrafficLightGroupArray instead of
 * autoware_perception_msgs::msg::TrafficSignalArray
 */

#include <gtest/gtest.h>

#include "../expect_eq_macros.hpp"
#include "common_test_fixtures.hpp"
#include "helper.hpp"

constexpr double timing_eps = 1e-3;
constexpr double frequency_eps = 0.5;

// Define V2I type for use in tests with V2I traffic lights only
using V2ITrafficLightsTestNewArchitecture =
  TrafficLightsInternalTestNewArchitecture<traffic_simulator::V2ITrafficLights>;

TEST_F(V2ITrafficLightsTestNewArchitecture, startUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficLightGroupArray> signals;

  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals](const TrafficLightGroupArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1005);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

TEST_F(V2ITrafficLightsTestNewArchitecture, startUpdate_publishSignalsLegacy)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficLightGroupArray> signals;

  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficLightGroupArray>(
      "/v2x/traffic_signals", 10,
      [&signals](const TrafficLightGroupArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  const auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1005);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<builtin_interfaces::msg::Time> stamps;

  for (std::size_t i = 0; i < signals.size(); ++i) {
    stamps.push_back(signals[i].stamp);

    const auto & one_message = signals[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

TEST_F(V2ITrafficLightsTestNewArchitecture, resetUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficLightGroupArray> signals, signals_reset;

  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals](const TrafficLightGroupArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  subscriber = this->node_ptr->create_subscription<TrafficLightGroupArray>(
    "/perception/traffic_light_recognition/external/traffic_signals", 10,
    [&signals_reset](const TrafficLightGroupArray::SharedPtr msg_in) {
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

    const auto & one_message = signals[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

    const auto & one_message = signals_reset[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

TEST_F(V2ITrafficLightsTestNewArchitecture, resetUpdate_publishSignalsLegacy)
{
  using namespace autoware_perception_msgs::msg;

  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, 0.7);

  std::vector<TrafficLightGroupArray> signals, signals_reset;

  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr subscriber =
    this->node_ptr->create_subscription<TrafficLightGroupArray>(
      "/v2x/traffic_signals", 10,
      [&signals](const TrafficLightGroupArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  this->lights->startUpdate(20.0);

  // spin for 1 second
  auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(505);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  subscriber = this->node_ptr->create_subscription<TrafficLightGroupArray>(
    "/v2x/traffic_signals", 10, [&signals_reset](const TrafficLightGroupArray::SharedPtr msg_in) {
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

    const auto & one_message = signals[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

    const auto & one_message = signals_reset[i].traffic_light_groups;
    std::string info = "signals message number " + std::to_string(i);

    EXPECT_EQ(one_message.size(), static_cast<std::size_t>(1)) << info;
    EXPECT_EQ(one_message[0].traffic_light_group_id, signal_id) << info;

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

#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
