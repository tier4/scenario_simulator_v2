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

#include "../expect_eq_macros.hpp"
#include "common_test_fixtures.hpp"
#include "helper.hpp"

constexpr double eps = 1e-6;

// Frequency can fluctuate a bit due to timing, especially when the machine is under heavy load, so
// higher tolerance is needed
constexpr double frequency_eps = 0.5;

using namespace std::chrono_literals;

using V2ITrafficLightsTest = TrafficLightsInternalTest<traffic_simulator::V2ITrafficLights>;

TEST_F(V2ITrafficLightsTest, startUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  std::vector<TrafficSignalArray> signals;
  const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
    "/perception/traffic_light_recognition/external/traffic_signals", 10,
    [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  // start update with 20Hz frequency and subscribe for 1 second
  this->lights->startUpdate(20.0);
  const auto end = std::chrono::system_clock::now() + 1s;
  while (std::chrono::system_clock::now() < end) {
    this->executor.spin_some();
  }

  // verify contents of messages
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }

  // verify 20Hz frequency
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(stamps.size() - 1) /
    static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TEST_F(V2ITrafficLightsTest, startUpdate_publishSignalsLegacy)
{
  using namespace autoware_perception_msgs::msg;

  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  std::vector<TrafficSignalArray> signals;
  const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
    "/v2x/traffic_signals", 10,
    [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

  // start update with 20Hz frequency and subscribe for 1 second
  this->lights->startUpdate(20.0);
  const auto end = std::chrono::system_clock::now() + 1s;
  while (std::chrono::system_clock::now() < end) {
    this->executor.spin_some();
  }

  // verify contents of messages
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }

  // verify 20Hz frequency
  const double expected_frequency = 20.0;
  const double actual_frequency =
    static_cast<double>(stamps.size() - 1) /
    static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

TEST_F(V2ITrafficLightsTest, resetUpdate_publishSignals)
{
  using namespace autoware_perception_msgs::msg;

  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  std::vector<TrafficSignalArray> signals;
  {
    const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

    // start update with 20Hz frequency and subscribe for 0.5 second
    this->lights->startUpdate(20.0);
    const auto end = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < end) {
      this->executor.spin_some();
    }
  }

  std::vector<TrafficSignalArray> signals_reset;
  {
    const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
      "/perception/traffic_light_recognition/external/traffic_signals", 10,
      [&signals_reset](const TrafficSignalArray::SharedPtr msg_in) {
        signals_reset.push_back(*msg_in);
      });

    // reset update to 10Hz frequency and subscribe for 0.5 second
    this->lights->resetUpdate(10.0);
    const auto end_reset = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < end_reset) {
      this->executor.spin_some();
    }
  }

  // verify contents of messages - before reset
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }
  // verify 20Hz frequency - before reset
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(stamps.size() - 1) /
      static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }

  // verify contents of messages - after reset
  std::vector<builtin_interfaces::msg::Time> stamps_reset;
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }
  // verify 10Hz frequency - after reset
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

  constexpr double expected_confidence{0.7};
  this->lights->setTrafficLightsState(this->id, "red solidOn circle, yellow flashing circle");
  this->lights->setTrafficLightsConfidence(this->id, expected_confidence);

  std::vector<TrafficSignalArray> signals;
  {
    const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
      "/v2x/traffic_signals", 10,
      [&signals](const TrafficSignalArray::SharedPtr msg_in) { signals.push_back(*msg_in); });

    // start update with 20Hz frequency and subscribe for 0.5 second
    this->lights->startUpdate(20.0);
    const auto end = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < end) {
      this->executor.spin_some();
    }
  }

  std::vector<TrafficSignalArray> signals_reset;
  {
    const auto subscriber = this->node_ptr->create_subscription<TrafficSignalArray>(
      "/v2x/traffic_signals", 10, [&signals_reset](const TrafficSignalArray::SharedPtr msg_in) {
        signals_reset.push_back(*msg_in);
      });

    // reset update to 10Hz frequency and subscribe for 0.5 second
    this->lights->resetUpdate(10.0);
    const auto end_reset = std::chrono::system_clock::now() + 0.5s;
    while (std::chrono::system_clock::now() < end_reset) {
      this->executor.spin_some();
    }
  }

  // verify contents of messages - before reset
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }
  // verify 20Hz frequency - before reset
  {
    const double expected_frequency = 20.0;
    const double actual_frequency =
      static_cast<double>(stamps.size() - 1) /
      static_cast<double>(getTime(stamps.back()) - getTime(stamps.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }

  // verify contents of messages - after reset
  std::vector<builtin_interfaces::msg::Time> stamps_reset;
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
    EXPECT_NEAR(one_message[0].elements[0].confidence, expected_confidence, eps);

    EXPECT_EQ(one_message[0].elements[1].color, TrafficSignalElement::RED) << info;
    EXPECT_EQ(one_message[0].elements[1].shape, TrafficSignalElement::CIRCLE) << info;
    EXPECT_EQ(one_message[0].elements[1].status, TrafficSignalElement::SOLID_ON) << info;
    EXPECT_NEAR(one_message[0].elements[1].confidence, expected_confidence, eps);
  }
  // verify 10Hz frequency - after reset
  {
    const double expected_frequency = 10.0;
    const double actual_frequency =
      static_cast<double>(stamps_reset.size() - 1) /
      static_cast<double>(getTime(stamps_reset.back()) - getTime(stamps_reset.front())) * 1e+9;
    EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
  }
}
