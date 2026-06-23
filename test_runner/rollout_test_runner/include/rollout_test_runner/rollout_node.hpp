// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#ifndef ROLLOUT_TEST_RUNNER__ROLLOUT_NODE_HPP_
#define ROLLOUT_TEST_RUNNER__ROLLOUT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rollout_test_runner/bag_topic_republisher.hpp>
#include <rollout_test_runner/ego_replay_source.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <traffic_simulator/api/api.hpp>

namespace rollout_test_runner
{
/// Replays ego states from a rosbag while Autoware runs open-loop, then switches to
/// closed-loop vehicle simulation when the switch service is called.
class RolloutNode : public rclcpp::Node
{
public:
  explicit RolloutNode(const rclcpp::NodeOptions & option);

  void start();

private:
  enum class Phase { REPLAY, CLOSED_LOOP };

  void onUpdate();

  void injectEgoStateFromBag(const rclcpp::Time & bag_time);

  auto switchToClosedLoop() -> std::string;  // returns an error message, or empty on success

  auto configure() -> traffic_simulator::Configuration;

  EgoReplaySource ego_replay_source_;

  const double bag_anchor_seconds_;

  traffic_simulator::API api_;

  BagTopicRepublisher passthrough_;

  Phase phase_ = Phase::REPLAY;

  std::string on_bag_end_;

  double auto_switch_time_;

  double global_timeout_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr switch_service_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  static constexpr auto ego_name_ = "ego";
};
}  // namespace rollout_test_runner

#endif  // ROLLOUT_TEST_RUNNER__ROLLOUT_NODE_HPP_
