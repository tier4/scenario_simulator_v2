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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__GOAL_REACHED_METRIC_H
#define RANDOM_TEST_RUNNER__GOAL_REACHED_METRIC_H

#include <iostream>
#include <optional>

#include "geometry_msgs/msg/pose.hpp"
#include "random_test_runner/data_types.hpp"
#include "traffic_simulator_msgs/msg/entity_status.hpp"

class GoalReachedMetric
{
public:
  bool isGoalReached(const traffic_simulator::CanonicalizedEntityStatus & status)
  {
    if (!goal_pose_) {
      return false;
    }

    geometry_msgs::msg::Point current_position = status.getMapPose().position;
    geometry_msgs::msg::Point goal_position = goal_pose_->position;

    if (distance(current_position, goal_position) < goal_reaching_accuracy_threshold_) {
      return true;
    }

    return false;
  }

  void setGoal(const geometry_msgs::msg::Pose & goal_pose) { goal_pose_ = goal_pose; }

private:
  const double goal_reaching_accuracy_threshold_ = 3.0;

  std::optional<geometry_msgs::msg::Pose> goal_pose_;

  static double distance(
    const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
  {
    geometry_msgs::msg::Point diff;
    diff.x = lhs.x - rhs.x;
    diff.y = lhs.y - rhs.y;
    diff.z = lhs.z - rhs.z;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
  }
};

#endif  // RANDOM_TEST_RUNNER__GOAL_REACHED_METRIC_H
