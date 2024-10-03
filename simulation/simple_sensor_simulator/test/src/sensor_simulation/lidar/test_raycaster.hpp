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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_RAYCASTER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_RAYCASTER_HPP_

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator;
using namespace geometry_msgs::msg;

constexpr static double degToRad(double deg) { return deg * M_PI / 180.0; }

class RaycasterTest : public ::testing::Test
{
protected:
  RaycasterTest()
  : raycaster_(std::make_unique<Raycaster>()),
    config_(utils::constructLidarConfiguration("ego", "awf/universe", 0.0, 0.1)),
    origin_(utils::makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
    box_pose_(utils::makePose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
  {
    raycaster_->setDirection(config_);
  }

  std::unique_ptr<Raycaster> raycaster_;
  simulation_api_schema::LidarConfiguration config_;

  const float box_depth_{1.0f};
  const float box_width_{1.0f};
  const float box_height_{1.0f};
  const std::string box_name_{"box"};

  const rclcpp::Time stamp_{0};
  const std::string frame_id_{"frame_id"};

  geometry_msgs::msg::Pose origin_;
  geometry_msgs::msg::Pose box_pose_;
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_RAYCASTER_HPP_
