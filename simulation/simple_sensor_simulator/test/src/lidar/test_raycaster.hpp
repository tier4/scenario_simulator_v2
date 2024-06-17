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

using namespace simple_sensor_simulator;
using namespace geometry_msgs::msg;

constexpr static double degToRad(double deg) { return deg * M_PI / 180.0; }

class RaycasterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    raycaster_ = std::make_unique<Raycaster>();
    configureLidar();

    origin_ =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0))
        .orientation(
          geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
    box_pose_ =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(5.0).y(0.0).z(0.0))
        .orientation(
          geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));
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

private:
  auto configureLidar() -> void
  {
    // Setting vertical angles from -15 to +15 degrees in 2 degree steps
    for (double angle = -15.0; angle <= 15.0; angle += 2.0) {
      config_.add_vertical_angles(degToRad(angle));
    }
    // Setting horizontal resolution to 0.5 degrees
    config_.set_horizontal_resolution(degToRad(0.5));

    raycaster_->setDirection(config_);
  }
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_RAYCASTER_HPP_
