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

#include "test_lidar_sensor.hpp"

/**
 * @note Test function behavior when called on a scene without Ego entity added - the goal is to
 * test error throwing.
 */
TEST_F(LidarSensorTest, update_noEgo)
{
  status_.clear();  // Remove ego
  EXPECT_THROW(
    lidar_->update(current_simulation_time_, status_, current_ros_time_), std::runtime_error);
}

/**
 * @note Test basic functionality. Test lidar sensor correctness on a sample scene with some vehicle
 * - the goal is to check if the correct pointcloud is published on the correct topic.
 */
TEST_F(LidarSensorTest, update_correct)
{
  lidar_->update(current_simulation_time_, status_, current_ros_time_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  const auto total_num_of_points = received_msg_->width * received_msg_->height;
  EXPECT_GT(total_num_of_points, 0);
  EXPECT_EQ(received_msg_->header.frame_id, "base_link");
}

/**
 * @note Test function behavior when called with a current_time significantly smaller than one call
 * earlier - the goal is to test whether the function clears detected_objects.
 */
TEST_F(LidarSensorTest, update_goBackInTime)
{
  lidar_->update(current_simulation_time_, status_, rclcpp::Time(1000));

  // Ensure there are detected objects
  ASSERT_FALSE(lidar_->getDetectedObjects().empty());

  lidar_->update(current_simulation_time_, status_, rclcpp::Time(1));

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  EXPECT_TRUE(lidar_->getDetectedObjects().empty());
}

/**
 * @note Test basic functionality. Test detected objects obtaining from the statuses list containing
 * Ego.
 */
TEST_F(LidarSensorTest, getDetectedObjects)
{
  const std::set<std::string> expected_objects = {status_[1].name(), status_[2].name()};

  lidar_->update(current_simulation_time_, status_, current_ros_time_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  const auto & detected_objects = lidar_->getDetectedObjects();

  // LidarSensor returns duplicates. To avoid them, a std::set is used.
  const std::set<std::string> unique_objects(detected_objects.begin(), detected_objects.end());

  ASSERT_FALSE(unique_objects.empty());
  EXPECT_EQ(unique_objects, expected_objects);
}
