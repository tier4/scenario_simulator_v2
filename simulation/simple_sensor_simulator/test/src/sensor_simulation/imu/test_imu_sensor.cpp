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

#include "test_imu_sensor.hpp"

TEST_F(ImuSensorTest, update_noNoiseNoGravity)
{
  // Reset noise
  config_.set_seed(0);
  config_.set_noise_standard_deviation_orientation(0);
  config_.set_noise_standard_deviation_twist(0);
  config_.set_noise_standard_deviation_acceleration(0);

  config_.set_add_gravity(false);

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z, 0.5);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x, 1.2);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, 0.0);
}

TEST_F(ImuSensorTest, update_noNoise)
{
  // Reset noise
  config_.set_seed(0);
  config_.set_noise_standard_deviation_orientation(0);
  config_.set_noise_standard_deviation_twist(0);
  config_.set_noise_standard_deviation_acceleration(0);

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z, 0.5);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x, 1.2);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -9.81);
}

TEST_F(ImuSensorTest, update_noiseLegacyNoGravity)
{
  config_.set_add_gravity(false);

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x, 0.77278062387447033);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y, -5.8781837092410099);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z, -4.0234230559671058);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x, 4.6327154870933622);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y, 0.2853719564601242);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -9.1692371736549081);
}

TEST_F(ImuSensorTest, update_noiseLegacy)
{
  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x, 0.77278062387447033);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y, -5.8781837092410099);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z, -4.0234230559671058);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x, 2.9561810004350946);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y, 8.9882587335964885);
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -4.9638722834948945);
}
