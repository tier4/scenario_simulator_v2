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

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,          0.5133810994901602  );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,          0.84164755514435452 );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,         -0.077266493853651891);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,          0.14865775638481957 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,     0.77278062387447033 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,    -5.8781837092410099  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,    -4.0234230559671058  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x,  4.6327154870933622  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y,  0.2853719564601242  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -9.1692371736549081  );
  // clang-format on
}

TEST_F(ImuSensorTest, update_noiseLegacy)
{
  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,          0.5133810994901602  );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,          0.84164755514435452 );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,         -0.077266493853651891);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,          0.14865775638481957 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,     0.77278062387447033 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,    -5.8781837092410099  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,    -4.0234230559671058  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x,  2.9561810004350946  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y,  8.9882587335964885  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -4.9638722834948945  );
  // clang-format on
}

TEST_F(ImuSensorTest, update_noiseLegacyParamsSetNoGravity)
{
  config_.set_add_gravity(false);

  // clang-format off
  node_->declare_parameter<int   >(topic_ + ".seed",                                                                                 1);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.mean",                              2);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.standard_deviation",                3);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.mean",                        4);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.standard_deviation",          5);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.mean",                              6);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.standard_deviation",                7);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.mean",                        8);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.standard_deviation",          9);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.mean",                             10);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.standard_deviation",               11);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.mean",                       12);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.standard_deviation",         13);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.mean",                        14);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.standard_deviation",          15);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.mean",                  16);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.standard_deviation",    17);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.mean",                        18);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.standard_deviation",          19);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.mean",                  20);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.standard_deviation",    21);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.mean",                        22);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.standard_deviation",          23);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.mean",                  24);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.standard_deviation",    25);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.mean",                     26);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.standard_deviation",       27);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.mean",               28);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.standard_deviation", 29);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.mean",                     30);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.standard_deviation",       31);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.mean",               32);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.standard_deviation", 33);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.mean",                     34);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.standard_deviation",       35);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.mean",               36);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.standard_deviation", 37);
  // clang-format on

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,          0.5133810994901602  );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,          0.84164755514435452 );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,         -0.077266493853651891);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,          0.14865775638481957 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,     0.77278062387447033 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,    -5.8781837092410099  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,    -4.0234230559671058  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x,  4.6327154870933622  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y,  0.2853719564601242  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -9.1692371736549081  );
  // clang-format on
}

TEST_F(ImuSensorTest, update_noiseNoGravity)
{
  config_.set_add_gravity(false);

  node_->declare_parameter<bool>(topic_ + ".override_legacy_configuration", true);

  // clang-format off
  node_->declare_parameter<int   >(topic_ + ".seed",                                                                                 1);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.mean",                              2);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.standard_deviation",                3);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.mean",                        4);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.standard_deviation",          5);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.mean",                              6);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.standard_deviation",                7);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.mean",                        8);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.standard_deviation",          9);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.mean",                             10);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.standard_deviation",               11);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.mean",                       12);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.standard_deviation",         13);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.mean",                        14);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.standard_deviation",          15);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.mean",                  16);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.standard_deviation",    17);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.mean",                        18);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.standard_deviation",          19);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.mean",                  20);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.standard_deviation",    21);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.mean",                        22);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.standard_deviation",          23);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.mean",                  24);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.standard_deviation",    25);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.mean",                     26);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.standard_deviation",       27);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.mean",               28);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.standard_deviation", 29);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.mean",                     30);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.standard_deviation",       31);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.mean",               32);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.standard_deviation", 33);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.mean",                     34);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.standard_deviation",       35);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.mean",               36);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.standard_deviation", 37);
  // clang-format on

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,         -0.25471123590618572);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,         -0.26808591042725899);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,         -0.81054281449241206);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,          0.45417229859190028);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,    27.706498770761797  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,    36.693910966990558  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,    -5.7310867071314711 );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x, 83.397645584104936  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y, 42.186386919810495  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, 22.434366843481051  );
  // clang-format on
}

TEST_F(ImuSensorTest, update_noiseLegacyParamsSet)
{
  // clang-format off
  node_->declare_parameter<int   >(topic_ + ".seed",                                                                                 1);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.mean",                              2);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.standard_deviation",                3);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.mean",                        4);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.standard_deviation",          5);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.mean",                              6);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.standard_deviation",                7);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.mean",                        8);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.standard_deviation",          9);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.mean",                             10);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.standard_deviation",               11);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.mean",                       12);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.standard_deviation",         13);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.mean",                        14);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.standard_deviation",          15);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.mean",                  16);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.standard_deviation",    17);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.mean",                        18);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.standard_deviation",          19);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.mean",                  20);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.standard_deviation",    21);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.mean",                        22);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.standard_deviation",          23);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.mean",                  24);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.standard_deviation",    25);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.mean",                     26);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.standard_deviation",       27);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.mean",               28);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.standard_deviation", 29);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.mean",                     30);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.standard_deviation",       31);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.mean",               32);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.standard_deviation", 33);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.mean",                     34);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.standard_deviation",       35);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.mean",               36);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.standard_deviation", 37);
  // clang-format on

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,          0.5133810994901602  );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,          0.84164755514435452 );
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,         -0.077266493853651891);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,          0.14865775638481957 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,     0.77278062387447033 );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,    -5.8781837092410099  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,    -4.0234230559671058  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x,  2.9561810004350946  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y,  8.9882587335964885  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -4.9638722834948945  );
  // clang-format on
}

TEST_F(ImuSensorTest, update_noise)
{
  node_->declare_parameter<bool>(topic_ + ".override_legacy_configuration", true);

  // clang-format off
  node_->declare_parameter<int   >(topic_ + ".seed",                                                                                 1);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.mean",                              2);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.additive.standard_deviation",                3);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.mean",                        4);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.r.error.multiplicative.standard_deviation",          5);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.mean",                              6);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.additive.standard_deviation",                7);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.mean",                        8);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.p.error.multiplicative.standard_deviation",          9);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.mean",                             10);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.additive.standard_deviation",               11);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.mean",                       12);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.orientation.y.error.multiplicative.standard_deviation",         13);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.mean",                        14);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.additive.standard_deviation",          15);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.mean",                  16);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.x.error.multiplicative.standard_deviation",    17);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.mean",                        18);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.additive.standard_deviation",          19);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.mean",                  20);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.y.error.multiplicative.standard_deviation",    21);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.mean",                        22);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.additive.standard_deviation",          23);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.mean",                  24);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.angular_velocity.z.error.multiplicative.standard_deviation",    25);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.mean",                     26);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.additive.standard_deviation",       27);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.mean",               28);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.x.error.multiplicative.standard_deviation", 29);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.mean",                     30);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.additive.standard_deviation",       31);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.mean",               32);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.y.error.multiplicative.standard_deviation", 33);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.mean",                     34);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.additive.standard_deviation",       35);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.mean",               36);
  node_->declare_parameter<double>(topic_ + ".sensor_msgs::msg::Imu.linear_acceleration.z.error.multiplicative.standard_deviation", 37);
  // clang-format on

  initializeSensor();

  imu_->update(current_ros_time_, status_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // clang-format off
  EXPECT_DOUBLE_EQ(received_msg_->orientation.w,           -0.25471123590618572);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.x,           -0.26808591042725899);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.y,           -0.81054281449241206);
  EXPECT_DOUBLE_EQ(received_msg_->orientation.z,            0.45417229859190028);
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.x,      27.706498770761797  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.y,      36.693910966990558  );
  EXPECT_DOUBLE_EQ(received_msg_->angular_velocity.z,      -5.7310867071314711 );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.x,   83.397645584104936  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.y,   42.186386919810495  );
  EXPECT_DOUBLE_EQ(received_msg_->linear_acceleration.z, -237.95398578137113   );
  // clang-format on
}
