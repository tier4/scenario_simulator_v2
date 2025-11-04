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

#include <concealer/publisher.hpp>

auto rclcpp_initialized = false;

TEST(NormalDistribution_nav_msgs_msg_Odometry, when_no_parameters_are_given)
{
  if (not std::exchange(rclcpp_initialized, true)) {
    rclcpp::init(0, nullptr);
  }

  auto node = rclcpp::Node("node_name", "simulation");

  // clang-format off
  ASSERT_FALSE(node.has_parameter("/topic_name.seed"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.standard_deviation"));
  // clang-format on

  auto randomize = concealer::NormalDistribution<nav_msgs::msg::Odometry>(
    node.get_node_parameters_interface(), "/topic_name");

  ASSERT_EQ(randomize.seed, 0);
  ASSERT_EQ(randomize.position_local_x_error.additive.mean(), 0);
  ASSERT_EQ(randomize.position_local_x_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.position_local_x_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.position_local_x_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.position_local_y_error.additive.mean(), 0);
  ASSERT_EQ(randomize.position_local_y_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.position_local_y_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.position_local_y_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.position_local_z_error.additive.mean(), 0);
  ASSERT_EQ(randomize.position_local_z_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.position_local_z_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.position_local_z_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.orientation_r_error.additive.mean(), 0);
  ASSERT_EQ(randomize.orientation_r_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.orientation_r_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.orientation_r_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.orientation_p_error.additive.mean(), 0);
  ASSERT_EQ(randomize.orientation_p_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.orientation_p_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.orientation_p_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.orientation_y_error.additive.mean(), 0);
  ASSERT_EQ(randomize.orientation_y_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.orientation_y_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.orientation_y_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.linear_x_error.additive.mean(), 0);
  ASSERT_EQ(randomize.linear_x_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.linear_x_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.linear_x_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.linear_y_error.additive.mean(), 0);
  ASSERT_EQ(randomize.linear_y_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.linear_y_error.additive.mean(), 0);
  ASSERT_EQ(randomize.linear_y_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.angular_x_error.additive.mean(), 0);
  ASSERT_EQ(randomize.angular_x_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.angular_x_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.angular_x_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.angular_y_error.additive.mean(), 0);
  ASSERT_EQ(randomize.angular_y_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.angular_y_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.angular_y_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.angular_z_error.additive.mean(), 0);
  ASSERT_EQ(randomize.angular_z_error.additive.stddev(), 0);
  ASSERT_EQ(randomize.angular_z_error.multiplicative.mean(), 0);
  ASSERT_EQ(randomize.angular_z_error.multiplicative.stddev(), 0);

  const auto odometry = nav_msgs::msg::Odometry();

  const auto randomized_odometry = randomize(odometry);

  // clang-format off
  ASSERT_EQ(randomized_odometry.pose.pose.position.x,    0);
  ASSERT_EQ(randomized_odometry.pose.pose.position.y,    0);
  ASSERT_EQ(randomized_odometry.pose.pose.position.z,    0);
  ASSERT_EQ(randomized_odometry.pose.pose.orientation.x, 0);
  ASSERT_EQ(randomized_odometry.pose.pose.orientation.y, 0);
  ASSERT_EQ(randomized_odometry.pose.pose.orientation.z, 0);
  ASSERT_EQ(randomized_odometry.pose.pose.orientation.w, 1);
  ASSERT_EQ(randomized_odometry.twist.twist.linear.x,    0);
  ASSERT_EQ(randomized_odometry.twist.twist.linear.y,    0);
  ASSERT_EQ(randomized_odometry.twist.twist.linear.z,    0);
  ASSERT_EQ(randomized_odometry.twist.twist.angular.x,   0);
  ASSERT_EQ(randomized_odometry.twist.twist.angular.y,   0);
  ASSERT_EQ(randomized_odometry.twist.twist.angular.z,   0);
  // clang-format on
}

TEST(NormalDistribution_nav_msgs_msg_Odometry, when_all_parameters_are_given)
{
  if (not std::exchange(rclcpp_initialized, true)) {
    rclcpp::init(0, nullptr);
  }

  auto node = rclcpp::Node("node_name", "simulation");

  // clang-format off
  ASSERT_FALSE(node.has_parameter("/topic_name.seed"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.standard_deviation"));
  // clang-format on

  // clang-format off
  node.declare_parameter<int   >("/topic_name.seed",                                                                                        1);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.mean",                      2);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.standard_deviation",        3);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.mean",                4);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.standard_deviation",  5);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.mean",                      6);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.standard_deviation",        7);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.mean",                8);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.standard_deviation",  9);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.mean",                     10);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.standard_deviation",       11);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.mean",               12);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.standard_deviation", 13);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.mean",                        14);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.standard_deviation",          15);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.mean",                  16);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.standard_deviation",    17);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.mean",                        18);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.standard_deviation",          19);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.mean",                  20);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.standard_deviation",    21);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.mean",                        22);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.standard_deviation",          23);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.mean",                  24);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.standard_deviation",    25);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.mean",                           26);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.standard_deviation",             27);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.mean",                     28);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.standard_deviation",       29);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.mean",                           30);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.standard_deviation",             31);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.mean",                     32);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.standard_deviation",       33);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.mean",                           34);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.standard_deviation",             35);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.mean",                     36);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.standard_deviation",       37);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.mean",                          38);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.standard_deviation",            39);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.mean",                    40);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.standard_deviation",      41);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.mean",                          42);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.standard_deviation",            43);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.mean",                    44);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.standard_deviation",      45);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.mean",                          46);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.standard_deviation",            47);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.mean",                    48);
  node.declare_parameter<double>("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.standard_deviation",      49);
  // clang-format on

  // clang-format off
  ASSERT_TRUE(node.has_parameter("/topic_name.seed"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_x.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_y.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.position.local_z.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.r.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.p.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.pose.pose.orientation.y.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.x.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.y.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.linear.z.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.x.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.y.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.nav_msgs::msg::Odometry.twist.twist.angular.z.error.multiplicative.standard_deviation"));
  // clang-format on

  auto randomize = concealer::NormalDistribution<nav_msgs::msg::Odometry>(
    node.get_node_parameters_interface(), "/topic_name");

  // clang-format off
  ASSERT_EQ(randomize.seed,                                            1);
  ASSERT_EQ(randomize.position_local_x_error.additive.mean(),          2);
  ASSERT_EQ(randomize.position_local_x_error.additive.stddev(),        3);
  ASSERT_EQ(randomize.position_local_x_error.multiplicative.mean(),    4);
  ASSERT_EQ(randomize.position_local_x_error.multiplicative.stddev(),  5);
  ASSERT_EQ(randomize.position_local_y_error.additive.mean(),          6);
  ASSERT_EQ(randomize.position_local_y_error.additive.stddev(),        7);
  ASSERT_EQ(randomize.position_local_y_error.multiplicative.mean(),    8);
  ASSERT_EQ(randomize.position_local_y_error.multiplicative.stddev(),  9);
  ASSERT_EQ(randomize.position_local_z_error.additive.mean(),         10);
  ASSERT_EQ(randomize.position_local_z_error.additive.stddev(),       11);
  ASSERT_EQ(randomize.position_local_z_error.multiplicative.mean(),   12);
  ASSERT_EQ(randomize.position_local_z_error.multiplicative.stddev(), 13);
  ASSERT_EQ(randomize.orientation_r_error.additive.mean(),            14);
  ASSERT_EQ(randomize.orientation_r_error.additive.stddev(),          15);
  ASSERT_EQ(randomize.orientation_r_error.multiplicative.mean(),      16);
  ASSERT_EQ(randomize.orientation_r_error.multiplicative.stddev(),    17);
  ASSERT_EQ(randomize.orientation_p_error.additive.mean(),            18);
  ASSERT_EQ(randomize.orientation_p_error.additive.stddev(),          19);
  ASSERT_EQ(randomize.orientation_p_error.multiplicative.mean(),      20);
  ASSERT_EQ(randomize.orientation_p_error.multiplicative.stddev(),    21);
  ASSERT_EQ(randomize.orientation_y_error.additive.mean(),            22);
  ASSERT_EQ(randomize.orientation_y_error.additive.stddev(),          23);
  ASSERT_EQ(randomize.orientation_y_error.multiplicative.mean(),      24);
  ASSERT_EQ(randomize.orientation_y_error.multiplicative.stddev(),    25);
  ASSERT_EQ(randomize.linear_x_error.additive.mean(),                 26);
  ASSERT_EQ(randomize.linear_x_error.additive.stddev(),               27);
  ASSERT_EQ(randomize.linear_x_error.multiplicative.mean(),           28);
  ASSERT_EQ(randomize.linear_x_error.multiplicative.stddev(),         29);
  ASSERT_EQ(randomize.linear_y_error.additive.mean(),                 30);
  ASSERT_EQ(randomize.linear_y_error.additive.stddev(),               31);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.mean(),           32);
  ASSERT_EQ(randomize.linear_y_error.multiplicative.stddev(),         33);
  ASSERT_EQ(randomize.linear_z_error.additive.mean(),                 34);
  ASSERT_EQ(randomize.linear_z_error.additive.stddev(),               35);
  ASSERT_EQ(randomize.linear_z_error.multiplicative.mean(),           36);
  ASSERT_EQ(randomize.linear_z_error.multiplicative.stddev(),         37);
  ASSERT_EQ(randomize.angular_x_error.additive.mean(),                38);
  ASSERT_EQ(randomize.angular_x_error.additive.stddev(),              39);
  ASSERT_EQ(randomize.angular_x_error.multiplicative.mean(),          40);
  ASSERT_EQ(randomize.angular_x_error.multiplicative.stddev(),        41);
  ASSERT_EQ(randomize.angular_y_error.additive.mean(),                42);
  ASSERT_EQ(randomize.angular_y_error.additive.stddev(),              43);
  ASSERT_EQ(randomize.angular_y_error.multiplicative.mean(),          44);
  ASSERT_EQ(randomize.angular_y_error.multiplicative.stddev(),        45);
  ASSERT_EQ(randomize.angular_z_error.additive.mean(),                46);
  ASSERT_EQ(randomize.angular_z_error.additive.stddev(),              47);
  ASSERT_EQ(randomize.angular_z_error.multiplicative.mean(),          48);
  ASSERT_EQ(randomize.angular_z_error.multiplicative.stddev(),        49);
  // clang-format on

  const auto odometry = nav_msgs::msg::Odometry();

  const auto randomized_odometry = randomize(odometry);

  // clang-format off
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.position.x,     4.0604709175379767   );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.position.y,    19.565623431299677    );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.position.z,     2.8688348437534232   );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.orientation.x, -0.80099847581632522  );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.orientation.y, -0.29036040347964753  );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.orientation.z,  0.52344585659820897  );
  ASSERT_DOUBLE_EQ(randomized_odometry.pose.pose.orientation.w,  0.0098342788870005027);
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.linear.x,    13.010296958573839    );
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.linear.y,    42.186386919810495    );
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.linear.z,    22.434366843481051    );
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.angular.x,   41.550077290481802    );
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.angular.y,   40.801412395444473    );
  ASSERT_DOUBLE_EQ(randomized_odometry.twist.twist.angular.z,   44.530605885302229    );
  // clang-format on
}

TEST(NormalDistribution_autoware_vehicle_msgs_msg_VelocityReport, when_no_parameters_are_given)
{
  if (not std::exchange(rclcpp_initialized, true)) {
    rclcpp::init(0, nullptr);
  }

  auto node = rclcpp::Node("node_name", "simulation");

  // clang-format off
  ASSERT_FALSE(node.has_parameter("/topic_name.seed"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.standard_deviation"));
  // clang-format on

  auto randomize = concealer::NormalDistribution<autoware_vehicle_msgs::msg::VelocityReport>(
    node.get_node_parameters_interface(), "/topic_name");

  // clang-format off
  ASSERT_EQ(randomize.seed,                                                0);
  ASSERT_EQ(randomize.longitudinal_velocity_error.additive.mean(),         0);
  ASSERT_EQ(randomize.longitudinal_velocity_error.additive.stddev(),       0);
  ASSERT_EQ(randomize.longitudinal_velocity_error.multiplicative.mean(),   0);
  ASSERT_EQ(randomize.longitudinal_velocity_error.multiplicative.stddev(), 0);
  ASSERT_EQ(randomize.lateral_velocity_error.additive.mean(),              0);
  ASSERT_EQ(randomize.lateral_velocity_error.additive.stddev(),            0);
  ASSERT_EQ(randomize.lateral_velocity_error.multiplicative.mean(),        0);
  ASSERT_EQ(randomize.lateral_velocity_error.multiplicative.stddev(),      0);
  ASSERT_EQ(randomize.heading_rate_error.additive.mean(),                  0);
  ASSERT_EQ(randomize.heading_rate_error.additive.stddev(),                0);
  ASSERT_EQ(randomize.heading_rate_error.multiplicative.mean(),            0);
  ASSERT_EQ(randomize.heading_rate_error.multiplicative.stddev(),          0);
  // clang-format on

  const auto velocity_report = autoware_vehicle_msgs::msg::VelocityReport();

  const auto randomized_velocity_report = randomize(velocity_report);

  // clang-format off
  ASSERT_EQ(randomized_velocity_report.longitudinal_velocity, 0);
  ASSERT_EQ(randomized_velocity_report.lateral_velocity,      0);
  ASSERT_EQ(randomized_velocity_report.heading_rate,          0);
  // clang-format on
}

TEST(NormalDistribution_autoware_vehicle_msgs_msg_VelocityReport, when_all_parameters_are_given)
{
  if (not std::exchange(rclcpp_initialized, true)) {
    rclcpp::init(0, nullptr);
  }

  auto node = rclcpp::Node("node_name", "simulation");

  // clang-format off
  ASSERT_FALSE(node.has_parameter("/topic_name.seed"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.standard_deviation"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.mean"));
  ASSERT_FALSE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.standard_deviation"));
  // clang-format on

  // clang-format off
  node.declare_parameter<int   >("/topic_name.seed",                                                                                                     1);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.mean",                     2);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.standard_deviation",       3);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.mean",               4);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.standard_deviation", 5);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.mean",                          6);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.standard_deviation",            7);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.mean",                    8);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.standard_deviation",      9);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.mean",                             10);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.standard_deviation",               11);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.mean",                       12);
  node.declare_parameter<double>("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.standard_deviation",         13);
  // clang-format on

  // clang-format off
  ASSERT_TRUE(node.has_parameter("/topic_name.seed"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error.multiplicative.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.additive.standard_deviation"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.mean"));
  ASSERT_TRUE(node.has_parameter("/topic_name.autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error.multiplicative.standard_deviation"));
  // clang-format on

  auto randomize = concealer::NormalDistribution<autoware_vehicle_msgs::msg::VelocityReport>(
    node.get_node_parameters_interface(), "/topic_name");

  // clang-format off
  ASSERT_EQ(randomize.seed,                                                1);
  ASSERT_EQ(randomize.longitudinal_velocity_error.additive.mean(),         2);
  ASSERT_EQ(randomize.longitudinal_velocity_error.additive.stddev(),       3);
  ASSERT_EQ(randomize.longitudinal_velocity_error.multiplicative.mean(),   4);
  ASSERT_EQ(randomize.longitudinal_velocity_error.multiplicative.stddev(), 5);
  ASSERT_EQ(randomize.lateral_velocity_error.additive.mean(),              6);
  ASSERT_EQ(randomize.lateral_velocity_error.additive.stddev(),            7);
  ASSERT_EQ(randomize.lateral_velocity_error.multiplicative.mean(),        8);
  ASSERT_EQ(randomize.lateral_velocity_error.multiplicative.stddev(),      9);
  ASSERT_EQ(randomize.heading_rate_error.additive.mean(),                 10);
  ASSERT_EQ(randomize.heading_rate_error.additive.stddev(),               11);
  ASSERT_EQ(randomize.heading_rate_error.multiplicative.mean(),           12);
  ASSERT_EQ(randomize.heading_rate_error.multiplicative.stddev(),         13);
  // clang-format on

  const auto velocity_report = autoware_vehicle_msgs::msg::VelocityReport();

  const auto randomized_velocity_report = randomize(velocity_report);

  // clang-format off
  ASSERT_FLOAT_EQ(randomized_velocity_report.longitudinal_velocity, 4.06047  );
  ASSERT_FLOAT_EQ(randomized_velocity_report.lateral_velocity,     19.565622 );
  ASSERT_FLOAT_EQ(randomized_velocity_report.heading_rate,          2.8688359);
  // clang-format on
}
