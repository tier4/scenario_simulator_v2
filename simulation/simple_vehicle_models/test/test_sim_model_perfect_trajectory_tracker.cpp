// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>
#include <simple_vehicle_models/sim_model_perfect_trajectory_tracker.hpp>

namespace
{
// Build a straight trajectory along +x with constant speed and zero yaw.
autoware_planning_msgs::msg::Trajectory makeStraightTrajectory(double speed, std::size_t n)
{
  autoware_planning_msgs::msg::Trajectory traj;
  for (std::size_t i = 0; i < n; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;  // yaw = 0
    p.longitudinal_velocity_mps = static_cast<float>(speed);
    p.acceleration_mps2 = 0.0f;
    p.front_wheel_angle_rad = 0.0f;
    traj.points.push_back(p);
  }
  return traj;
}

// SimModelPerfectTrajectoryTracker holds a std::mutex, so it is neither copyable nor
// movable — initialize in place through a reference rather than returning by value.
void initTrackerAtOrigin(SimModelPerfectTrajectoryTracker & tracker)
{
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.orientation.w = 1.0;
  tracker.setInitialReference(initial_pose, Eigen::Matrix3d::Identity());
  tracker.setStateZInitialFrame(0.0);
  tracker.setGear(autoware_vehicle_msgs::msg::GearCommand::DRIVE);
}
}  // namespace

// Proves the standalone library is linkable and that a straight trajectory advances the state.
TEST(SimModelPerfectTrajectoryTracker, StraightLineAdvancesAlongX)
{
  SimModelPerfectTrajectoryTracker tracker(0.0 /* no delay */);
  initTrackerAtOrigin(tracker);
  const double speed = 2.0;
  tracker.setTrajectory(rclcpp::Time(0, 0), makeStraightTrajectory(speed, 20));

  const double dt = 0.1;
  const int steps = 10;
  for (int i = 0; i < steps; ++i) {
    tracker.setStateZInitialFrame(0.0);
    tracker.update(dt);
  }

  // With v=2 m/s over 1.0 s along +x, x should be ~2 m, y ~0, yaw ~0, vx=2.
  EXPECT_NEAR(tracker.getX(), speed * dt * steps, 0.2);
  EXPECT_NEAR(tracker.getY(), 0.0, 1e-6);
  EXPECT_NEAR(tracker.getYaw(), 0.0, 1e-6);
  EXPECT_NEAR(tracker.getVx(), speed, 1e-6);
}

// Gear PARK must clamp velocity to zero regardless of trajectory speed.
TEST(SimModelPerfectTrajectoryTracker, ParkGearHoldsPosition)
{
  SimModelPerfectTrajectoryTracker tracker(0.0 /* no delay */);
  initTrackerAtOrigin(tracker);
  tracker.setGear(autoware_vehicle_msgs::msg::GearCommand::PARK);
  tracker.setTrajectory(rclcpp::Time(0, 0), makeStraightTrajectory(5.0, 20));

  for (int i = 0; i < 10; ++i) {
    tracker.setStateZInitialFrame(0.0);
    tracker.update(0.1);
  }

  EXPECT_NEAR(tracker.getX(), 0.0, 1e-9);
  EXPECT_NEAR(tracker.getVx(), 0.0, 1e-9);
}

// Without a trajectory or initial reference, update() is a no-op (no throw, no motion).
TEST(SimModelPerfectTrajectoryTracker, NoTrajectoryIsNoOp)
{
  SimModelPerfectTrajectoryTracker tracker(0.0);
  tracker.update(0.1);  // no reference, no trajectory
  EXPECT_NEAR(tracker.getX(), 0.0, 1e-9);
  EXPECT_NEAR(tracker.getVx(), 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
