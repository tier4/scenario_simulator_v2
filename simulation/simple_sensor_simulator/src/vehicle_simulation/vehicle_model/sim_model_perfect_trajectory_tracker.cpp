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

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/duration.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_perfect_trajectory_tracker.hpp>

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Extract yaw from quaternion without requiring tf2_geometry_msgs
double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

SimModelPerfectTrajectoryTracker::SimModelPerfectTrajectoryTracker(double delay_time_sec)
: SimModelInterface(4 /* dim_x: x, y, yaw, vx */, 0 /* dim_u */), delay_time_sec_(delay_time_sec)
{
}

void SimModelPerfectTrajectoryTracker::setInitialReference(
  const geometry_msgs::msg::Pose & initial_pose, const Eigen::Matrix3d & initial_rotation_matrix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  initial_pose_ = initial_pose;
  initial_rotation_matrix_ = initial_rotation_matrix;
  initial_yaw_ = quatToYaw(initial_pose.orientation);
  reference_initialized_ = true;
}

void SimModelPerfectTrajectoryTracker::setTrajectory(
  const rclcpp::Time & stamp, const autoware_planning_msgs::msg::Trajectory & message)
{
  if (message.points.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!trajectory_queue_.empty() && trajectory_queue_.front().stamp == stamp) return;
  trajectory_queue_.push_front({stamp, message});
  while (trajectory_queue_.size() > kQueueMax) {
    trajectory_queue_.pop_back();
  }
}

double SimModelPerfectTrajectoryTracker::getX()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_(X);
}

double SimModelPerfectTrajectoryTracker::getY()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_(Y);
}

double SimModelPerfectTrajectoryTracker::getYaw()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_(YAW);
}

double SimModelPerfectTrajectoryTracker::getVx()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_(VX);
}

double SimModelPerfectTrajectoryTracker::getAx()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_ax_;
}

double SimModelPerfectTrajectoryTracker::getWz()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_wz_;
}

double SimModelPerfectTrajectoryTracker::getSteer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_steer_;
}

void SimModelPerfectTrajectoryTracker::setStateZInitialFrame(double z)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_z_initial_frame_ = z;
}

void SimModelPerfectTrajectoryTracker::update(const double & dt)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!reference_initialized_ || trajectory_queue_.empty()) return;

  // 1. Select trajectory: oldest entry that is still newer than (latest - delay_time_sec_)
  const rclcpp::Time latest_stamp = trajectory_queue_.front().stamp;
  const rclcpp::Time target_stamp = latest_stamp - rclcpp::Duration::from_seconds(delay_time_sec_);

  const autoware_planning_msgs::msg::Trajectory * trajectory = &trajectory_queue_.back().msg;
  for (const auto & entry : trajectory_queue_) {
    if (entry.stamp <= target_stamp) {
      trajectory = &entry.msg;
      break;
    }
  }

  if (trajectory->points.empty()) return;

  // 2. Restore map-frame position from model-relative state
  const Eigen::Vector3d p_world =
    initial_rotation_matrix_ * Eigen::Vector3d(state_(X), state_(Y), state_z_initial_frame_);
  double cx = initial_pose_.position.x + p_world.x();
  double cy = initial_pose_.position.y + p_world.y();
  const double cyaw_map = initial_yaw_ + state_(YAW);

  // 3. Find closest trajectory point (O(N) linear scan, same as perfect_tracker)
  const std::size_t traj_size = trajectory->points.size();
  double min_dist_sq = std::numeric_limits<double>::max();
  std::size_t closest_idx = 0;
  for (std::size_t i = 0; i < traj_size; ++i) {
    const auto & p = trajectory->points[i];
    const double dx = cx - p.pose.position.x;
    const double dy = cy - p.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      closest_idx = i;
    }
  }

  const auto & target = trajectory->points[closest_idx];

  // 4. Copy trajectory kinematics, then clamp velocity to the direction allowed by the
  // current gear command. The trajectory's velocity sign is not always consistent with the
  // gear command (e.g. a stale trajectory may carry negative velocity after the planner
  // shifts to DRIVE), so the gear is treated as the source of truth for direction.
  double v = target.longitudinal_velocity_mps;
  switch (gear_) {
    case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
    case autoware_vehicle_msgs::msg::GearCommand::REVERSE_2:
      v = std::min(v, 0.0);  // REVERSE: forbid forward motion
      break;
    case autoware_vehicle_msgs::msg::GearCommand::PARK:
    case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
    case autoware_vehicle_msgs::msg::GearCommand::NONE:
      v = 0.0;  // PARK/NEUTRAL/NONE: no motion
      break;
    default:
      v = std::max(v, 0.0);  // DRIVE / DRIVE_2..20 / LOW / LOW_2: forbid reverse
      break;
  }
  // Acceleration and steer are independent of gear direction (e.g. ax<0 is
  // valid during braking in DRIVE), so they are copied from the trajectory as-is.
  current_ax_ = target.acceleration_mps2;
  current_steer_ = target.front_wheel_angle_rad;

  // Compute wz from consecutive yaw difference (heading_rate_rps is 0 in diffusion planner output)
  {
    const std::size_t j =
      (closest_idx + 1 < traj_size) ? closest_idx + 1 : (closest_idx > 0 ? closest_idx - 1 : 0);
    if (j != closest_idx) {
      const double yaw_i = quatToYaw(trajectory->points[closest_idx].pose.orientation);
      const double yaw_j = quatToYaw(trajectory->points[j].pose.orientation);
      const double dt_traj = (rclcpp::Duration(trajectory->points[j].time_from_start) -
                              rclcpp::Duration(trajectory->points[closest_idx].time_from_start))
                               .seconds();
      current_wz_ = (std::abs(dt_traj) > 1e-6) ? normalizeAngle(yaw_j - yaw_i) / dt_traj : 0.0;
    } else {
      current_wz_ = 0.0;
    }
  }

  // 5. Euler-integrate in map frame; yaw snapped to trajectory orientation.
  const double cyaw_map_next = quatToYaw(target.pose.orientation);
  cx += v * std::cos(cyaw_map) * dt;
  cy += v * std::sin(cyaw_map) * dt;

  // 6. Write back to model-relative state.
  //    Z is carried through state_z_initial_frame_ (set by EgoEntitySimulation from the
  //    lanelet-corrected altitude before each update()), so cz from step 2 is used as-is.
  const Eigen::Vector3d p_world_new(
    cx - initial_pose_.position.x, cy - initial_pose_.position.y, p_world.z());
  const Eigen::Vector3d p_rel_new = initial_rotation_matrix_.transpose() * p_world_new;
  state_(X) = p_rel_new.x();
  state_(Y) = p_rel_new.y();
  state_(YAW) = normalizeAngle(cyaw_map_next - initial_yaw_);
  state_(VX) = v;
}

Eigen::VectorXd SimModelPerfectTrajectoryTracker::calcModel(
  const Eigen::VectorXd & /*state*/, const Eigen::VectorXd & /*input*/)
{
  return Eigen::VectorXd::Zero(4);
}
