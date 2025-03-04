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

#ifndef TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_
#define TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_

#include <simulation_api_schema.pb.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <string>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>
#include <unordered_set>
#include <vector>

namespace traffic_simulator
{
namespace helper
{
/**
 * @brief helper function for constructing action status
 *
 * @param linear_vel linear velocity
 * @param angular_vel angular velocity
 * @param linear_accel linear acceleration
 * @param angular_accel angular acceleration
 * @return traffic_simulator_msgs::msg::ActionStatus
 */
traffic_simulator_msgs::msg::ActionStatus constructActionStatus(
  double linear_vel = 0, double angular_vel = 0, double linear_accel = 0, double angular_accel = 0);

/**
 * @brief helper function for constructing lanelet pose
 *
 * @param lanelet_id lanelet id
 * @param s s value in lane coordinate
 * @param offset offset value in lane coordinate
 * @param roll roll value in the lane coordinate
 * @param pitch pitch value in the lane coordinate
 * @param yaw yaw value in the lane coordinate
 * @return LaneletPose
 */
LaneletPose constructLaneletPose(
  lanelet::Id lanelet_id, double s, double offset = 0, double roll = 0, double pitch = 0,
  double yaw = 0);

/**
 * @brief helper function for constructing canonicalized lanelet pose
 *
 * @param lanelet_id lanelet id
 * @param s s value in lane coordinate
 * @param offset offset value in lane coordinate
 * @return LaneletPose
 */
auto constructCanonicalizedLaneletPose(lanelet::Id lanelet_id, double s, double offset)
  -> CanonicalizedLaneletPose;

/**
 * @brief helper function for constructing canonicalized lanelet pose
 *
 * @param lanelet_id lanelet id
 * @param s s value in lane coordinate
 * @param offset offset value in lane coordinate
 * @param roll roll value in the lane coordinate
 * @param pitch pitch value in the lane coordinate
 * @param yaw yaw value in the lane coordinate
 * @return LaneletPose
 */
auto constructCanonicalizedLaneletPose(
  lanelet::Id lanelet_id, double s, double offset, double roll, double pitch, double yaw)
  -> CanonicalizedLaneletPose;

/**
 * @brief helper function for constructing rpy
 *
 * @param roll roll value of the orientation
 * @param pitch pitch value of the orientation
 * @param yaw yaw value of the orientation
 * @return geometry_msgs::msg::Vector3 RPY values
 */
geometry_msgs::msg::Vector3 constructRPY(double roll = 0, double pitch = 0, double yaw = 0);

/**
 * @brief helper function for constructing rpy
 *
 * @param quaternion quaternion class
 * @return geometry_msgs::msg::Vector3 RPY value
 */
geometry_msgs::msg::Vector3 constructRPYfromQuaternion(geometry_msgs::msg::Quaternion quaternion);

/**
 * @brief helper function for constructing pose
 *
 * @param x x value in position
 * @param y y value in position
 * @param z z value in position
 * @param roll roll value in orientation
 * @param pitch pitch value in orientation
 * @param yaw yaw value in orientation
 * @return geometry_msgs::msg::Pose
 */
geometry_msgs::msg::Pose constructPose(
  double x, double y, double z, double roll, double pitch, double yaw);

/**
 * @brief helper function for creating vector without duplicates, with preserved order
 *
 * @param vector input std::vector
 * @return new std::vector without duplicates and with relative order preserved
 */
template <typename T>
std::vector<T> getUniqueValues(const std::vector<T> & input_vector)
{
  std::vector<T> output_vector(input_vector);

  std::unordered_set<T> unique_values;
  auto empty_elements_start = std::remove_if(
    output_vector.begin(), output_vector.end(),
    [&unique_values](T const & element) { return !unique_values.insert(element).second; });
  output_vector.erase(empty_elements_start, output_vector.end());

  return output_vector;
}

enum class LidarType { VLP16, VLP32 };

const simulation_api_schema::LidarConfiguration constructLidarConfiguration(
  const LidarType type, const std::string & entity, const std::string & architecture_type,
  const double lidar_sensor_delay = 0, const double horizontal_resolution = 1.0 / 180.0 * M_PI);

const simulation_api_schema::DetectionSensorConfiguration constructDetectionSensorConfiguration(
  const std::string & entity, const std::string & architecture_type, const double update_duration,
  const double range = 300.0, const bool detect_all_objects_in_range = false,
  const double pos_noise_stddev = 0, const int random_seed = 0,
  const double probability_of_lost = 0, const double object_recognition_delay = 0,
  const double object_recognition_ground_truth_delay = 0);
}  // namespace helper
}  // namespace traffic_simulator

template <typename T>
auto operator+(const std::vector<T> & v0, const std::vector<T> & v1) -> decltype(auto)
{
  auto result = v0;
  result.reserve(v0.size() + v1.size());
  result.insert(result.end(), v1.begin(), v1.end());
  return result;
}

template <typename T>
auto operator+=(std::vector<T> & v0, const std::vector<T> & v1) -> decltype(auto)
{
  v0.reserve(v0.size() + v1.size());
  v0.insert(v0.end(), v1.begin(), v1.end());
  return v0;
}

template <typename T>
auto sortAndUnique(const std::vector<T> & data) -> std::vector<T>
{
  std::vector<T> ret = data;
  std::sort(ret.begin(), ret.end());
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}
#endif  // TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_
