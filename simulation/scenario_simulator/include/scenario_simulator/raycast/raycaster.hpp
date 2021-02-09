// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef SCENARIO_SIMULATOR__RAYCAST__RAYCASTER_HPP_
#define SCENARIO_SIMULATOR__RAYCAST__RAYCASTER_HPP_

#include <scenario_simulator/primitives/box.hpp>
#include <scenario_simulator/primitives/primitive.hpp>

#include <embree3/rtcore.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <random>

namespace scenario_simulator
{

class Raycaster
{
public:
  Raycaster();
  explicit Raycaster(std::string embree_config);
  ~Raycaster();
  template<typename T, typename ... Ts>
  void addPrimitive(std::string name, Ts && ... xs)
  {
    if (primitive_ptrs_.count(name) != 0) {
      throw std::runtime_error("primitive " + name + " already exist.");
    }
    auto primitive_ptr = std::make_unique<T>(std::forward<Ts>(xs)...);
    primitive_ptrs_.emplace(name, std::move(primitive_ptr));
  }
  const sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Pose origin,
    double horizontal_resolution,
    std::vector<double> vertical_angles,
    double horizontal_angle_start = 0,
    double horizontal_angle_end = 2 * M_PI,
    double max_distance = 100, double min_distance = 0
  );

private:
  std::unordered_map<std::string, std::unique_ptr<primitives::Primitive>> primitive_ptrs_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  const sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::Pose origin,
    std::vector<geometry_msgs::msg::Quaternion> directions,
    double max_distance = 100, double min_distance = 0);
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__RAYCAST__RAYCASTER_HPP_
