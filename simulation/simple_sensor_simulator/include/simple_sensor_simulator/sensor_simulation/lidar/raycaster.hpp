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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_

#include <embree3/rtcore.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
class Raycaster
{
public:
  Raycaster();
  explicit Raycaster(const std::string & embree_config);
  ~Raycaster();
  template <typename T, typename... Ts>
  void addPrimitive(std::string name, Ts &&... xs)
  {
    if (primitive_ptrs_.count(name) != 0) {
      throw std::runtime_error("primitive " + name + " already exist.");
    }
    auto primitive_ptr = std::make_unique<T>(std::forward<Ts>(xs)...);
    primitive_ptrs_.emplace(name, std::move(primitive_ptr));
  }
  void clearPrimitives() { primitive_ptrs_.clear(); }
  sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
    double horizontal_resolution, const std::vector<double> & vertical_angles,
    double horizontal_angle_start = 0, double horizontal_angle_end = 2 * M_PI,
    double max_distance = 100, double min_distance = 0);
  const std::vector<std::string> & getDetectedObject() const;

private:
  std::unordered_map<std::string, std::unique_ptr<primitives::Primitive>> primitive_ptrs_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
    const std::vector<geometry_msgs::msg::Quaternion> & directions, double max_distance = 100,
    double min_distance = 0);
  std::vector<std::string> detected_objects_;
  std::unordered_map<unsigned int, std::string> geometry_ids_;

  std::function<std::tuple<std::unordered_set<unsigned int>, pcl::PointCloud<pcl::PointXYZI>::Ptr>(
    RTCScene, RTCIntersectContext &, const geometry_msgs::msg::Pose &,
    const std::vector<geometry_msgs::msg::Quaternion> &, double, double)>
    calc_intersects;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
