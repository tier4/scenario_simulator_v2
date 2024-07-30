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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_

#include <embree4/rtcore.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
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
  explicit Raycaster(std::string embree_config);
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
  const sensor_msgs::msg::PointCloud2 raycast(
    const std::string & frame_id, const rclcpp::Time & stamp,
    const geometry_msgs::msg::Pose & origin, double max_distance = 300, double min_distance = 0);
  const std::vector<std::string> & getDetectedObject() const;
  void setDirection(
    const simulation_api_schema::LidarConfiguration & configuration,
    double horizontal_angle_start = 0, double horizontal_angle_end = 2 * M_PI);

private:
  std::vector<geometry_msgs::msg::Quaternion> getDirections(
    const std::vector<double> & vertical_angles, double horizontal_angle_start,
    double horizontal_angle_end, double horizontal_resolution);
  std::vector<geometry_msgs::msg::Quaternion> directions_;
  double previous_horizontal_angle_start_;
  double previous_horizontal_angle_end_;
  double previous_horizontal_resolution_;
  std::vector<double> previous_vertical_angles_;
  std::unordered_map<std::string, std::unique_ptr<primitives::Primitive>> primitive_ptrs_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  std::vector<std::string> detected_objects_;
  std::unordered_map<unsigned int, std::string> geometry_ids_;
  std::vector<Eigen::Matrix3d> rotation_matrices_;

  static void intersect(
    int thread_id, int thread_count, RTCScene scene,
    pcl::PointCloud<pcl::PointXYZI>::Ptr thread_cloud, geometry_msgs::msg::Pose origin,
    std::reference_wrapper<std::set<unsigned int>> ref_thread_detected_ids, double max_distance,
    double min_distance,
    std::reference_wrapper<const std::vector<Eigen::Matrix3d>> ref_rotation_matrices)
  {
    auto & rotation_matrices = ref_rotation_matrices.get();
    auto & thread_detected_ids = ref_thread_detected_ids.get();
    const auto orientation_matrix = math::geometry::getRotationMatrix(origin.orientation);
    for (unsigned int i = thread_id; i < rotation_matrices.size(); i += thread_count) {
      RTCRayHit rayhit = {};
      rayhit.ray.org_x = origin.position.x;
      rayhit.ray.org_y = origin.position.y;
      rayhit.ray.org_z = origin.position.z;
      // make raycast interact with all objects
      rayhit.ray.mask = 0b11111111'11111111'11111111'11111111;
      rayhit.ray.tfar = max_distance;
      rayhit.ray.tnear = min_distance;
      rayhit.ray.flags = false;

      const auto rotation_mat = orientation_matrix * rotation_matrices.at(i);
      rayhit.ray.dir_x = rotation_mat(0);
      rayhit.ray.dir_y = rotation_mat(1);
      rayhit.ray.dir_z = rotation_mat(2);
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rtcIntersect1(scene, &rayhit);

      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        double distance = rayhit.ray.tfar;
        pcl::PointXYZI p;
        {
          p.x = rotation_matrices.at(i)(0) * distance;
          p.y = rotation_matrices.at(i)(1) * distance;
          p.z = rotation_matrices.at(i)(2) * distance;
        }
        thread_cloud->emplace_back(p);
        thread_detected_ids.insert(rayhit.hit.geomID);
      }
    }
  }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
