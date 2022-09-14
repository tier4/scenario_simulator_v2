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
#include <quaternion_operation/quaternion_operation.h>

namespace simple_sensor_simulator
{
class Raycaster
{
public:
  Raycaster();
  explicit Raycaster(std::string embree_config);
  ~Raycaster();
  unsigned int addToScene(RTCDevice device, RTCScene scene, primitives::Primitive primitive);
  void updateOnScene(RTCScene scene, std::string entity_name, primitives::Primitive primitive);
  template <typename T, typename... Ts>
  void addPrimitive(std::string name, Ts &&... xs)
  {
    // todo change name to updatePrimitve
    // todo delete this if
    // if (primitive_ptrs_.count(name) != 0) {
    //   //? is it neccessary
    //   std::clog << "primitive " + name + " already exist." << std::endl;
    //   return;
    // }

    //TODO mutable primitives
    auto primitive_ptr = std::make_unique<T>(std::forward<Ts>(xs)...);
    primitive_ptrs_.insert_or_assign(name, std::move(primitive_ptr));
  }
  const sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
    double horizontal_resolution, std::vector<double> vertical_angles,
    double horizontal_angle_start = 0, double horizontal_angle_end = 2 * M_PI,
    double max_distance = 100, double min_distance = 0);
  const std::vector<std::string> & getDetectedObject() const;

private:
  std::vector<geometry_msgs::msg::Quaternion> getDirections(
    std::vector<double> vertical_angles, double horizontal_angle_start,
    double horizontal_angle_end, double horizontal_resolution);
  std::vector<geometry_msgs::msg::Quaternion> directions_;
  double previous_horizontal_angle_start_;
  double previous_horizontal_angle_end_;
  double previous_horizontal_resolution_;
  std::vector<double>  previous_vertical_angles_;
  std::unordered_map<std::string, std::unique_ptr<primitives::Primitive>> primitive_ptrs_;
  std::unordered_map<std::string, unsigned int> primitive_on_scene_id_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  const sensor_msgs::msg::PointCloud2 raycast(
    std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
    std::vector<geometry_msgs::msg::Quaternion> directions, double max_distance = 100,
    double min_distance = 0);
  std::vector<std::string> detected_objects_;
  std::unordered_map<unsigned int, std::string> geometry_ids_;

  static void intersect(int thread_id, int thread_count,
                        RTCScene scene,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr thread_cloud,
                        RTCIntersectContext context,
                        geometry_msgs::msg::Pose origin,
                        std::set<unsigned int>& thread_detected_ids,
                        const std::vector<geometry_msgs::msg::Quaternion>& directions,
                        double max_distance, double min_distance)
  {
    for(int i = thread_id ; i < directions.size(); i += thread_count)
    {
      RTCRayHit rayhit = {};
      rayhit.ray.org_x = origin.position.x;
      rayhit.ray.org_y = origin.position.y;
      rayhit.ray.org_z = origin.position.z;
      // make raycast interact with all objects
      rayhit.ray.mask = 0b11111111'11111111'11111111'11111111;
      rayhit.ray.tfar = max_distance;
      rayhit.ray.tnear = min_distance;
      rayhit.ray.flags = false;
      const auto ray_direction = origin.orientation * directions.at(i);
      const auto rotation_mat = quaternion_operation::getRotationMatrix(ray_direction);
      const Eigen::Vector3d rotated_direction = rotation_mat * Eigen::Vector3d(1.0, 0.0, 0.0);
      rayhit.ray.dir_x = rotated_direction[0];
      rayhit.ray.dir_y = rotated_direction[1];
      rayhit.ray.dir_z = rotated_direction[2];
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

      std::chrono::steady_clock::time_point begin, end;
      // begin = std::chrono::steady_clock::now();
      rtcIntersect1(scene, &context, &rayhit);
      // end = std::chrono::steady_clock::now();
      // times.push_back(std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count());

      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        double distance = rayhit.ray.tfar;
        const Eigen::Vector3d vector = quaternion_operation::getRotationMatrix(directions.at(i)) *
                                      Eigen::Vector3d(1.0, 0.0, 0.0) * distance;
        pcl::PointXYZI p;
        {
          p.x = vector[0];
          p.y = vector[1];
          p.z = vector[2];
        }
        thread_cloud->emplace_back(p);
        thread_detected_ids.insert(rayhit.hit.geomID);
      }
    }
  }

};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
