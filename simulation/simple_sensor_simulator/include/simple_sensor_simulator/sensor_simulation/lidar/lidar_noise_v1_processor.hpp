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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_V1_PROCESSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_V1_PROCESSOR_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <unordered_map>
#include <vector>

namespace simple_sensor_simulator
{

class LidarNoiseV1Processor
{
public:
  struct NoiseParameters
  {
    double radial_distance_mean;
    double radial_distance_std_dev;
    double tangential_distance_mean;
    double tangential_distance_std_dev;
    double true_positive_rate;
    std::string config_name;

    NoiseParameters()
    : radial_distance_mean(0.0),
      radial_distance_std_dev(0.0),
      tangential_distance_mean(0.0),
      tangential_distance_std_dev(0.0),
      true_positive_rate(1.0),
      config_name("")
    {
    }
  };

  struct DistanceNoiseParams
  {
    double mean_ellipse_normalized_x_radius;
    std::vector<double> mean_values;
    double std_dev_ellipse_normalized_x_radius;
    std::vector<double> std_dev_values;

    DistanceNoiseParams(const std::string & param_base_path, const std::string & direction_name);
  };

  struct EntityNoiseConfig
  {
    const std::string config_name;
    const std::string parameter_base_path;
    const std::vector<double> ellipse_y_radii;
    const std::vector<double> ellipse_y_radii_squared;  // Precomputed
    const DistanceNoiseParams radial_distance;
    const DistanceNoiseParams tangential_distance;
    const double true_positive_rate_ellipse_normalized_x_radius;
    const std::vector<double> true_positive_rate_values;

    EntityNoiseConfig(const std::string & topic_name, const std::string & config_name);
  };

  explicit LidarNoiseV1Processor(const std::string & topic_name, int seed);

  void applyNoise(Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose);

private:
  std::unordered_map<std::string, EntityNoiseConfig> noise_configs_;
  std::unordered_map<std::string, const EntityNoiseConfig *> entity_to_config_;
  std::default_random_engine random_engine_;
  std::string topic_name_;
  std::uniform_real_distribution<double> uniform_distribution_;
  std::normal_distribution<double> radial_distribution_;
  std::normal_distribution<double> tangential_distribution_;

  void loadAllNoiseConfigs();

  const EntityNoiseConfig * findOrAssignConfigForEntity(
    const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status);

  auto getNoiseParameters(
    const traffic_simulator_msgs::EntityStatus & entity, const geometry_msgs::msg::Pose & ego_pose)
    -> NoiseParameters;

  bool applyNoiseToPoint(pcl::PointXYZI & point, const NoiseParameters & params);

  static void removeMarkedPoints(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::vector<bool> & points_to_remove);

  static double getValueFromEllipse(
    double x, double y, double ellipse_normalized_x_radius,
    const std::vector<double> & ellipse_y_radii_squared, const std::vector<double> & values);
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_V1_PROCESSOR_HPP_
