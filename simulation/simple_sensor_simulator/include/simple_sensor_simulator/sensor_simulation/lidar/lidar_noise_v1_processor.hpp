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
private:
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

    // Cached parameters for fast access
    const std::vector<double> ellipse_y_radii;
    const std::vector<double> ellipse_y_radii_squared;  // Precomputed for fast comparison

    // Distance parameters (radial and tangential)
    const DistanceNoiseParams radial_distance;
    const DistanceNoiseParams tangential_distance;

    const double true_positive_rate_ellipse_normalized_x_radius;
    const std::vector<double> true_positive_rate_values;

    EntityNoiseConfig(const std::string & topic_name, const std::string & config_name);
  };

  std::unordered_map<std::string, EntityNoiseConfig> noise_configs_;  // config_name -> config
  std::unordered_map<std::string, const EntityNoiseConfig *>
    entity_to_config_;  // entity_name -> config
  std::default_random_engine random_engine_;
  int noise_model_version_;
  std::string topic_name_;

  // Reusable distribution objects (to avoid per-point instantiation overhead)
  std::uniform_real_distribution<double> uniform_distribution_;
  std::normal_distribution<double> radial_distribution_;
  std::normal_distribution<double> tangential_distribution_;

public:
  explicit LidarNoiseV1Processor(const std::string & topic_name, int noise_model_version, int seed);

  void applyNoise(
    Raycaster::RaycastResult & result, double simulation_time,
    const geometry_msgs::msg::Pose & ego_pose);

private:
  void loadAllNoiseConfigs();

  auto getNoiseParameters(
    const traffic_simulator_msgs::EntityStatus & entity, const geometry_msgs::msg::Pose & ego_pose)
    -> NoiseParameters;

  /**
   * @brief Apply noise to a single point
   * @param point The point to modify (input/output)
   * @param params Noise parameters for this point
   * @return true if the point should be removed (based on true_positive rate), false otherwise
   */
  bool applyNoiseToPoint(pcl::PointXYZI & point, const NoiseParameters & params);
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_V1_PROCESSOR_HPP_
