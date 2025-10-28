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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_MODEL_V1_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_MODEL_V1_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <random>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <unordered_map>
#include <vector>

namespace simple_sensor_simulator
{

class LidarNoiseModel
{
public:
  struct DistanceParams
  {
    double mean_ellipse_normalized_x_radius;
    std::vector<double> mean_values;
    double std_dev_ellipse_normalized_x_radius;
    std::vector<double> std_dev_values;

    DistanceParams(const std::string & param_base_path, const std::string & direction_name);
  };

  struct Config
  {
    struct PositionParams
    {
      double true_positive_rate;
      std::string config_name;
      std::normal_distribution<double> radial_distribution;
      std::normal_distribution<double> tangential_distribution;

      PositionParams()
      : true_positive_rate(1.0),
        config_name(""),
        radial_distribution(0.0, 0.0),
        tangential_distribution(0.0, 0.0)
      {
      }
    };

    const std::string config_name;
    const std::string parameter_base_path;
    const std::vector<double> ellipse_y_radii;
    const std::vector<double> ellipse_y_radii_squared;  // Precomputed
    const DistanceParams radial_distance;
    const DistanceParams tangential_distance;
    const double true_positive_rate_ellipse_normalized_x_radius;
    const std::vector<double> true_positive_rate_values;

    Config(const std::string & topic_name, const std::string & config_name);

    PositionParams getPositionParams(double x, double y) const;

  private:
    static double getValueFromEllipse(
      double x, double y, double ellipse_normalized_x_radius,
      const std::vector<double> & ellipse_y_radii_squared, const std::vector<double> & values);
  };

  explicit LidarNoiseModel(const std::string & topic_name, int seed);

  void applyNoise(Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose);

private:
  std::unordered_map<std::string, Config> configs_;
  std::unordered_map<std::string, const Config *> entity_to_config_;
  std::default_random_engine random_engine_;
  std::string topic_name_;
  std::uniform_real_distribution<double> uniform_distribution_;

  void loadConfigs();

  const Config * getConfigFor(
    const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status);

  static void removeMarkedPoints(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::vector<bool> & points_to_remove);
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_MODEL_V1_HPP_
