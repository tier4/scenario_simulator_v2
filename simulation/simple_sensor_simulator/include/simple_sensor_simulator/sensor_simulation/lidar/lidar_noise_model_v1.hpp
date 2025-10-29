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

class LidarNoiseModelV1
{
public:
  struct Config
  {
    struct DistanceBin
    {
      double ellipse_y_radius_squared;
      std::bernoulli_distribution detection_distribution;
      std::normal_distribution<double> radial_distribution;
      std::normal_distribution<double> tangential_distribution;

      DistanceBin(
        double y_radius_sq, double r_mean, double r_stddev, double t_mean, double t_stddev,
        double tpr)
      : ellipse_y_radius_squared(y_radius_sq),
        detection_distribution(tpr),
        radial_distribution(r_mean, r_stddev),
        tangential_distribution(t_mean, t_stddev)
      {
      }
    };

    const double true_positive_rate_ellipse_normalized_x_radius;
    std::vector<DistanceBin> distance_bins_;

    Config(const std::string & topic_name, const std::string & config_name);

    DistanceBin & getDistanceBin(double x, double y);

  private:
    size_t getBinIndex(double x, double y) const;
  };

  explicit LidarNoiseModelV1(const std::string & topic_name, int seed);

  void applyNoise(Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose);

private:
  std::unordered_map<std::string, Config> configs_;
  std::unordered_map<std::string, Config *> entity_to_config_;
  std::default_random_engine random_engine_;
  std::string topic_name_;

  void loadConfigs();

  Config * getConfigFor(
    const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status);

  static void removeMarkedPoints(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::vector<bool> & points_to_remove);
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_NOISE_MODEL_V1_HPP_
