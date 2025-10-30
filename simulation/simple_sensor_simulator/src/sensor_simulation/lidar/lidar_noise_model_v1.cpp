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

#include <cmath>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_noise_model_v1.hpp>
#include <simple_sensor_simulator/sensor_simulation/noise_parameter_selector.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
LidarNoiseModelV1::LidarNoiseModelV1(const std::string & topic_name, int seed)
: random_engine_(seed == 0 ? std::random_device{}() : seed), topic_name_(topic_name)
{
  const auto config_names = noise_parameter_selector::listAvailableNoiseConfigs(topic_name_, "v1");
  for (const auto & config_name : config_names) {
    configs_.try_emplace(config_name, topic_name_, config_name);
  }
}

LidarNoiseModelV1::Config::Config(const std::string & topic_name, const std::string & config_name)
: true_positive_rate_ellipse_normalized_x_radius([&]() {
    const std::string base_path = topic_name + ".noise.v1." + config_name + ".";
    return common::getParameter<double>(
      base_path + "true_positive.rate.ellipse_normalized_x_radius");
  }()),
  distance_bins_([&]() {
    const std::string base_path = topic_name + ".noise.v1." + config_name + ".";

    const auto ellipse_y_radii =
      common::getParameter<std::vector<double>>(base_path + "ellipse_y_radii");
    const auto radial_mean =
      common::getParameter<std::vector<double>>(base_path + "distance.radial.mean.values");
    const auto radial_stddev = common::getParameter<std::vector<double>>(
      base_path + "distance.radial.standard_deviation.values");
    const auto tangential_mean =
      common::getParameter<std::vector<double>>(base_path + "distance.tangential.mean.values");
    const auto tangential_stddev = common::getParameter<std::vector<double>>(
      base_path + "distance.tangential.standard_deviation.values");
    const auto tpr_values =
      common::getParameter<std::vector<double>>(base_path + "true_positive.rate.values");

    std::vector<DistanceBin> bins;
    bins.reserve(ellipse_y_radii.size());
    for (size_t i = 0; i < ellipse_y_radii.size(); ++i) {
      bins.emplace_back(
        ellipse_y_radii[i] * ellipse_y_radii[i], radial_mean[i], radial_stddev[i],
        tangential_mean[i], tangential_stddev[i], tpr_values[i]);
    }
    return bins;
  }())
{
}

void LidarNoiseModelV1::applyNoise(
  Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose)
{
  // TODO
}
}  // namespace simple_sensor_simulator
