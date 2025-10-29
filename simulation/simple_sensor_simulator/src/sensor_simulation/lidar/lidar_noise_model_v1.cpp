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
  loadConfigs();
}

LidarNoiseModelV1::Config::Config(const std::string & topic_name, const std::string & config_name)
: true_positive_rate_ellipse_normalized_x_radius([&]() {
    const std::string param_base = topic_name + ".noise.v1." + config_name + ".";
    return common::getParameter<double>(
      param_base + "true_positive.rate.ellipse_normalized_x_radius");
  }()),
  distance_bins_([&]() {
    const std::string param_base = topic_name + ".noise.v1." + config_name + ".";

    // Load all required parameters locally
    const auto ellipse_y_radii =
      common::getParameter<std::vector<double>>(param_base + "ellipse_y_radii");
    const auto radial_mean =
      common::getParameter<std::vector<double>>(param_base + "distance.radial.mean.values");
    const auto radial_stddev = common::getParameter<std::vector<double>>(
      param_base + "distance.radial.standard_deviation.values");
    const auto tangential_mean =
      common::getParameter<std::vector<double>>(param_base + "distance.tangential.mean.values");
    const auto tangential_stddev = common::getParameter<std::vector<double>>(
      param_base + "distance.tangential.standard_deviation.values");
    const auto tpr_values =
      common::getParameter<std::vector<double>>(param_base + "true_positive.rate.values");

    // Create distance bins
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

void LidarNoiseModelV1::loadConfigs()
{
  const auto config_names = noise_parameter_selector::listAvailableNoiseConfigs(topic_name_, "v1");
  for (const auto & config_name : config_names) {
    configs_.try_emplace(config_name, topic_name_, config_name);
  }
}

size_t LidarNoiseModelV1::Config::getBinIndex(double x, double y) const
{
  // Use true_positive_rate ellipse parameters as representative for bin determination
  const double x_normalized = x / true_positive_rate_ellipse_normalized_x_radius;
  const double distance_squared = x_normalized * x_normalized + y * y;

  for (size_t i = 0; i < distance_bins_.size(); ++i) {
    if (distance_squared < distance_bins_[i].ellipse_y_radius_squared) {
      return i;
    }
  }
  // Return last bin index if beyond all ellipses
  return distance_bins_.empty() ? 0 : distance_bins_.size() - 1;
}

LidarNoiseModelV1::Config::DistanceBin & LidarNoiseModelV1::Config::getDistanceBin(
  double x, double y)
{
  const size_t bin_index = getBinIndex(x, y);
  return distance_bins_[bin_index];
}

LidarNoiseModelV1::Config * LidarNoiseModelV1::getConfigFor(
  const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status)
{
  if (auto it = entity_to_config_.find(entity_name); it != entity_to_config_.end()) {
    return it->second;
  }

  Config * config_ptr = nullptr;
  if (const auto config_name = noise_parameter_selector::findMatchingNoiseConfigForEntity(
        entity_status, "v1", topic_name_);
      not config_name.empty()) {
    config_ptr = &configs_.at(config_name);
  }

  entity_to_config_[entity_name] = config_ptr;
  return config_ptr;
}

void LidarNoiseModelV1::removeMarkedPoints(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::vector<bool> & points_to_remove)
{
  const size_t cloud_size = cloud->size();
  auto write_it = cloud->points.begin();

  for (size_t i = 0; i < cloud_size; ++i) {
    if (!points_to_remove[i]) {
      if (write_it != cloud->points.begin() + i) {
        *write_it = cloud->points[i];
      }
      ++write_it;
    }
  }

  cloud->points.resize(std::distance(cloud->points.begin(), write_it));
  cloud->width = cloud->points.size();
  cloud->height = 1;
}

void LidarNoiseModelV1::applyNoise(
  Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose)
{
  auto & cloud = result.cloud;
  const auto & point_entity_indices = result.point_to_entity_index;
  const auto & raycast_entities = result.raycast_entities;

  if (!cloud || cloud->empty() || point_entity_indices.size() != cloud->size()) {
    return;
  }

  const size_t cloud_size = cloud->size();
  std::vector<bool> points_to_remove(cloud_size, false);

  // Group points by entity
  std::vector<std::vector<size_t>> entity_to_point_indices(raycast_entities.size());
  for (size_t i = 0; i < cloud_size; ++i) {
    const size_t entity_idx = point_entity_indices[i];
    if (entity_idx >= raycast_entities.size()) {
      points_to_remove[i] = true;
    } else {
      entity_to_point_indices[entity_idx].push_back(i);
    }
  }

  // Process points grouped by entity
  for (size_t entity_idx = 0; entity_idx < raycast_entities.size(); ++entity_idx) {
    const auto & point_indices = entity_to_point_indices[entity_idx];
    if (point_indices.empty()) {
      continue;
    }

    const auto & entity_status = raycast_entities[entity_idx].entity_status;
    auto * config = getConfigFor(entity_status.name(), entity_status);
    if (config == nullptr) {
      continue;
    }

    const auto x = entity_status.pose().position().x() - ego_pose.position.x;
    const auto y = entity_status.pose().position().y() - ego_pose.position.y;
    auto & bin = config->getDistanceBin(x, y);

    // Apply noise to all points from this entity
    for (size_t i : point_indices) {
      auto & point = cloud->points[i];

      // Check if point is detected (true positive rate)
      if (!bin.detection_distribution(random_engine_)) {
        points_to_remove[i] = true;
        continue;
      }

      // Generate noise using pre-configured distributions
      const double radial_noise = bin.radial_distribution(random_engine_);
      const double tangential_noise = bin.tangential_distribution(random_engine_);

      // Apply noise to point coordinates
      const double distance = std::hypot(point.x, point.y, point.z);
      const double radial_x = point.x / distance;
      const double radial_y = point.y / distance;
      const double radial_z = point.z / distance;
      const double noised_distance = distance + radial_noise;

      // Tangential direction: perpendicular to radial in XY plane
      const double tangential_x = radial_y;
      const double tangential_y = -radial_x;

      point.x = static_cast<float>(radial_x * noised_distance + tangential_x * tangential_noise);
      point.y = static_cast<float>(radial_y * noised_distance + tangential_y * tangential_noise);
      point.z = static_cast<float>(radial_z * noised_distance);
    }
  }

  removeMarkedPoints(cloud, points_to_remove);
}

}  // namespace simple_sensor_simulator
