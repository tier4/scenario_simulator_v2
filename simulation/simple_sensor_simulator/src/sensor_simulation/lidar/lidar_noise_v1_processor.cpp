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
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_noise_v1_processor.hpp>
#include <simple_sensor_simulator/sensor_simulation/noise_parameter_selector.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{

LidarNoiseV1Processor::LidarNoiseV1Processor(
  const std::string & topic_name, int noise_model_version, int seed)
: random_engine_(seed),
  noise_model_version_(noise_model_version),
  topic_name_(topic_name),
  uniform_distribution_(0.0, 1.0),
  radial_distribution_(0.0, 1.0),
  tangential_distribution_(0.0, 1.0)
{
  loadAllNoiseConfigs();
}

LidarNoiseV1Processor::DistanceNoiseParams::DistanceNoiseParams(
  const std::string & param_base_path, const std::string & direction_name)
: mean_ellipse_normalized_x_radius(common::getParameter<double>(
    param_base_path + "distance." + direction_name + ".mean.ellipse_normalized_x_radius")),
  mean_values(common::getParameter<std::vector<double>>(
    param_base_path + "distance." + direction_name + ".mean.values")),
  std_dev_ellipse_normalized_x_radius(common::getParameter<double>(
    param_base_path + "distance." + direction_name +
    ".standard_deviation.ellipse_normalized_x_radius")),
  std_dev_values(common::getParameter<std::vector<double>>(
    param_base_path + "distance." + direction_name + ".standard_deviation.values"))
{
}

LidarNoiseV1Processor::EntityNoiseConfig::EntityNoiseConfig(
  const std::string & topic_name, const std::string & config_name)
: config_name(config_name),
  parameter_base_path(topic_name + ".noise.v1." + config_name + "."),
  ellipse_y_radii(
    common::getParameter<std::vector<double>>(parameter_base_path + "ellipse_y_radii")),
  ellipse_y_radii_squared([this]() {
    std::vector<double> squared;
    squared.reserve(ellipse_y_radii.size());
    for (const auto & radius : ellipse_y_radii) {
      squared.push_back(radius * radius);
    }
    return squared;
  }()),
  radial_distance(parameter_base_path, "radial"),
  tangential_distance(parameter_base_path, "tangential"),
  true_positive_rate_ellipse_normalized_x_radius(common::getParameter<double>(
    parameter_base_path + "true_positive.rate.ellipse_normalized_x_radius")),
  true_positive_rate_values(
    common::getParameter<std::vector<double>>(parameter_base_path + "true_positive.rate.values"))
{
}

void LidarNoiseV1Processor::loadAllNoiseConfigs()
{
  // Get all config names using common utility
  const auto config_names = noise_parameter_selector::listAvailableNoiseConfigs(topic_name_, "v1");

  // Load parameters for each config
  for (const auto & config_name : config_names) {
    noise_configs_.try_emplace(config_name, topic_name_, config_name);
  }
}

auto LidarNoiseV1Processor::getNoiseParameters(
  const traffic_simulator_msgs::EntityStatus & entity, const geometry_msgs::msg::Pose & ego_pose)
  -> NoiseParameters
{
  // Find or assign config for this entity
  auto it = entity_to_config_.find(entity.name());
  if (it == entity_to_config_.end()) {
    // First time seeing this entity - find matching config
    const auto config_name =
      noise_parameter_selector::findMatchingNoiseConfigForEntity(entity, "v1", topic_name_);
    if (config_name.empty()) {
      // No noise configuration for this entity
      it = entity_to_config_.emplace(entity.name(), nullptr).first;
    } else {
      it = entity_to_config_.emplace(entity.name(), &noise_configs_.at(config_name)).first;
    }
  }

  const auto * config = it->second;
  if (config == nullptr) {
    // No noise configuration for this entity - return default parameters
    return NoiseParameters();
  }

  // Compute noise parameters based on entity position
  NoiseParameters params;
  params.config_name = config->config_name;

  const auto x = entity.pose().position().x() - ego_pose.position.x;
  const auto y = entity.pose().position().y() - ego_pose.position.y;

  // Helper lambda to get value from cached parameters based on elliptical distance
  // Uses squared distance comparison to avoid sqrt
  auto get_value_from_ellipse = [](
                                  double x, double y, double ellipse_normalized_x_radius,
                                  const std::vector<double> & ellipse_y_radii_squared,
                                  const std::vector<double> & values) -> double {
    const double x_normalized = x / ellipse_normalized_x_radius;
    const double distance_squared = x_normalized * x_normalized + y * y;
    for (size_t i = 0; i < ellipse_y_radii_squared.size(); ++i) {
      if (distance_squared < ellipse_y_radii_squared[i]) {
        return values[i];
      }
    }
    return 0.0;
  };

  // Get radial distance noise parameters from cached values
  params.radial_distance_mean = get_value_from_ellipse(
    x, y, config->radial_distance.mean_ellipse_normalized_x_radius, config->ellipse_y_radii_squared,
    config->radial_distance.mean_values);
  params.radial_distance_std_dev = get_value_from_ellipse(
    x, y, config->radial_distance.std_dev_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->radial_distance.std_dev_values);

  // Get tangential distance noise parameters from cached values
  params.tangential_distance_mean = get_value_from_ellipse(
    x, y, config->tangential_distance.mean_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->tangential_distance.mean_values);
  params.tangential_distance_std_dev = get_value_from_ellipse(
    x, y, config->tangential_distance.std_dev_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->tangential_distance.std_dev_values);

  // Get true_positive rate from cached values
  params.true_positive_rate = get_value_from_ellipse(
    x, y, config->true_positive_rate_ellipse_normalized_x_radius, config->ellipse_y_radii_squared,
    config->true_positive_rate_values);

  return params;
}

bool LidarNoiseV1Processor::applyNoiseToPoint(
  pcl::PointXYZI & point, const NoiseParameters & params)
{
  // Check true_positive rate first (using reusable uniform distribution)
  const bool true_positive = uniform_distribution_(random_engine_) < params.true_positive_rate;

  if (!true_positive) {
    // Should be removed based on true_positive rate
    return true;
  }

  // Generate noise for THIS POINT independently (update distribution parameters)
  radial_distribution_.param(std::normal_distribution<double>::param_type(
    params.radial_distance_mean, params.radial_distance_std_dev));
  const double radial_noise = radial_distribution_(random_engine_);

  tangential_distribution_.param(std::normal_distribution<double>::param_type(
    params.tangential_distance_mean, params.tangential_distance_std_dev));
  const double tangential_noise = tangential_distribution_(random_engine_);

  // Calculate radial vector (from sensor to point)
  const double distance = std::hypot(point.x, point.y, point.z);
  if (distance > 0.0) {
    // Normalized radial direction
    const double radial_x = point.x / distance;
    const double radial_y = point.y / distance;
    const double radial_z = point.z / distance;

    // Apply radial noise along the ray direction
    const double noised_distance = distance + radial_noise;

    // Tangential direction: 90-degree clockwise rotation in XY plane
    // For radial vector (radial_x, radial_y) in XY plane,
    // tangential is (radial_y, -radial_x) for clockwise rotation
    const double tangential_x = radial_y;
    const double tangential_y = -radial_x;
    const double tangential_z = 0.0;

    // Apply both radial and tangential noise
    point.x = static_cast<float>(radial_x * noised_distance + tangential_x * tangential_noise);
    point.y = static_cast<float>(radial_y * noised_distance + tangential_y * tangential_noise);
    point.z = static_cast<float>(radial_z * noised_distance + tangential_z * tangential_noise);
  }

  // Point is kept (with noise applied)
  return false;
}

void LidarNoiseV1Processor::applyNoise(
  Raycaster::RaycastResult & result, [[maybe_unused]] double simulation_time,
  const geometry_msgs::msg::Pose & ego_pose)
{
  auto & cloud = result.cloud;
  const auto & point_entity_indices = result.point_to_entity_index;
  const auto & raycast_entities = result.raycast_entities;

  if (!cloud || cloud->empty() || point_entity_indices.size() != cloud->size()) {
    return;
  }

  // Build index-based config lookup for fast access in point loop
  std::vector<const EntityNoiseConfig *> entity_idx_to_config(raycast_entities.size(), nullptr);

  for (size_t entity_idx = 0; entity_idx < raycast_entities.size(); ++entity_idx) {
    const auto & entity_status = raycast_entities[entity_idx].entity_status;
    const auto & entity_name = entity_status.name();

    // Check if this entity has noise config (avoid repeated lookups in point loop)
    auto it = entity_to_config_.find(entity_name);
    if (it == entity_to_config_.end()) {
      // First time seeing this entity - find matching config
      const auto config_name = noise_parameter_selector::findMatchingNoiseConfigForEntity(
        entity_status, "v1", topic_name_);
      if (config_name.empty()) {
        entity_to_config_[entity_name] = nullptr;
        entity_idx_to_config[entity_idx] = nullptr;
      } else {
        const auto * config_ptr = &noise_configs_.at(config_name);
        entity_to_config_[entity_name] = config_ptr;
        entity_idx_to_config[entity_idx] = config_ptr;
      }
    } else {
      entity_idx_to_config[entity_idx] = it->second;
    }
  }

  // Apply noise to points
  const size_t cloud_size = cloud->size();
  std::vector<bool> points_to_remove(cloud_size, false);

  for (size_t i = 0; i < cloud_size; ++i) {
    auto & point = cloud->points[i];
    const size_t entity_idx = point_entity_indices[i];

    if (entity_idx >= raycast_entities.size()) {
      // Invalid entity_idx, remove point
      points_to_remove[i] = true;
      continue;
    }

    if (entity_idx_to_config[entity_idx] == nullptr) {
      // No noise configuration for this entity, leave point as-is
      continue;
    }

    const auto & entity_status = raycast_entities[entity_idx].entity_status;
    const auto params = getNoiseParameters(entity_status, ego_pose);

    // Apply noise and check if point should be removed
    points_to_remove[i] = applyNoiseToPoint(point, params);
  }

  // Remove marked points in-place efficiently
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

}  // namespace simple_sensor_simulator
