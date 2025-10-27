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

LidarNoiseV1Processor::LidarNoiseV1Processor(const std::string & topic_name, int seed)
: random_engine_(seed),
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
  const auto config_names = noise_parameter_selector::listAvailableNoiseConfigs(topic_name_, "v1");
  for (const auto & config_name : config_names) {
    noise_configs_.try_emplace(config_name, topic_name_, config_name);
  }
}

double LidarNoiseV1Processor::getValueFromEllipse(
  double x, double y, double ellipse_normalized_x_radius,
  const std::vector<double> & ellipse_y_radii_squared, const std::vector<double> & values)
{
  // Ellipse formula: (x/a)^2 + (y/b)^2 < 1
  const double x_normalized = x / ellipse_normalized_x_radius;
  const double distance_squared = x_normalized * x_normalized + y * y;

  for (size_t i = 0; i < ellipse_y_radii_squared.size(); ++i) {
    if (distance_squared < ellipse_y_radii_squared[i]) {
      return values[i];
    }
  }
  return 0.0;
}

const LidarNoiseV1Processor::EntityNoiseConfig * LidarNoiseV1Processor::findOrAssignConfigForEntity(
  const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status)
{
  auto it = entity_to_config_.find(entity_name);
  if (it != entity_to_config_.end()) {
    return it->second;
  }

  const auto config_name =
    noise_parameter_selector::findMatchingNoiseConfigForEntity(entity_status, "v1", topic_name_);

  const EntityNoiseConfig * config_ptr = nullptr;
  if (!config_name.empty()) {
    config_ptr = &noise_configs_.at(config_name);
  }

  entity_to_config_[entity_name] = config_ptr;
  return config_ptr;
}

auto LidarNoiseV1Processor::getNoiseParameters(
  const traffic_simulator_msgs::EntityStatus & entity, const geometry_msgs::msg::Pose & ego_pose)
  -> NoiseParameters
{
  const auto * config = findOrAssignConfigForEntity(entity.name(), entity);
  if (config == nullptr) {
    return NoiseParameters();
  }

  NoiseParameters params;
  params.config_name = config->config_name;

  const auto x = entity.pose().position().x() - ego_pose.position.x;
  const auto y = entity.pose().position().y() - ego_pose.position.y;

  params.radial_distance_mean = getValueFromEllipse(
    x, y, config->radial_distance.mean_ellipse_normalized_x_radius, config->ellipse_y_radii_squared,
    config->radial_distance.mean_values);
  params.radial_distance_std_dev = getValueFromEllipse(
    x, y, config->radial_distance.std_dev_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->radial_distance.std_dev_values);

  params.tangential_distance_mean = getValueFromEllipse(
    x, y, config->tangential_distance.mean_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->tangential_distance.mean_values);
  params.tangential_distance_std_dev = getValueFromEllipse(
    x, y, config->tangential_distance.std_dev_ellipse_normalized_x_radius,
    config->ellipse_y_radii_squared, config->tangential_distance.std_dev_values);

  params.true_positive_rate = getValueFromEllipse(
    x, y, config->true_positive_rate_ellipse_normalized_x_radius, config->ellipse_y_radii_squared,
    config->true_positive_rate_values);

  return params;
}

bool LidarNoiseV1Processor::applyNoiseToPoint(
  pcl::PointXYZI & point, const NoiseParameters & params)
{
  // Check if point is detected (true positive rate)
  if (uniform_distribution_(random_engine_) >= params.true_positive_rate) {
    return true;  // Mark for removal (false negative)
  }

  // Generate radial and tangential noise
  radial_distribution_.param(std::normal_distribution<double>::param_type(
    params.radial_distance_mean, params.radial_distance_std_dev));
  const double radial_noise = radial_distribution_(random_engine_);

  tangential_distribution_.param(std::normal_distribution<double>::param_type(
    params.tangential_distance_mean, params.tangential_distance_std_dev));
  const double tangential_noise = tangential_distribution_(random_engine_);

  // Apply noise to point coordinates
  const double distance = std::hypot(point.x, point.y, point.z);
  if (distance > 0.0) {
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

  return false;  // Keep point
}

void LidarNoiseV1Processor::removeMarkedPoints(
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

void LidarNoiseV1Processor::applyNoise(
  Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose)
{
  auto & cloud = result.cloud;
  const auto & point_entity_indices = result.point_to_entity_index;
  const auto & raycast_entities = result.raycast_entities;

  if (!cloud || cloud->empty() || point_entity_indices.size() != cloud->size()) {
    return;
  }

  // Pre-compute noise parameters for each entity (optimization)
  std::vector<NoiseParameters> entity_idx_to_params(raycast_entities.size());
  for (size_t entity_idx = 0; entity_idx < raycast_entities.size(); ++entity_idx) {
    const auto & entity_status = raycast_entities[entity_idx].entity_status;
    const auto * config = findOrAssignConfigForEntity(entity_status.name(), entity_status);
    if (config != nullptr) {
      entity_idx_to_params[entity_idx] = getNoiseParameters(entity_status, ego_pose);
    }
  }

  // Apply noise to each point
  const size_t cloud_size = cloud->size();
  std::vector<bool> points_to_remove(cloud_size, false);

  for (size_t i = 0; i < cloud_size; ++i) {
    auto & point = cloud->points[i];
    const size_t entity_idx = point_entity_indices[i];

    if (entity_idx >= raycast_entities.size()) {
      points_to_remove[i] = true;
      continue;
    }

    const auto & params = entity_idx_to_params[entity_idx];
    if (params.config_name.empty()) {
      continue;
    }

    points_to_remove[i] = applyNoiseToPoint(point, params);
  }

  removeMarkedPoints(cloud, points_to_remove);
}

}  // namespace simple_sensor_simulator
