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

#include <algorithm>
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

LidarNoiseModelV1::Config::DistanceBin & LidarNoiseModelV1::Config::getDistanceBin(
  double x, double y)
{
  assert(distance_bins_.empty());
  const double x_normalized = x / true_positive_rate_ellipse_normalized_x_radius;
  const double distance_squared = x_normalized * x_normalized + y * y;

  for (size_t i = 0; i < distance_bins_.size(); ++i) {
    if (distance_squared < distance_bins_[i].ellipse_y_radius_squared) {
      return distance_bins_[i];
    }
  }
  return distance_bins_[distance_bins_.size() - 1];
}

std::optional<std::reference_wrapper<LidarNoiseModelV1::Config>> LidarNoiseModelV1::getConfigFor(
  const std::string & entity_name, const traffic_simulator_msgs::EntityStatus & entity_status)
{
  if (auto config_itr = entity_to_config_.find(entity_name);
      config_itr != entity_to_config_.end()) {
    return config_itr->second;
  } else {
    std::optional<std::reference_wrapper<Config>> result = std::nullopt;
    if (const auto config_name = noise_parameter_selector::findMatchingNoiseConfigForEntity(
          entity_status, "v1", topic_name_);
        not config_name.empty()) {
      result = std::ref(configs_.at(config_name));
    }

    // cache result
    entity_to_config_[entity_name] = result;
    return result;
  }
}

void LidarNoiseModelV1::removeMarkedPoints(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const std::vector<bool> & points_to_remove)
{
  size_t index = 0;
  auto new_end = std::remove_if(cloud->points.begin(), cloud->points.end(), [&](const auto &) {
    return points_to_remove[index++];
  });

  cloud->points.erase(new_end, cloud->points.end());
  cloud->width = cloud->points.size();
  cloud->height = 1;
}

void LidarNoiseModelV1::applyNoise(
  Raycaster::RaycastResult & result, const geometry_msgs::msg::Pose & ego_pose)
{
  auto & [cloud, point_entity_indices, raycast_entities] = result;

  if (not cloud or cloud->empty() or point_entity_indices.size() != cloud->size()) {
    return;
  }

  std::vector<bool> points_to_remove(cloud->size(), false);

  std::vector<std::vector<size_t>> entity_to_point_indices(raycast_entities.size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (const size_t entity_index = point_entity_indices[i];
        entity_index >= raycast_entities.size()) {
      points_to_remove[i] = true;
    } else {
      entity_to_point_indices[entity_index].push_back(i);
    }
  }

  for (size_t entity_idx = 0; entity_idx < raycast_entities.size(); ++entity_idx) {
    const auto & point_indices = entity_to_point_indices[entity_idx];
    if (point_indices.empty()) {
      continue;
    }

    const auto & entity_status = raycast_entities[entity_idx].entity_status;
    auto config = getConfigFor(entity_status.name(), entity_status);
    if (not config.has_value()) {
      continue;
    }

    const auto x = entity_status.pose().position().x() - ego_pose.position.x;
    const auto y = entity_status.pose().position().y() - ego_pose.position.y;
    auto & bin = config->get().getDistanceBin(x, y);

    // Apply noise to all points from this entity
    for (size_t i : point_indices) {
      if (not bin.detection_distribution(random_engine_)) {
        points_to_remove[i] = true;
      } else {
        auto & point = cloud->points[i];
        const double radial_noise = bin.radial_distribution(random_engine_);
        const double tangential_noise = bin.tangential_distribution(random_engine_);

        const double distance = std::hypot(point.x, point.y, point.z);
        const double radial_x = point.x / distance;
        const double radial_y = point.y / distance;
        const double radial_z = point.z / distance;
        const double tangential_x = radial_y;
        const double tangential_y = -radial_x;

        point.x += radial_x * radial_noise + tangential_x * tangential_noise;
        point.y += radial_y * radial_noise + tangential_y * tangential_noise;
        point.z += radial_z * radial_noise;
      }
    }
  }

  removeMarkedPoints(cloud, points_to_remove);
}
}  // namespace simple_sensor_simulator
