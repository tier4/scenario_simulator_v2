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

#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

constexpr std::size_t spawnable_lanes_limit = 1e3;
constexpr double lanelet_sampling_step = 0.1;
constexpr int max_randomization_attempts = 1e4;

namespace traffic_simulator
{
namespace traffic
{
TrafficSource::TrafficSource(
  const double radius, const double rate, const double speed,
  const geometry_msgs::msg::Point & position, const std::optional<int> random_seed,
  const std::function<void(const std::string &, const geometry_msgs::msg::Pose &, const double)> &
    spawn_function,
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils)
: radius_(radius),
  rate_(rate),
  speed_(speed),
  position_(position),
  source_id_(next_source_id_++),
  spawn_function_(spawn_function),
  hdmap_utils_(hdmap_utils),
  spawnable_lanelets_(hdmap_utils->getNearbyLaneletIds(
    position, radius, false, static_cast<std::size_t>(spawnable_lanes_limit))),
  id_distribution_(0, spawnable_lanelets_.size() - 1)
{
  if (spawnable_lanelets_.empty()) {
    THROW_SIMULATION_ERROR("TrafficSource ", source_id_, " has no spawnable lanelets.");
  }
  for (const auto & id : spawnable_lanelets_) {
    s_distributions_.emplace(
      id, std::uniform_real_distribution<double>(getSmallestSValue(id), getBiggestSValue(id)));
  }
  if (random_seed) {
    engine_.seed(random_seed.value());
  }
}

auto TrafficSource::getSmallestSValue(const lanelet::Id id) -> double
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = id;
  for (pose.s = 0.0; pose.s < hdmap_utils_->getLaneletLength(id); pose.s += lanelet_sampling_step) {
    if (convertToPoseInArea(pose)) {
      return pose.s;
    }
  }
  return 0.0;
}

auto TrafficSource::getBiggestSValue(const lanelet::Id id) -> double
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = id;
  for (pose.s = hdmap_utils_->getLaneletLength(id); pose.s > 0.0; pose.s -= lanelet_sampling_step) {
    if (convertToPoseInArea(pose)) {
      return pose.s;
    }
  }
  return hdmap_utils_->getLaneletLength(id);
}

auto TrafficSource::convertToPoseInArea(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto map_pose = hdmap_utils_->toMapPose(lanelet_pose).pose;
  const double distance2D =
    std::hypot(map_pose.position.x - position_.x, map_pose.position.y - position_.y);
  if (distance2D <= radius_) {
    return map_pose;
  }
  return std::nullopt;
}

auto TrafficSource::getNewEntityName() -> std::string
{
  return "traffic_source_" + std::to_string(source_id_) + "_entity_" + std::to_string(entity_id_++);
}

void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  if (current_time - last_spawn_time_ < 1.0 / rate_) {
    return;
  }
  last_spawn_time_ = current_time;

  spawn_function_(getNewEntityName(), getValidRandomPose(), speed_);
}

auto TrafficSource::getValidRandomPose() -> geometry_msgs::msg::Pose
{
  for (int tries = 0; tries < max_randomization_attempts; ++tries) {
    const auto candidate_pose = getRandomLaneletPose();
    if (const auto map_pose = convertToPoseInArea(candidate_pose)) {
      return map_pose.value();
    }
  }
  THROW_SIMULATION_ERROR("TrafficSource failed to get valid random pose.");
}

auto TrafficSource::getRandomLaneletPose() -> traffic_simulator_msgs::msg::LaneletPose
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = getRandomLaneletId();
  pose.s = getRandomSValue(pose.lanelet_id);
  return pose;
}

auto TrafficSource::getRandomLaneletId() -> lanelet::Id
{
  return spawnable_lanelets_.at(id_distribution_(engine_));
}

auto TrafficSource::getRandomSValue(const lanelet::Id id) -> double
{
  if (s_distributions_.find(id) == s_distributions_.end()) {
    THROW_SIMULATION_ERROR(
      "TrafficSource::getRandomSValue failed to find random distribution for lanelet_id ", id,
      " this should not happen. Please contact the developer.");
  }
  return s_distributions_.at(id)(engine_);
}
}  // namespace traffic
}  // namespace traffic_simulator
