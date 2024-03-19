/**
 * @file traffic_source.cpp
 * @author Mateusz Palczuk (mateusz.palczuk@robotec.ai)
 * @brief implementation of the TrafficSource class
 * @version 0.1
 * @date 2024-03-14
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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
: radius(radius),
  rate(rate),
  speed(speed),
  position(position),
  source_id(next_source_id++),
  spawn_function(spawn_function),
  hdmap_utils(hdmap_utils),
  spawnable_lanelets(
    /// @note hardcoded parameter
    hdmap_utils->getNearbyLaneletIds(position, radius, false, static_cast<std::size_t>(1000))),
  id_distribution(0, spawnable_lanelets.size() - 1)
{
  if (spawnable_lanelets.empty()) {
    THROW_SIMULATION_ERROR("TrafficSource ", source_id, " has no spawnable lanelets.");
  }
  for (const auto & id : spawnable_lanelets) {
    s_distributions.emplace(
      id, std::uniform_real_distribution<double>(getSmallestSValue(id), getBiggestSValue(id)));
  }
  if (random_seed) {
    engine.seed(random_seed.value());
  }
}

auto TrafficSource::getSmallestSValue(const lanelet::Id id) -> double
{
  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = id;
  /// @note hardcoded parameter, sampling the lanelet in 0.1m
  for (pose.s = 0.0; pose.s < hdmap_utils->getLaneletLength(id); pose.s += 0.1) {
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
  /// @note hardcoded parameter, sampling the lanelet in 0.1m
  for (pose.s = hdmap_utils->getLaneletLength(id); pose.s > 0.0; pose.s -= 0.1) {
    if (convertToPoseInArea(pose)) {
      return pose.s;
    }
  }
  return hdmap_utils->getLaneletLength(id);
}

auto TrafficSource::convertToPoseInArea(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto map_pose = hdmap_utils->toMapPose(lanelet_pose).pose;
  const double distance2D =
    std::hypot(map_pose.position.x - position.x, map_pose.position.y - position.y);
  if (distance2D <= radius) {
    return map_pose;
  }
  return std::nullopt;
}

auto TrafficSource::getNewEntityName() -> std::string
{
  return "traffic_source_" + std::to_string(source_id) + "_entity_" + std::to_string(entity_id++);
}

void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  if (current_time - last_spawn_time < 1.0 / rate) {
    return;
  }
  last_spawn_time = current_time;

  spawn_function(getNewEntityName(), getValidRandomPose(), speed);
}

auto TrafficSource::getValidRandomPose() -> geometry_msgs::msg::Pose
{
  /// @note hardcoded parameter
  for (int tries = 0; tries < 1e5; ++tries) {
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
  return spawnable_lanelets.at(id_distribution(engine));
}

auto TrafficSource::getRandomSValue(const lanelet::Id id) -> double
{
  if (s_distributions.find(id) == s_distributions.end()) {
    THROW_SIMULATION_ERROR(
      "TrafficSource::getRandomSValue failed to find random distribution for lanelet_id ", id,
      " this should not happen. Please contact the developer.");
  }
  return s_distributions.at(id)(engine);
}
}  // namespace traffic
}  // namespace traffic_simulator
