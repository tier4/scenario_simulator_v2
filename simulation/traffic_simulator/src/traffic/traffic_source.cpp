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

#include <geometry/intersection/collision.hpp>
#include <geometry/vector3/hypot.hpp>
#include <traffic_simulator/helper/helper.hpp>
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
  const geometry_msgs::msg::Pose & position,
  const std::vector<std::pair<std::variant<VehicleParams, PedestrianParams>, double>> & params,
  const std::optional<int> random_seed, const double current_time,
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const VehicleParams &, const double)> &
    vehicle_spawn_function,
  const std::function<void(
    const std::string &, const geometry_msgs::msg::Pose &, const PedestrianParams &,
    const double)> & pedestrian_spawn_function,
  const Configuration & configuration, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils)
: radius_(radius),
  rate_(rate),
  speed_(speed),
  source_pose_(position),
  source_id_(next_source_id_++),
  vehicle_spawn_function_(vehicle_spawn_function),
  pedestrian_spawn_function_(pedestrian_spawn_function),
  hdmap_utils_(hdmap_utils),
  spawnable_lanelets_(hdmap_utils->getNearbyLaneletIds(
    position.position, radius, false, static_cast<std::size_t>(spawnable_lanes_limit))),
  id_distribution_(0, spawnable_lanelets_.size() - 1),
  angle_distribution_(0.0, M_PI * 2.0),
  radius_distribution_(0.0, radius_),
  start_execution_time_(
    /// @note Failsafe for the case where TrafficSource is added before the simulation starts or delay is negative
    (!std::isnan(current_time) ? current_time : 0.0) + std::max(configuration.start_delay, 0.0)),
  config_(configuration)
{
  std::transform(
    params.begin(), params.end(), std::back_inserter(params_),
    [](const std::pair<std::variant<VehicleParams, PedestrianParams>, double> & pair) {
      return pair.first;
    });

  std::vector<double> weights;
  std::transform(
    params.begin(), params.end(), std::back_inserter(weights),
    [](const std::pair<std::variant<VehicleParams, PedestrianParams>, double> & pair) {
      return pair.second;
    });
  params_distribution_ = std::discrete_distribution<>(weights.begin(), weights.end());

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

auto TrafficSource::getRandomPose(const bool random_orientation) -> geometry_msgs::msg::Pose
{
  const double angle = angle_distribution_(engine_);
  const double radius = radius_distribution_(engine_);

  geometry_msgs::msg::Pose pose;
  /// @todo add orientation from TrafficSource orientation when it will have orientation
  pose = source_pose_;
  pose.position.x += radius * std::cos(angle);
  pose.position.y += radius * std::sin(angle);

  if (random_orientation) {
    pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
      traffic_simulator::helper::constructRPY(0.0, 0.0, angle_distribution_(engine_)));
  }

  return pose;
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
  const double distance2D = std::hypot(
    map_pose.position.x - source_pose_.position.x, map_pose.position.y - source_pose_.position.y);
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
  /// @note For now this mechanism allows for the spawn rate to be as high as the rate at which this function is called
  if (current_time - start_execution_time_ < 1.0 / rate_ * spawn_count_) {
    return;
  }
  ++spawn_count_;

  randomizeParams();
  if (isPedestrian(*current_params_)) {
    pedestrian_spawn_function_(
      getNewEntityName(), getValidRandomPose(), std::get<PedestrianParams>(*current_params_),
      speed_);
  } else {
    vehicle_spawn_function_(
      getNewEntityName(), getValidRandomPose(), std::get<VehicleParams>(*current_params_), speed_);
  }
}

auto TrafficSource::randomizeParams() -> void
{
  const auto current_params_idx = params_distribution_(engine_);
  current_params_ = std::next(params_.begin(), current_params_idx);
}

auto TrafficSource::isPedestrian(const std::variant<VehicleParams, PedestrianParams> & params)
  -> bool
{
  return std::holds_alternative<PedestrianParams>(params);
}

auto TrafficSource::getCurrentBoundingBox() -> traffic_simulator_msgs::msg::BoundingBox
{
  if (isPedestrian(*current_params_)) {
    return std::get<PedestrianParams>(*current_params_).bounding_box;
  }
  return std::get<VehicleParams>(*current_params_).bounding_box;
}

auto TrafficSource::isPoseValid(const geometry_msgs::msg::Pose & pose) -> bool
{
  /// @note transform bounding box corners to world coordinate system
  auto bbox_corners = math::geometry::getPointsFromBbox(getCurrentBoundingBox());

  tf2::Transform ref_transform;
  tf2::fromMsg(pose, ref_transform);

  std::for_each(
    bbox_corners.begin(), bbox_corners.end(), [&](geometry_msgs::msg::Point & corner) -> void {
      tf2::Vector3 tf2_corner(corner.x, corner.y, corner.z);
      const tf2::Vector3 corner_transformed = ref_transform * tf2_corner;
      corner.x = corner_transformed.x();
      corner.y = corner_transformed.y();
      corner.z = corner_transformed.z();
    });

  const auto is_outside_spawnable_area = [&](const geometry_msgs::msg::Point & corner) -> bool {
    return math::geometry::hypot(corner, source_pose_.position) > radius_;
  };

  /// @note Step 1: check whether all corners are inside spawnable area
  if (
    std::find_if_not(bbox_corners.begin(), bbox_corners.end(), is_outside_spawnable_area) ==
    bbox_corners.end()) {
    return false;
  }

  /// @note Step 2: check whether can be outside lanelet
  if (config_.allow_spawn_outside_lane) {
    return true;
  }

  const auto lanelet_pose = hdmap_utils_->toLaneletPose(pose, isPedestrian(*current_params_));
  if (lanelet_pose) {
    /// @note Step 3: check whether the bounding box can be outside lanelet
    if (!config_.require_footprint_fitting) {
      return true;
    }
    /// @note Step 4: check whether the bounding box fits inside the lanelet
    /// @todo enable footprint fitting
    THROW_SPECIFICATION_VIOLATION("TrafficSource footprint validation is not supported yet");
  }
  return false;
}

auto TrafficSource::getValidRandomPose() -> geometry_msgs::msg::Pose
{
  for (int tries = 0; tries < max_randomization_attempts; ++tries) {
    const auto candidate_pose = getRandomPose(config_.use_random_orientation);
    if (isPoseValid(candidate_pose)) {
      return candidate_pose;
    }
  }
  THROW_SIMULATION_ERROR("TrafficSource failed to get valid random pose.");
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
