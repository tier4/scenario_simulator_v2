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
constexpr int max_randomization_attempts = 1e5;

namespace traffic_simulator
{
namespace traffic
{
SpawnPoseValidator::LaneletArea::LaneletArea(
  const lanelet::Id & id, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils)
: first_id(id),
  last_id(id),
  ids({id}),
  left_bound(hdmap_utils->getLeftBound(id)),
  right_bound(hdmap_utils->getRightBound(id))
{
}

auto SpawnPoseValidator::LaneletArea::operator+(const LaneletArea & other) const -> LaneletArea
{
  return combine(*this, other);
}

auto SpawnPoseValidator::LaneletArea::combine(const LaneletArea & before, const LaneletArea & after)
  -> LaneletArea
{
  LaneletArea ret;
  ret.first_id = before.first_id;
  ret.last_id = after.last_id;

#define COMBINE_VECTOR(RET, VEC1, VEC2)            \
  RET.reserve(VEC1.size() + VEC2.size());          \
  RET.insert(RET.end(), VEC1.begin(), VEC1.end()); \
  RET.insert(RET.end(), VEC2.begin(), VEC2.end());

  COMBINE_VECTOR(ret.ids, before.ids, after.ids);
  COMBINE_VECTOR(ret.left_bound, before.left_bound, after.left_bound);
  COMBINE_VECTOR(ret.right_bound, before.right_bound, after.right_bound);

#undef COMBINE_VECTOR

  return ret;
}

auto SpawnPoseValidator::LaneletArea::contains(const LaneletArea & other) const -> bool
{
  const std::set<lanelet::Id> this_ids_set(ids.begin(), ids.end());
  const std::set<lanelet::Id> other_ids_set(other.ids.begin(), other.ids.end());
  return std::includes(
    this_ids_set.begin(), this_ids_set.end(), other_ids_set.begin(), other_ids_set.end());
}

auto SpawnPoseValidator::LaneletArea::contains(const lanelet::Id & id) const -> bool
{
  for (const auto & this_id : this->ids) {
    if (id == this_id) {
      return true;
    }
  }
  return false;
}

auto SpawnPoseValidator::LaneletArea::toBoostPolygon(
  const std::vector<geometry_msgs::msg::Point> & points) -> boost_polygon
{
  boost_polygon poly;
  poly.outer().reserve(points.size() + 1);

  std::transform(
    points.begin(), points.end(), std::back_inserter(poly.outer()),
    [](const geometry_msgs::msg::Point & p) { return math::geometry::toBoostPoint(p); });

  poly.outer().push_back(poly.outer().front());
  return poly;
}

auto SpawnPoseValidator::LaneletArea::getBoostPolygon() const -> const boost_polygon &
{
  // recalculate only when necessary (+1 because boost polygon is closed)
  if (polygon_b.outer().size() != getPolygon().size() + static_cast<std::size_t>(1)) {
    polygon_b = toBoostPolygon(getPolygon());
  }
  return polygon_b;
}

auto SpawnPoseValidator::LaneletArea::getPolygon() const
  -> const std::vector<geometry_msgs::msg::Point> &
{
  if (left_bound.empty() || right_bound.empty()) {
    polygon.clear();
    return polygon;
  }
  if (polygon.size() != left_bound.size() + right_bound.size()) {
    createPolygon();
  }
  return polygon;
}

void SpawnPoseValidator::LaneletArea::createPolygon() const
{
  polygon.clear();
  polygon.reserve(left_bound.size() + right_bound.size());
  polygon.insert(polygon.end(), left_bound.begin(), left_bound.end());
  polygon.insert(polygon.end(), right_bound.rbegin(), right_bound.rend());
}

SpawnPoseValidator::SpawnPoseValidator(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  const geometry_msgs::msg::Pose & source_pose, const double source_radius,
  const bool include_crosswalk)
: hdmap_utils_(hdmap_utils)
{
  const auto spawnable_lanelets = hdmap_utils_->getNearbyLaneletIds(
    source_pose.position, source_radius, include_crosswalk, spawnable_lanes_limit);
  std::set<lanelet::Id> ids(spawnable_lanelets.begin(), spawnable_lanelets.end());

  if (ids.empty()) {
    THROW_SIMULATION_ERROR("TrafficSource has no spawnable lanelets.");
  }

  /// @note start recursive search from every lanelet to find all possibilities
  for (const auto & id : ids) {
    areas_.emplace_back(id, hdmap_utils_);
    findAllSpawnableAreas(id, ids);
  }

  removeRedundantAreas();
}

auto SpawnPoseValidator::isValid(
  const std::vector<geometry_msgs::msg::Point> & polygon, const lanelet::Id & id) const -> bool
{
  const auto boost_polygon = LaneletArea::toBoostPolygon(polygon);

  for (const auto & area : areas_) {
    if (area.contains(id) && boost::geometry::within(boost_polygon, area.getBoostPolygon())) {
      return true;
    }
  }
  return false;
}

void SpawnPoseValidator::findAllSpawnableAreas(
  const lanelet::Id id, const std::set<lanelet::Id> & ids,
  const std::optional<LaneletArea> & previous_area, const bool in_front)
{
  LaneletArea area(id, hdmap_utils_);
  if (previous_area) {
    if (in_front) {
      area = previous_area.value() + area;
    } else {
      area = area + previous_area.value();
    }
  }

  const auto following_ids = hdmap_utils_->getFollowingLanelets(area.last_id);
  for (const auto & following_id : following_ids) {
    if (following_id != area.last_id && ids.count(following_id) == 1) {
      areas_.push_back(area);
      findAllSpawnableAreas(following_id, ids, area, true);
    }
  }

  const auto previous_ids = hdmap_utils_->getPreviousLanelets(area.first_id);
  for (const auto & previous_id : previous_ids) {
    if (previous_id != area.first_id && ids.count(previous_id) == 1) {
      areas_.push_back(area);
      findAllSpawnableAreas(previous_id, ids, area, false);
    }
  }
}

void SpawnPoseValidator::removeRedundantAreas()
{
  for (auto it = areas_.begin(); it != areas_.end(); ++it) {
    for (auto other_it = areas_.begin(); other_it != areas_.end(); /* do not increment here! */) {
      if (it != other_it && it->contains(*other_it)) {
        other_it = areas_.erase(other_it);
      } else {
        ++other_it;
      }
    }
  }
}

TrafficSource::TrafficSource(
  const double radius, const double rate, const double speed,
  const geometry_msgs::msg::Pose & position,
  const std::vector<std::tuple<ParamsVariant, std::string, std::string, double>> & params,
  const std::optional<int> random_seed, const double current_time,
#define MAKE_FUNCTION_REF_TYPE(PARAMST, POSET)                                                     \
  const std::function<void(                                                                        \
    const std::string &, const POSET &, const PARAMST &, const std::string &, const std::string &, \
    const double)> &
  // clang-format off
  MAKE_FUNCTION_REF_TYPE(VehicleParams,    CanonicalizedLaneletPose) vehicle_ll_spawn_function,
  MAKE_FUNCTION_REF_TYPE(PedestrianParams, CanonicalizedLaneletPose) pedestrian_ll_spawn_function,
  MAKE_FUNCTION_REF_TYPE(VehicleParams,    geometry_msgs::msg::Pose) vehicle_spawn_function,
  MAKE_FUNCTION_REF_TYPE(PedestrianParams, geometry_msgs::msg::Pose) pedestrian_spawn_function,
// clang-format on
#undef MAKE_FUNCTION_REF_TYPE
  const Configuration & configuration, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils)
: radius_(radius),
  rate_(rate),
  speed_(speed),
  source_pose_(position),
  source_id_(next_source_id_++),
  vehicle_ll_spawn_function_(vehicle_ll_spawn_function),
  pedestrian_ll_spawn_function_(pedestrian_ll_spawn_function),
  vehicle_spawn_function_(vehicle_spawn_function),
  pedestrian_spawn_function_(pedestrian_spawn_function),
  hdmap_utils_(hdmap_utils),
  angle_distribution_(0.0, M_PI * 2.0),
  radius_distribution_(0.0, radius_),
  start_execution_time_(
    /// @note Failsafe for the case where TrafficSource is added before the simulation starts or delay is negative
    (!std::isnan(current_time) ? current_time : 0.0) + std::max(configuration.start_delay, 0.0)),
  config_(configuration),
  params_(obtainParams<ParamsVariant, 0>(params)),
  behaviors_(obtainParams<std::string, 1>(params)),
  models3d_(obtainParams<std::string, 2>(params)),
  validator_(hdmap_utils, position, radius, false),
  validator_crosswalk_(hdmap_utils, position, radius, true)
{
  // Create a parameter distribution from the weights
  const auto weights = obtainParams<double, 3>(params);
  params_distribution_ = std::discrete_distribution<>(weights.begin(), weights.end());

  if (random_seed) {
    engine_.seed(random_seed.value());
  }
}

auto TrafficSource::getRandomPose(const bool random_orientation) -> geometry_msgs::msg::Pose
{
  const double angle = angle_distribution_(engine_);
  const double radius = radius_distribution_(engine_);

  geometry_msgs::msg::Pose pose = source_pose_;
  pose.position.x += radius * std::cos(angle);
  pose.position.y += radius * std::sin(angle);

  if (random_orientation) {
    pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
      traffic_simulator::helper::constructRPY(0.0, 0.0, angle_distribution_(engine_)));
  }

  return pose;
}

auto TrafficSource::getNewEntityName() -> std::string
{
  return "traffic_source_" + std::to_string(source_id_) + "_entity_" + std::to_string(entity_id_++);
}

void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  while (current_time - start_execution_time_ > 1.0 / rate_ * spawn_count_) {
    ++spawn_count_;

    randomizeCurrentParams();
    const auto name = getNewEntityName();
    const auto [pose, lanelet_pose] = getValidRandomPose();

    if (lanelet_pose) {
      if (isCurrentPedestrian()) {
        pedestrian_ll_spawn_function_(
          name, lanelet_pose.value(), std::get<PedestrianParams>(*current_params_),
          *current_behavior_, *current_model3d_, speed_);
      } else {
        vehicle_ll_spawn_function_(
          name, lanelet_pose.value(), std::get<VehicleParams>(*current_params_), *current_behavior_,
          *current_model3d_, speed_);
      }
    } else {
      if (isCurrentPedestrian()) {
        pedestrian_spawn_function_(
          name, pose, std::get<PedestrianParams>(*current_params_), *current_behavior_,
          *current_model3d_, speed_);
      } else {
        vehicle_spawn_function_(
          name, pose, std::get<VehicleParams>(*current_params_), *current_behavior_,
          *current_model3d_, speed_);
      }
    }
  }
}

auto TrafficSource::randomizeCurrentParams() -> void
{
  const auto current_params_idx = params_distribution_(engine_);
  current_params_ = std::next(params_.begin(), current_params_idx);
  current_behavior_ = std::next(behaviors_.begin(), current_params_idx);
  current_model3d_ = std::next(models3d_.begin(), current_params_idx);
}

auto TrafficSource::isPedestrian(const ParamsVariant & params) -> bool
{
  return std::holds_alternative<PedestrianParams>(params);
}

auto TrafficSource::isCurrentPedestrian() -> bool { return isPedestrian(*current_params_); }

auto TrafficSource::getCurrentBoundingBox() -> traffic_simulator_msgs::msg::BoundingBox
{
  if (isCurrentPedestrian()) {
    return std::get<PedestrianParams>(*current_params_).bounding_box;
  }
  return std::get<VehicleParams>(*current_params_).bounding_box;
}

auto TrafficSource::isPoseValid(
  geometry_msgs::msg::Pose & pose, std::optional<CanonicalizedLaneletPose> & out_pose) -> bool
{
  out_pose = std::nullopt;
  /// @note transform bounding box corners to world coordinate system
  const auto bbox_corners = math::geometry::getPointsFromBbox(getCurrentBoundingBox());
  auto bbox_corners_transformed = math::geometry::transformPoints(pose, bbox_corners);

  /// @note 2D validation - does not account for height
  const auto is_outside_spawnable_area = [&](const geometry_msgs::msg::Point & corner) -> bool {
    return std::hypot(corner.x - source_pose_.position.x, corner.y - source_pose_.position.y) >
           radius_;
  };

  /// @note Step 1: check whether all corners are inside spawnable area
  if (const auto first_corner_outside = std::find_if(
        bbox_corners_transformed.begin(), bbox_corners_transformed.end(),
        is_outside_spawnable_area);
      first_corner_outside != bbox_corners_transformed.end()) {
    return false;
  }

  /// @note Step 2: check whether can be outside lanelet
  if (config_.allow_spawn_outside_lane) {
    return true;
  }

  const auto lanelet_pose = hdmap_utils_->toLaneletPose(pose, isCurrentPedestrian());
  if (lanelet_pose) {
    /// @note reset orientation - to align the entity with lane
    auto corrected_pose = lanelet_pose.value();
    corrected_pose.rpy.z = 0.0;
    out_pose.emplace(corrected_pose, hdmap_utils_);
    pose = hdmap_utils_->toMapPose(corrected_pose).pose;

    /// @note Step 3: check whether the bounding box can be outside lanelet
    if (!config_.require_footprint_fitting) {
      return true;
    }

    /// @note Step 4: check whether the bounding box fits inside the lanelet
    bbox_corners_transformed = math::geometry::transformPoints(pose, bbox_corners);
    if (isCurrentPedestrian()) {
      return validator_crosswalk_.isValid(bbox_corners_transformed, corrected_pose.lanelet_id);
    }
    return validator_.isValid(bbox_corners_transformed, corrected_pose.lanelet_id);
  }
  return false;
}

auto TrafficSource::getValidRandomPose()
  -> std::pair<geometry_msgs::msg::Pose, std::optional<CanonicalizedLaneletPose>>
{
  for (int tries = 0; tries < max_randomization_attempts; ++tries) {
    auto candidate_pose = getRandomPose(config_.use_random_orientation);
    std::optional<CanonicalizedLaneletPose> out_pose;
    if (isPoseValid(candidate_pose, out_pose)) {
      return std::make_pair(candidate_pose, out_pose);
    }
  }
  THROW_SIMULATION_ERROR(
    "TrafficSource ", source_id_, " failed to get valid random pose in ",
    max_randomization_attempts, ".");
}
}  // namespace traffic
}  // namespace traffic_simulator
