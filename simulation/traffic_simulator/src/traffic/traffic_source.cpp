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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/vector3/hypot.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace traffic
{
TrafficSource::Validator::Validator(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
  const geometry_msgs::msg::Pose & pose, const double radius, const bool include_crosswalk)
: ids(hdmap_utils->getNearbyLaneletIds(
    pose.position, radius, include_crosswalk, spawning_lanes_limit)),
  lanelets(hdmap_utils->getLanelets(ids))
{
}

auto TrafficSource::Validator::operator()(
  const std::vector<geometry_msgs::msg::Point> & points, lanelet::Id id) const -> bool
{
  const auto points2d = [&]() {
    auto points2d = lanelet::Points2d();
    for (const auto point : points) {
      points2d.emplace_back(lanelet::utils::getId(), point.x, point.y);
    }
    return points2d;
  }();

  /**
   * @note Possibly undesirable behavior
   * This implementation will consider cases like intersections as one big spawning area.
   * If an Entity is positioned in the middle of the intersection (where it does not fit in any of
   * the lanes alone, but it does fit in all of the lanes combined) it will be considered valid.
   */

  /**
   * @note This implementation does not cover the case, when the Entity is on a curve and the curved
   * lanelet bound intersects with en edge of the Entity's bounding box, but all corners are still
   * inside the lanelet bounds.
   * Example:
   *   . ______  .
   *    .|    |   .
   *     |.   |    .
   *     | .  |     .
   *     | .  |     .
   *     |.   |    .
   *    .|    |   .
   *   . |____|  .
   */
  return std::find(ids.begin(), ids.end(), id) != ids.end() and
         std::all_of(points2d.begin(), points2d.end(), [&](const auto & point) {
           return std::any_of(lanelets.begin(), lanelets.end(), [&](const auto & lane) {
             return lanelet::geometry::inside(lane, point);
           });
         });
}

auto TrafficSource::makeRandomPose(const bool random_orientation) -> geometry_msgs::msg::Pose
{
  const double angle = angle_distribution_(engine_);

  const double radius = radius_distribution_(engine_);

  auto random_pose = pose;

  random_pose.position.x += radius * std::cos(angle);
  random_pose.position.y += radius * std::sin(angle);

  if (random_orientation) {
    random_pose.orientation = math::geometry::convertEulerAngleToQuaternion(
      traffic_simulator::helper::constructRPY(0.0, 0.0, angle_distribution_(engine_)));
  }

  return random_pose;
}

void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  for (; current_time - start_execution_time_ > 1.0 / rate * entity_count_; ++entity_count_) {
    const auto index = params_distribution_(engine_);

    const auto & parameter = std::get<0>(distribution_[index]);

    const auto name =
      "TrafficSource_" + std::to_string(id) + "_Entity_" + std::to_string(entity_count_);

    const auto [pose, lanelet_pose] = [&]() {
      static constexpr auto max_randomization_attempts = 10000;

      for (auto tries = 0; tries < max_randomization_attempts; ++tries) {
        auto candidate_pose = makeRandomPose(configuration_.use_random_orientation);
        if (auto [valid, lanelet_pose] = isPoseValid(parameter, candidate_pose); valid) {
          return std::make_pair(candidate_pose, lanelet_pose);
        }
      }
      THROW_SIMULATION_ERROR(
        "TrafficSource ", id, " failed to generate valid random pose in ",
        max_randomization_attempts, ".");
    }();

    if (lanelet_pose) {
      /// @note If lanelet pose is valid spawn using lanelet pose
      if (std::holds_alternative<PedestrianParameter>(parameter)) {
        spawn_pedestrian_in_lane_coordinate(
          name, lanelet_pose.value(), std::get<PedestrianParameter>(parameter),
          std::get<1>(distribution_[index]), std::get<2>(distribution_[index]));
      } else {
        spawn_vehicle_in_lane_coordinate(
          name, lanelet_pose.value(), std::get<VehicleParameter>(parameter),
          std::get<1>(distribution_[index]), std::get<2>(distribution_[index]));
      }
    } else {
      /// @note If lanelet pose is not valid spawn using normal map pose
      if (std::holds_alternative<PedestrianParameter>(parameter)) {
        spawn_pedestrian_in_world_coordinate(
          name, pose, std::get<PedestrianParameter>(parameter), std::get<1>(distribution_[index]),
          std::get<2>(distribution_[index]));
      } else {
        spawn_vehicle_in_world_coordinate(
          name, pose, std::get<VehicleParameter>(parameter), std::get<1>(distribution_[index]),
          std::get<2>(distribution_[index]));
      }
    }
  }
}

auto TrafficSource::isPoseValid(
  const VehicleOrPedestrianParameter & parameter, const geometry_msgs::msg::Pose & pose)
  -> std::pair<bool, std::optional<CanonicalizedLaneletPose>>
{
  const auto bbox_corners = math::geometry::getPointsFromBbox(
    std::holds_alternative<PedestrianParameter>(parameter)
      ? std::get<PedestrianParameter>(parameter).bounding_box
      : std::get<VehicleParameter>(parameter).bounding_box);

  auto are_all_corners_inside_the_spawning_area = [&]() {
    /// @note transform bounding box corners to world coordinate system
    const auto corners = math::geometry::transformPoints(pose, bbox_corners);

    return std::all_of(corners.begin(), corners.end(), [&](const auto & corner) {
      /// @note 2D validation - does not account for height
      return std::hypot(corner.x - pose.position.x, corner.y - pose.position.y) <
             radius_distribution_.max();
    });
  };

  /// @note Step 1: check whether all corners are inside spawning area
  if (not are_all_corners_inside_the_spawning_area()) {
    return {false, std::nullopt};
  }

  /// @note Step 2: check whether can be outside lanelet
  if (configuration_.allow_spawn_outside_lane) {
    return {true, std::nullopt};
  }

  if (const auto lanelet_pose =
        hdmap_utils_->toLaneletPose(pose, std::holds_alternative<PedestrianParameter>(parameter));
      lanelet_pose) {
    /// @note reset orientation - to align the entity with lane
    auto corrected_pose = lanelet_pose.value();
    corrected_pose.rpy.z = 0.0;

    auto out_pose = std::make_optional<CanonicalizedLaneletPose>(corrected_pose, hdmap_utils_);

    /// @note Step 3: check whether the bounding box can be outside lanelet
    if (not configuration_.require_footprint_fitting) {
      return std::make_pair(true, out_pose);
    }

    /// @note Step 4: check whether the bounding box fits inside the lanelet
    if (std::holds_alternative<PedestrianParameter>(parameter)) {
      return std::make_pair(
        not configuration_.require_footprint_fitting or
          validate_considering_crosswalk(
            math::geometry::transformPoints(
              hdmap_utils_->toMapPose(corrected_pose).pose, bbox_corners),
            corrected_pose.lanelet_id),
        out_pose);
    } else {
      return std::make_pair(
        not configuration_.require_footprint_fitting or
          validate(
            math::geometry::transformPoints(
              hdmap_utils_->toMapPose(corrected_pose).pose, bbox_corners),
            corrected_pose.lanelet_id),
        out_pose);
    }
  } else {
    return {false, std::nullopt};
  }
}
}  // namespace traffic
}  // namespace traffic_simulator
