// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <geometry/spline/catmull_rom_spline.hpp>
#include <traffic_simulator/utils/lanelet_core/other.hpp>
#include <traffic_simulator/utils/lanelet_core/pose.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>

namespace traffic_simulator
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route_lanelets) -> bool
{
  return std::find_if(route_lanelets.begin(), route_lanelets.end(), [lanelet_id](const auto id) {
           return lanelet_id == id;
         }) != route_lanelets.end();
}

auto toSpline(const lanelet::Ids & route_lanelets) -> Spline
{
  return Spline(lanelet_core::other::getCenterPoints(route_lanelets));
}

auto isAnyConflictingEntity(
  const lanelet::Ids & following_lanelets,
  const std::vector<CanonicalizedLaneletPose> & other_poses) -> bool
{
  auto conflicting_crosswalks = lanelet_core::route::getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = lanelet_core::route::getConflictingLaneIds(following_lanelets);
  for (const auto & pose : other_poses) {
    if (
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        static_cast<LaneletPose>(pose).lanelet_id) >= 1) {
      return true;
    }
    if (
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(),
        static_cast<LaneletPose>(pose).lanelet_id) >= 1) {
      return true;
    }
  }
  return false;
}

auto isNeedToRightOfWay(
  const lanelet::Ids & following_lanelets,
  const std::vector<CanonicalizedLaneletPose> & other_poses) -> bool
{
  auto isTheSameRightOfWay =
    [&](const std::int64_t & lanelet_id, const std::int64_t & following_lanelet) {
      const auto right_of_way_lanelet_ids =
        lanelet_core::route::getRightOfWayLaneletIds(lanelet_id);
      const auto the_same_right_of_way_it = std::find(
        right_of_way_lanelet_ids.begin(), right_of_way_lanelet_ids.end(), following_lanelet);
      return the_same_right_of_way_it != std::end(right_of_way_lanelet_ids);
    };

  const auto lanelet_ids_list = lanelet_core::route::getRightOfWayLaneletIds(following_lanelets);
  for (const auto & pose : other_poses) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const lanelet::Id lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          isSameLaneletId(pose, lanelet_id) &&
          not isTheSameRightOfWay(lanelet_id, following_lanelet)) {
          return true;
        }
      }
    }
  }
  return false;
}

auto moveAlongLaneletPose(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Ids & route_lanelets,
  const double distance) -> LaneletPose
{
  auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  lanelet_pose.s = lanelet_pose.s + distance;
  const auto canonicalized =
    lanelet_core::pose::canonicalizeLaneletPose(lanelet_pose, route_lanelets);
  if (const auto canonicalized_lanelet_pose = std::get<std::optional<LaneletPose>>(canonicalized)) {
    // If canonicalize succeed, just return canonicalized pose
    return canonicalized_lanelet_pose.value();
  } else {
    // If canonicalize failed, return lanelet pose as end of road
    if (const auto end_of_road_lanelet_id = std::get<std::optional<lanelet::Id>>(canonicalized)) {
      LaneletPose end_of_road_lanelet_pose;
      end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.value();
      end_of_road_lanelet_pose.offset = lanelet_pose.offset;
      end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
      /// @note here was condition: .s < 0, now try to use .s <= 0
      end_of_road_lanelet_pose.s =
        lanelet_pose.s <= 0 ? 0
                            : lanelet_core::other::getLaneletLength(end_of_road_lanelet_id.value());
      return end_of_road_lanelet_pose;
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
    }
  }
}

auto laneChangeAlongLaneletPose(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const lane_change::Parameter & parameter) -> LaneletPose
{
  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  switch (parameter.constraint.type) {
    case lane_change::Constraint::Type::NONE:
      return lanelet_core::lane_change::getAlongLaneletPose(
        lanelet_pose, lane_change::Parameter::default_lanechange_distance);
    case lane_change::Constraint::Type::LATERAL_VELOCITY:
      return lanelet_core::lane_change::getAlongLaneletPose(
        lanelet_pose, lane_change::Parameter::default_lanechange_distance);
    case lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      return lanelet_core::lane_change::getAlongLaneletPose(
        lanelet_pose, parameter.constraint.value);
    case lane_change::Constraint::Type::TIME:
      return lanelet_core::lane_change::getAlongLaneletPose(
        lanelet_pose, parameter.constraint.value);
    default:
      throw std::invalid_argument("Unknown lane change constraint type");
  }
  return pose::quietNaNLaneletPose();
}

auto moveBackPoints(const CanonicalizedLaneletPose & canonicalized_lanelet_pose)
  -> std::vector<Point>
{
  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  const auto ids = route::previousLanelets(lanelet_pose.lanelet_id);
  // DIFFERENT SPLINE - recalculation needed
  Spline spline(lanelet_core::other::getCenterPoints(ids));
  double s_in_spline = 0;
  for (const auto id : ids) {
    if (id == lanelet_pose.lanelet_id) {
      s_in_spline = s_in_spline + lanelet_pose.s;
      break;
    } else {
      s_in_spline = lanelet_core::other::getLaneletLength(id) + s_in_spline;
    }
  }
  return spline.getTrajectory(s_in_spline, s_in_spline - 5, 1.0, lanelet_pose.offset);
}

auto laneChangeTrajectory(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const lane_change::Parameter & parameter) -> std::optional<std::pair<Curve, double>>
{
  if (lanelet_core::lane_change::canChangeLane(
        canonicalized_lanelet_pose.getLaneletId(), parameter.target.lanelet_id)) {
    const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
    switch (parameter.constraint.type) {
      /**
        @note Hard coded parameter,
        10.0 is a maximum_curvature_threshold (If the curvature of the trajectory is over 10.0, the trajectory was not selected.)
        20.0 is a target_trajectory_length (The one with the closest length to 20 m is selected from the candidate trajectories.)
        1.0 is a forward_distance_threshold (If the goal x position in the cartesian coordinate was under 1.0, the goal was rejected.)
      */
      case lane_change::Constraint::Type::NONE:
        return lanelet_core::lane_change::getLaneChangeTrajectory(
          pose::toMapPose(lanelet_pose), parameter, 10.0, 20.0, 1.0);
      case lane_change::Constraint::Type::LATERAL_VELOCITY:
        return lanelet_core::lane_change::getLaneChangeTrajectory(lanelet_pose, parameter);
      case lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
        return lanelet_core::lane_change::getLaneChangeTrajectory(lanelet_pose, parameter);
      case lane_change::Constraint::Type::TIME:
        return lanelet_core::lane_change::getLaneChangeTrajectory(lanelet_pose, parameter);
      default:
        throw std::invalid_argument("Unknown lane change constraint type");
    }
  }
  return std::nullopt;
}

auto laneChangePoints(
  const Curve & curve, const double current_s, const double target_s, const double horizon,
  const lane_change::Parameter & parameter) -> std::vector<Point>
{
  if (const double rest_s = current_s + horizon - curve.getLength(); rest_s < 0) {
    return curve.getTrajectory(current_s, current_s + horizon, 1.0, true);
  } else {
    const auto following_lanelets = route::followingLanelets(parameter.target.lanelet_id, 0);
    const auto center_points = lanelet_core::other::getCenterPoints(following_lanelets);
    // DIFFERENT SPLINE - recalculation needed
    const Spline spline(center_points);
    /// @note not the same as orginal one - here were duplicates and curve_waypoints
    return spline.getTrajectory(target_s, target_s + rest_s, 1.0);
  }
}
}  // namespace route
}  // namespace traffic_simulator