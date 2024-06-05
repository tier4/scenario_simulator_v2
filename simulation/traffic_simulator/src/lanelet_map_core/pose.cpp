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

#include <quaternion_operation/quaternion_operation.h>

#include <geometry/linear_algebra.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map.hpp>
#include <traffic_simulator/lanelet_map_core/pose.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace pose
{
auto toMapPose(const LaneletPose & lanelet_pose, const bool fill_pitch) -> PoseStamped
{
  if (
    const auto pose =
      std::get<std::optional<LaneletPose>>(pose::canonicalizeLaneletPose(lanelet_pose))) {
    PoseStamped ret;
    ret.header.frame_id = "map";
    const auto spline = lanelet_map::getCenterPointsSpline(pose->lanelet_id);
    ret.pose = spline->getPose(pose->s);
    const auto normal_vec = spline->getNormalVector(pose->s);
    const auto diff = math::geometry::normalize(normal_vec) * pose->offset;
    ret.pose.position = ret.pose.position + diff;
    const auto tangent_vec = spline->getTangentVector(pose->s);
    Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = fill_pitch ? std::atan2(-tangent_vec.z, std::hypot(tangent_vec.x, tangent_vec.y)) : 0.0;
    rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
    ret.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy) *
                           quaternion_operation::convertEulerAngleToQuaternion(pose->rpy);
    return ret;
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z, ") is invalid, please check lanelet length and connection.");
  }
}

auto toLaneletPose(const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance)
  -> std::optional<LaneletPose>
{
  const auto spline = lanelet_map::getCenterPointsSpline(lanelet_id);
  const auto s = spline->getSValue(pose, matching_distance);
  if (!s) {
    return std::nullopt;
  }
  auto pose_on_centerline = spline->getPose(s.value());
  auto rpy = quaternion_operation::convertQuaternionToEulerAngle(
    quaternion_operation::getRotation(pose_on_centerline.orientation, pose.orientation));
  double offset = std::sqrt(spline->getSquaredDistanceIn2D(pose.position, s.value()));
  /**
   * @note Hard coded parameter
   */
  double yaw_threshold = 0.25;
  if (M_PI * yaw_threshold < std::fabs(rpy.z) && std::fabs(rpy.z) < M_PI * (1 - yaw_threshold)) {
    return std::nullopt;
  }
  double inner_prod = math::geometry::innerProduct(
    spline->getNormalVector(s.value()), spline->getSquaredDistanceVector(pose.position, s.value()));
  if (inner_prod < 0) {
    offset = offset * -1;
  }
  LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s.value();
  lanelet_pose.offset = offset;
  lanelet_pose.rpy = rpy;
  return lanelet_pose;
}

auto toLaneletPose(const Pose & pose, const bool include_crosswalk, const double matching_distance)
  -> std::optional<LaneletPose>
{
  constexpr double distance_threshold{0.1};
  constexpr std::size_t search_count{5};
  const auto lanelet_ids = lanelet_map::getNearbyLaneletIds(
    pose.position, distance_threshold, include_crosswalk, search_count);
  if (lanelet_ids.empty()) {
    return std::nullopt;
  }
  for (const auto & id : lanelet_ids) {
    const auto lanelet_pose = toLaneletPose(pose, id, matching_distance);
    if (lanelet_pose) {
      return lanelet_pose;
    }
  }
  return std::nullopt;
}

auto toLaneletPose(
  const Pose & pose, const lanelet::Ids & lanelet_ids, const double matching_distance)
  -> std::optional<LaneletPose>
{
  for (const auto id : lanelet_ids) {
    if (const auto lanelet_pose = toLaneletPose(pose, id, matching_distance); lanelet_pose) {
      return lanelet_pose.value();
    }
  }
  return std::nullopt;
}

auto toLaneletPose(
  const Pose & pose, const BoundingBox & bbox, const bool include_crosswalk,
  const double matching_distance) -> std::optional<LaneletPose>
{
  constexpr double reduction_ratio{0.8};
  const auto lanelet_id =
    matchToLane(pose, bbox, include_crosswalk, matching_distance, reduction_ratio);
  if (!lanelet_id) {
    return toLaneletPose(pose, include_crosswalk, matching_distance);
  }
  const auto pose_in_target_lanelet = toLaneletPose(pose, lanelet_id.value(), matching_distance);
  if (pose_in_target_lanelet) {
    return pose_in_target_lanelet;
  }
  const auto previous = lanelet_map::getPreviousLaneletIds(lanelet_id.value());
  for (const auto id : previous) {
    const auto pose_in_previous = toLaneletPose(pose, id, matching_distance);
    if (pose_in_previous) {
      return pose_in_previous;
    }
  }
  const auto next = lanelet_map::getNextLaneletIds(lanelet_id.value());
  for (const auto id : previous) {
    const auto pose_in_next = toLaneletPose(pose, id, matching_distance);
    if (pose_in_next) {
      return pose_in_next;
    }
  }
  return toLaneletPose(pose, include_crosswalk, matching_distance);
}

auto toLaneletPoses(
  const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance,
  const bool include_opposite_direction) -> std::vector<LaneletPose>
{
  std::vector<LaneletPose> ret;
  EntityType type;
  type.type = EntityType::VEHICLE;
  std::vector lanelet_ids = {lanelet_id};
  lanelet_ids += getLeftLaneletIds(lanelet_id, type, include_opposite_direction);
  lanelet_ids += getRightLaneletIds(lanelet_id, type, include_opposite_direction);
  lanelet_ids += lanelet_map::getPreviousLaneletIds(lanelet_ids);
  lanelet_ids += lanelet_map::getNextLaneletIds(lanelet_ids);
  for (const auto & id : sortAndUnique(lanelet_ids)) {
    if (const auto & lanelet_pose = toLaneletPose(pose, id, matching_distance)) {
      ret.emplace_back(lanelet_pose.value());
    }
  }
  return ret;
}

auto getAlternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>
{
  /// @note Define lambda functions for canonicalizing previous/next lanelet.
  const auto canonicalize_to_previous_lanelet =
    [](const auto & lanelet_pose) -> std::vector<LaneletPose> {
    if (const auto ids = lanelet_map::getPreviousLaneletIds(lanelet_pose.lanelet_id);
        !ids.empty()) {
      std::vector<LaneletPose> canonicalized_all;
      for (const auto id : ids) {
        const auto lanelet_pose_tmp = helper::constructLaneletPose(
          id, lanelet_pose.s + lanelet_map::getLaneletLength(id), lanelet_pose.offset);
        if (const auto canonicalized_lanelet_poses = getAlternativeLaneletPoses(lanelet_pose_tmp);
            canonicalized_lanelet_poses.empty()) {
          canonicalized_all.emplace_back(lanelet_pose_tmp);
        } else {
          std::copy(
            canonicalized_lanelet_poses.begin(), canonicalized_lanelet_poses.end(),
            std::back_inserter(canonicalized_all));
        }
      }
      return canonicalized_all;
    } else {
      return {};
    }
  };
  const auto canonicalize_to_next_lanelet =
    [](const auto & lanelet_pose) -> std::vector<LaneletPose> {
    if (const auto ids = lanelet_map::getNextLaneletIds(lanelet_pose.lanelet_id); !ids.empty()) {
      std::vector<LaneletPose> canonicalized_all;
      for (const auto id : ids) {
        const auto lanelet_pose_tmp = helper::constructLaneletPose(
          id, lanelet_pose.s - lanelet_map::getLaneletLength(lanelet_pose.lanelet_id),
          lanelet_pose.offset);
        if (const auto canonicalized_lanelet_poses = getAlternativeLaneletPoses(lanelet_pose_tmp);
            canonicalized_lanelet_poses.empty()) {
          canonicalized_all.emplace_back(lanelet_pose_tmp);
        } else {
          std::copy(
            canonicalized_lanelet_poses.begin(), canonicalized_lanelet_poses.end(),
            std::back_inserter(canonicalized_all));
        }
      }
      return canonicalized_all;
    } else {
      return {};
    }
  };

  /// @note If s value under 0, it means this pose is on the previous lanelet.
  if (lanelet_pose.s < 0) {
    return canonicalize_to_previous_lanelet(lanelet_pose);
  }
  /// @note If s value overs it's lanelet length, it means this pose is on the next lanelet.
  else if (lanelet_pose.s > (lanelet_map::getLaneletLength(lanelet_pose.lanelet_id))) {
    return canonicalize_to_next_lanelet(lanelet_pose);
  }
  /// @note If s value is in range [0,length_of_the_lanelet], return lanelet_pose.
  else {
    return {lanelet_pose};
  }
}

auto getAlongLaneletPose(const LaneletPose & from_pose, const double distance) -> LaneletPose
{
  LaneletPose along_pose = from_pose;
  along_pose.s = along_pose.s + distance;
  if (along_pose.s >= 0) {
    while (along_pose.s >= lanelet_map::getLaneletLength(along_pose.lanelet_id)) {
      auto next_ids = lanelet_map::getNextLaneletIds(along_pose.lanelet_id, "straight");
      if (next_ids.empty()) {
        next_ids = lanelet_map::getNextLaneletIds(along_pose.lanelet_id);
        if (next_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + distance, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s - lanelet_map::getLaneletLength(along_pose.lanelet_id);
      along_pose.lanelet_id = next_ids[0];
    }
  } else {
    while (along_pose.s < 0) {
      auto previous_ids = lanelet_map::getPreviousLaneletIds(along_pose.lanelet_id, "straight");
      if (previous_ids.empty()) {
        previous_ids = lanelet_map::getPreviousLaneletIds(along_pose.lanelet_id);
        if (previous_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + distance, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s + lanelet_map::getLaneletLength(previous_ids[0]);
      along_pose.lanelet_id = previous_ids[0];
    }
  }
  return along_pose;
}

auto getAlongLaneletPose(
  const LaneletPose & from_pose, const lanelet::Ids & route_lanelets, const double distance)
  -> LaneletPose
{
  auto lanelet_pose = from_pose;
  lanelet_pose.s = lanelet_pose.s + distance;
  const auto canonicalized = canonicalizeLaneletPose(lanelet_pose, route_lanelets);
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
        lanelet_pose.s <= 0 ? 0 : lanelet_map::getLaneletLength(end_of_road_lanelet_id.value());
      return end_of_road_lanelet_pose;
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
    }
  }
}

// If route is not specified, the lanelet_id with the lowest array index is used as a candidate for
// canonicalize destination.
auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>
{
  auto canonicalized = lanelet_pose;
  while (canonicalized.s < 0) {
    if (const auto ids = lanelet_map::getPreviousLaneletIds(canonicalized.lanelet_id);
        ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s += lanelet_map::getLaneletLength(ids[0]);
      canonicalized.lanelet_id = ids[0];
    }
  }
  while (canonicalized.s > lanelet_map::getLaneletLength(canonicalized.lanelet_id)) {
    if (const auto ids = lanelet_map::getNextLaneletIds(canonicalized.lanelet_id); ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s -= lanelet_map::getLaneletLength(canonicalized.lanelet_id);
      canonicalized.lanelet_id = ids[0];
    }
  }
  return {canonicalized, std::nullopt};
}

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>
{
  auto canonicalized = lanelet_pose;
  while (canonicalized.s < 0) {
    // When canonicalizing to backward lanelet_id, do not consider route
    if (const auto ids = lanelet_map::getPreviousLaneletIds(canonicalized.lanelet_id);
        ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s += lanelet_map::getLaneletLength(ids[0]);
      canonicalized.lanelet_id = ids[0];
    }
  }
  while (canonicalized.s > lanelet_map::getLaneletLength(canonicalized.lanelet_id)) {
    bool next_lanelet_found = false;
    // When canonicalizing to forward lanelet_id, consider route
    for (const auto id : lanelet_map::getNextLaneletIds(canonicalized.lanelet_id)) {
      if (std::any_of(route_lanelets.begin(), route_lanelets.end(), [id](auto id_on_route) {
            return id == id_on_route;
          })) {
        canonicalized.s -= lanelet_map::getLaneletLength(canonicalized.lanelet_id);
        canonicalized.lanelet_id = id;
        next_lanelet_found = true;
      }
    }
    if (!next_lanelet_found) {
      return {std::nullopt, canonicalized.lanelet_id};
    }
  }
  return {canonicalized, std::nullopt};
}
}  // namespace pose
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
