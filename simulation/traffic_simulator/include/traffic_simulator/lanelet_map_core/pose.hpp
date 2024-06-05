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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_

#include <lanelet2_matching/LaneletMatching.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map_core.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace pose
{
using Point = geometry_msgs::msg::Point;
using Vector3 = geometry_msgs::msg::Vector3;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using BoundingBox = traffic_simulator_msgs::msg::BoundingBox;
using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;
using EntityType = traffic_simulator_msgs::msg::EntityType;

auto toMapPose(const LaneletPose &, const bool fill_pitch = true) -> PoseStamped;

auto toLaneletPose(const Pose & pose, const bool include_crosswalk, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPose(const Pose & pose, const lanelet::Ids &, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPose(
  const Pose & pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<LaneletPose>;

auto toLaneletPose(const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPoses(
  const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance,
  const bool include_opposite_direction) -> std::vector<LaneletPose>;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto getAlternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>;

auto getAlongLaneletPose(const LaneletPose & from_pose, const double distance) -> LaneletPose;

auto getAlongLaneletPose(
  const LaneletPose & from_pose, const lanelet::Ids & route_lanelets, const double distance)
  -> LaneletPose;

// private
namespace
{
auto matchToLane(
  const Pose & pose, const BoundingBox & bbox, const bool include_crosswalk,
  const double matching_distance, const double reduction_ratio) -> std::optional<lanelet::Id>
{
  auto absoluteHull = [](
                        const lanelet::BasicPolygon2d & relative_hull,
                        const lanelet::matching::Pose2d & pose) -> lanelet::BasicPolygon2d {
    lanelet::BasicPolygon2d hull_points;
    hull_points.reserve(relative_hull.size());
    for (const auto & hull_ptr : relative_hull) {
      hull_points.push_back(pose * hull_ptr);
    }
    return hull_points;
  };

  std::optional<lanelet::Id> id;
  lanelet::matching::Object2d obj;
  obj.pose.translation() = lanelet::BasicPoint2d(pose.position.x, pose.position.y);
  obj.pose.linear() = Eigen::Rotation2D<double>(
                        quaternion_operation::convertQuaternionToEulerAngle(pose.orientation).z)
                        .matrix();
  obj.absoluteHull = absoluteHull(
    lanelet::matching::Hull2d{
      lanelet::BasicPoint2d{
        bbox.center.x + bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y + bbox.dimensions.y * 0.5 * reduction_ratio},
      lanelet::BasicPoint2d{
        bbox.center.x - bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y - bbox.dimensions.y * 0.5 * reduction_ratio}},
    obj.pose);
  auto matches =
    lanelet::matching::getDeterministicMatches(*LaneletMapCore::map(), obj, matching_distance);
  if (!include_crosswalk) {
    matches = lanelet::matching::removeNonRuleCompliantMatches(
      matches, LaneletMapCore::trafficRulesVehicle());
  }
  if (matches.empty()) {
    return std::nullopt;
  }
  std::vector<std::pair<lanelet::Id, double>> id_and_distance;
  for (const auto & match : matches) {
    if (
      const auto lanelet_pose = pose::toLaneletPose(pose, match.lanelet.id(), matching_distance)) {
      id_and_distance.emplace_back(lanelet_pose->lanelet_id, lanelet_pose->offset);
    }
  }
  if (id_and_distance.empty()) {
    return std::nullopt;
  }
  const auto min_id_and_distance = std::min_element(
    id_and_distance.begin(), id_and_distance.end(),
    [](auto const & lhs, auto const & rhs) { return lhs.second < rhs.second; });
  return min_id_and_distance->first;
}

auto getLeftLaneletIds(
  const lanelet::Id lanelet_id, const EntityType & type, const bool include_opposite_direction)
  -> lanelet::Ids
{
  auto getLaneletIds = [](const auto & lanelets) -> lanelet::Ids {
    lanelet::Ids ids;
    std::transform(
      lanelets.begin(), lanelets.end(), std::back_inserter(ids),
      [](const auto & lanelet) { return lanelet.id(); });
    return ids;
  };

  switch (type.type) {
    case EntityType::EGO:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->lefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->adjacentLefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    case EntityType::VEHICLE:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->lefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->adjacentLefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    case EntityType::PEDESTRIAN:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::pedestrianRoutingGraph()->lefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::pedestrianRoutingGraph()->adjacentLefts(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    default:
    case EntityType::MISC_OBJECT:
      return {};
  }
}

auto getRightLaneletIds(
  const lanelet::Id lanelet_id, const EntityType & entity_type,
  const bool include_opposite_direction) -> lanelet::Ids
{
  auto getLaneletIds = [](const auto & lanelets) -> lanelet::Ids {
    lanelet::Ids ids;
    std::transform(
      lanelets.begin(), lanelets.end(), std::back_inserter(ids),
      [](const auto & lanelet) { return lanelet.id(); });
    return ids;
  };

  switch (entity_type.type) {
    case EntityType::EGO:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->rights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->adjacentRights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    case EntityType::VEHICLE:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->rights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::vehicleRoutingGraph()->adjacentRights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    case EntityType::PEDESTRIAN:
      if (include_opposite_direction) {
        return getLaneletIds(LaneletMapCore::pedestrianRoutingGraph()->rights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(LaneletMapCore::pedestrianRoutingGraph()->adjacentRights(
          LaneletMapCore::map()->laneletLayer.get(lanelet_id)));
      }
    default:
    case EntityType::MISC_OBJECT:
      return {};
  }
}
}  // namespace
}  // namespace pose
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_
