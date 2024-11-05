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

#include <geometry/bounding_box.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace pose
{
auto quietNaNPose() -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                .x(std::numeric_limits<double>::quiet_NaN())
                .y(std::numeric_limits<double>::quiet_NaN())
                .z(std::numeric_limits<double>::quiet_NaN()))
    .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
}

auto quietNaNLaneletPose() -> LaneletPose
{
  return traffic_simulator_msgs::build<LaneletPose>()
    .lanelet_id(std::numeric_limits<std::int64_t>::max())
    .s(std::numeric_limits<double>::quiet_NaN())
    .offset(std::numeric_limits<double>::quiet_NaN())
    .rpy(geometry_msgs::build<geometry_msgs::msg::Vector3>()
           .x(std::numeric_limits<double>::quiet_NaN())
           .y(std::numeric_limits<double>::quiet_NaN())
           .z(std::numeric_limits<double>::quiet_NaN()));
}

auto canonicalize(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (lanelet_pose == LaneletPose()) {
    return std::nullopt;
  } else {
    return CanonicalizedLaneletPose(lanelet_pose, hdmap_utils_ptr);
  }
}

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto toMapPose(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> geometry_msgs::msg::Pose
{
  return hdmap_utils_ptr
    ->toMapPose(lanelet_pose, CanonicalizedLaneletPose::getConsiderPoseByRoadSlope())
    .pose;
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, const bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  /// @todo here matching_distance should be passed
  if (const auto pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk)) {
    return canonicalize(pose.value(), hdmap_utils_ptr);
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto pose = hdmap_utils_ptr->toLaneletPose(
      map_pose, bounding_box, include_crosswalk, matching_distance)) {
    return canonicalize(pose.value(), hdmap_utils_ptr);
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  std::optional<LaneletPose> lanelet_pose;
  if (!unique_route_lanelets.empty()) {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, unique_route_lanelets, matching_distance);
  }
  if (!lanelet_pose) {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, bounding_box, include_crosswalk, matching_distance);
  }
  if (lanelet_pose) {
    return canonicalize(lanelet_pose.value(), hdmap_utils_ptr);
  } else {
    return std::nullopt;
  }
}

auto transformRelativePoseToGlobal(
  const geometry_msgs::msg::Pose & global_pose, const geometry_msgs::msg::Pose & relative_pose)
  -> geometry_msgs::msg::Pose
{
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(global_pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

auto relativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  try {
    return math::geometry::getRelativePose(from, to);
  } catch (...) {
    return std::nullopt;
  }
}

auto relativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return relativePose(from, static_cast<geometry_msgs::msg::Pose>(to));
}

auto relativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return relativePose(static_cast<geometry_msgs::msg::Pose>(from), to);
}

auto boundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box)
  -> std::optional<geometry_msgs::msg::Pose>
{
  if (const auto closest_points =
        math::geometry::getClosestPoses(from, from_bounding_box, to, to_bounding_box);
      closest_points) {
    const auto from_pose_bounding_box = relativePose(from, closest_points.value().first);
    const auto to_pose_bounding_box = relativePose(from, closest_points.value().second);
    if (from_pose_bounding_box && to_pose_bounding_box) {
      return math::geometry::subtractPoses(
        from_pose_bounding_box.value(), to_pose_bounding_box.value());
    }
  }
  return std::nullopt;
}

auto relativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if
  // it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_distance = longitudinalDistance(
      from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
      hdmap_utils_ptr)) {
    position.s = longitudinal_distance.value();
  }
  if (const auto lateral_distance = lateralDistance(from, to, allow_lane_change, hdmap_utils_ptr)) {
    position.offset = lateral_distance.value();
  }
  return position;
}

auto boundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, const bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if
  // it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_bounding_box_distance = boundingBoxLaneLongitudinalDistance(
      from, from_bounding_box, to, to_bounding_box, include_adjacent_lanelet,
      include_opposite_direction, allow_lane_change, hdmap_utils_ptr)) {
    position.s = longitudinal_bounding_box_distance.value();
  }
  if (
    const auto lateral_bounding_box_distance = boundingBoxLaneLateralDistance(
      from, from_bounding_box, to, to_bounding_box, allow_lane_change, hdmap_utils_ptr)) {
    position.offset = lateral_bounding_box_distance.value();
  }
  return position;
}

auto isInLanelet(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id lanelet_id,
  const double tolerance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> bool
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{false};
  constexpr bool allow_lane_change{false};

  if (isSameLaneletId(canonicalized_lanelet_pose, lanelet_id)) {
    return true;
  } else {
    const auto start_lanelet_pose =
      helper::constructCanonicalizedLaneletPose(lanelet_id, 0.0, 0.0, hdmap_utils_ptr);
    if (const auto distance_to_start_lanelet_pose = longitudinalDistance(
          start_lanelet_pose, canonicalized_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, allow_lane_change, hdmap_utils_ptr);
        distance_to_start_lanelet_pose and
        std::abs(distance_to_start_lanelet_pose.value()) <= tolerance) {
      return true;
    }

    const auto end_lanelet_pose = helper::constructCanonicalizedLaneletPose(
      lanelet_id, hdmap_utils_ptr->getLaneletLength(lanelet_id), 0.0, hdmap_utils_ptr);
    if (const auto distance_to_end_lanelet_pose = longitudinalDistance(
          canonicalized_lanelet_pose, end_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, allow_lane_change, hdmap_utils_ptr);
        distance_to_end_lanelet_pose and
        std::abs(distance_to_end_lanelet_pose.value()) <= tolerance) {
      return true;
    }
  }
  return false;
}

auto isAtEndOfLanelets(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> bool
{
  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  return hdmap_utils_ptr->getFollowingLanelets(lanelet_pose.lanelet_id).size() == 1 &&
         hdmap_utils_ptr->getLaneletLength(lanelet_pose.lanelet_id) <= lanelet_pose.s;
}

auto laneletLength(
  const lanelet::Id lanelet_id, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> double
{
  return hdmap_utils_ptr->getLaneletLength(lanelet_id);
}

namespace pedestrian
{
/*
  This function has been moved from pedestrian_action_node and modified,
  in case of inconsistency please compare in original:
  https://github.com/tier4/scenario_simulator_v2/blob/090a8d08bcb065d293a530cf641a953edf311f9f/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L67-L128
*/
auto transformToCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto canonicalized_lanelet_pose = toCanonicalizedLaneletPose(
      map_pose, bounding_box, unique_route_lanelets, include_crosswalk, matching_distance,
      hdmap_utils_ptr)) {
    return canonicalized_lanelet_pose;
  }
  /**
   * @note Hard coded parameter. 2.0 is a matching threshold for lanelet.
   * In this branch, the algorithm only consider entity pose.
   */
  if (const auto lanelet_pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk, 2.0)) {
    const auto canonicalized_tuple = hdmap_utils_ptr->canonicalizeLaneletPose(lanelet_pose.value());
    if (
      const auto canonicalized_lanelet_pose =
        std::get<std::optional<LaneletPose>>(canonicalized_tuple)) {
      return canonicalize(lanelet_pose.value(), hdmap_utils_ptr);
    } else {
      /// @note If canonicalize failed, set end of road lanelet pose.
      if (
        const auto end_of_road_lanelet_id =
          std::get<std::optional<lanelet::Id>>(canonicalized_tuple)) {
        if (lanelet_pose.value().s < 0) {
          return CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(0.0)
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils_ptr);
        } else {
          return CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(hdmap_utils_ptr->getLaneletLength(end_of_road_lanelet_id.value()))
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils_ptr);
        }
      } else {
        THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id for LaneletPose estimation.");
      }
    }
  } else {
    return std::nullopt;
  }
}
}  // namespace pedestrian
}  // namespace pose
}  // namespace traffic_simulator
