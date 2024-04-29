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
auto getQuietNaNPose() -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                .x(std::numeric_limits<double>::quiet_NaN())
                .y(std::numeric_limits<double>::quiet_NaN())
                .z(std::numeric_limits<double>::quiet_NaN()))
    .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
}

auto getQuietNaNLaneletPose() -> traffic_simulator::LaneletPose
{
  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
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
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose
{
  return CanonicalizedLaneletPose(lanelet_pose, hdmap_utils_ptr);
}

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto toMapPose(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> geometry_msgs::msg::Pose
{
  constexpr bool fill_pitch{true};
  return hdmap_utils_ptr->toMapPose(lanelet_pose, fill_pitch).pose;
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
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
  const geometry_msgs::msg::Pose & map_pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const bool include_crosswalk, const double matching_distance,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, bbox, include_crosswalk, matching_distance)) {
    return canonicalize(pose.value(), hdmap_utils_ptr);
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  std::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;
  if (!unique_route_lanelets.empty()) {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, unique_route_lanelets, matching_distance);
  }
  if (!lanelet_pose) {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, bbox, include_crosswalk, matching_distance);
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

auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  try {
    return math::geometry::getRelativePose(from, to);
  } catch (...) {
    return std::nullopt;
  }
}

auto getRelativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return getRelativePose(from, static_cast<geometry_msgs::msg::Pose>(to));
}

auto getRelativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return getRelativePose(static_cast<geometry_msgs::msg::Pose>(from), to);
}

auto getBoundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<geometry_msgs::msg::Pose>
{
  if (const auto closest_points = math::geometry::getClosestPoses(from, from_bbox, to, to_bbox);
      closest_points) {
    const auto from_pose_bbox = getRelativePose(from, closest_points.value().first);
    const auto to_pose_bbox = getRelativePose(from, closest_points.value().second);
    if (from_pose_bbox && to_pose_bbox) {
      return math::geometry::subtractPoses(from_pose_bbox.value(), to_pose_bbox.value());
    }
  }
  return std::nullopt;
}

auto getRelativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  traffic_simulator::LaneletPose position = traffic_simulator::pose::getQuietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if it is not possible to
  // calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_distance = traffic_simulator::distance::getLongitudinalDistance(
      from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
      hdmap_utils_ptr)) {
    position.s = longitudinal_distance.value();
  }
  if (
    const auto lateral_distance = traffic_simulator::distance::getLateralDistance(
      from, to, allow_lane_change, hdmap_utils_ptr)) {
    position.offset = lateral_distance.value();
  }
  return position;
}

auto getBoundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  traffic_simulator::LaneletPose position = traffic_simulator::pose::getQuietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if it is not possible to
  // calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_bb_distance =
      traffic_simulator::distance::getBoundingBoxLaneLongitudinalDistance(
        from, from_bbox, to, to_bbox, include_adjacent_lanelet, include_opposite_direction,
        allow_lane_change, hdmap_utils_ptr)) {
    position.s = longitudinal_bb_distance.value();
  }
  if (
    const auto lateral_bb_distance = traffic_simulator::distance::getBoundingBoxLaneLateralDistance(
      from, from_bbox, to, to_bbox, allow_lane_change, hdmap_utils_ptr)) {
    position.offset = lateral_bb_distance.value();
  }
  return position;
}

auto estimateCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<traffic_simulator::CanonicalizedLaneletPose>
{
  std::optional<traffic_simulator::LaneletPose> lanelet_pose;
  if (!unique_route_lanelets.empty()) {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, unique_route_lanelets, matching_distance);
  } else {
    lanelet_pose =
      hdmap_utils_ptr->toLaneletPose(map_pose, bbox, include_crosswalk, matching_distance);
  }
  if (!lanelet_pose) {
    /**
     * @note Hard coded parameter. 2.0 is a matching threshold for lanelet.
     * true means considering crosswalk.
     * In this branch, the algorithm only consider entity pose.
     */
    lanelet_pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk, 2.0);
  }

  if (lanelet_pose) {
    const auto canonicalized = hdmap_utils_ptr->canonicalizeLaneletPose(lanelet_pose.value());
    if (
      const auto canonicalized_lanelet_pose =
        std::get<std::optional<traffic_simulator::LaneletPose>>(canonicalized)) {
      /// @note If canonicalize succeed, set canonicalized pose and set other values.
      return traffic_simulator::CanonicalizedLaneletPose(lanelet_pose.value(), hdmap_utils_ptr);
    } else {
      /// @note If canonicalize failed, set end of road lanelet pose.
      if (const auto end_of_road_lanelet_id = std::get<std::optional<lanelet::Id>>(canonicalized)) {
        if (lanelet_pose.value().s < 0) {
          return traffic_simulator::CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<traffic_simulator::LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(0.0)
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils_ptr);
        } else {
          return traffic_simulator::CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<traffic_simulator::LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(hdmap_utils_ptr->getLaneletLength(end_of_road_lanelet_id.value()))
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils_ptr);
        }
      } else {
        THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
      }
    }
  } else {
    return std::nullopt;
  }
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
    const auto start_edge = traffic_simulator::helper::constructCanonicalizedLaneletPose(
      lanelet_id, 0.0, 0.0, hdmap_utils_ptr);
    const auto end_edge = traffic_simulator::helper::constructCanonicalizedLaneletPose(
      lanelet_id, hdmap_utils_ptr->getLaneletLength(lanelet_id), 0.0, hdmap_utils_ptr);
    auto dist0 = distance::getLongitudinalDistance(
      start_edge, canonicalized_lanelet_pose, include_adjacent_lanelet, include_opposite_direction,
      allow_lane_change, hdmap_utils_ptr);
    auto dist1 = distance::getLongitudinalDistance(
      canonicalized_lanelet_pose, end_edge, include_adjacent_lanelet, include_opposite_direction,
      allow_lane_change, hdmap_utils_ptr);
    if (dist0 and dist0.value() < tolerance) {
      return true;
    }
    if (dist1 and dist1.value() < tolerance) {
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

auto getLaneletLength(
  const lanelet::Id lanelet_id, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> double
{
  return hdmap_utils_ptr->getLaneletLength(lanelet_id);
}
}  // namespace pose
}  // namespace traffic_simulator
