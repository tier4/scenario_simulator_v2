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

auto quietNaNLaneletPose() -> traffic_simulator::LaneletPose
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

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> const geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto toLaneletPose(
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
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  traffic_simulator::LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if it is not possible to
  // calculate one of them - it happens that one is sufficient
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
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  traffic_simulator::LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if it is not possible to
  // calculate one of them - it happens that one is sufficient
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
}  // namespace pose
}  // namespace traffic_simulator
