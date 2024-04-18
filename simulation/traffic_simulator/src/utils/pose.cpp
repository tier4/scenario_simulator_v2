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

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> const geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto toLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (const auto pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk)) {
    return canonicalize(pose.value(), hdmap_utils_ptr);
  }
  return std::nullopt;
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

auto makeNativeRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  const auto longitudinal_distance = traffic_simulator::distance::getLongitudinalDistance(
    from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
    hdmap_utils_ptr);
  const auto lateral_distance =
    traffic_simulator::distance::getLateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);

  traffic_simulator::LaneletPose position = traffic_simulator::pose::getQuietNaNLaneletPose();
  if (longitudinal_distance && lateral_distance) {
    position.s = longitudinal_distance.value();
    position.offset = lateral_distance.value();
  }
  return position;
}

auto makeNativeBoundingBoxRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  const auto longitudinal_bb_distance =
    traffic_simulator::distance::getBoundingBoxLaneLongitudinalDistance(
      from, from_bbox, to, to_bbox, include_adjacent_lanelet, include_opposite_direction,
      allow_lane_change, hdmap_utils_ptr);
  const auto lateral_bb_distance = traffic_simulator::distance::getBoundingBoxLaneLateralDistance(
    from, from_bbox, to, to_bbox, allow_lane_change, hdmap_utils_ptr);

  traffic_simulator::LaneletPose position = traffic_simulator::pose::getQuietNaNLaneletPose();
  if (longitudinal_bb_distance && include_opposite_direction) {
    position.s = longitudinal_bb_distance.value();
    position.offset = lateral_bb_distance.value();
  }
  return position;
}

}  // namespace pose
}  // namespace traffic_simulator
