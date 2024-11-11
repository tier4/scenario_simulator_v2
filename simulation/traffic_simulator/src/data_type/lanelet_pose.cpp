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

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>

namespace traffic_simulator
{
inline namespace lanelet_pose
{
CanonicalizedLaneletPose::CanonicalizedLaneletPose(
  const LaneletPose & maybe_non_canonicalized_lanelet_pose)
: lanelet_pose_(pose::canonicalize(maybe_non_canonicalized_lanelet_pose)),
  lanelet_poses_(pose::alternativeLaneletPoses(maybe_non_canonicalized_lanelet_pose)),
  map_pose_(pose::toMapPose(lanelet_pose_))
{
  adjustOrientationAndOzPosition();
}

CanonicalizedLaneletPose::CanonicalizedLaneletPose(
  const LaneletPose & maybe_non_canonicalized_lanelet_pose, const lanelet::Ids & route_lanelets)
: lanelet_pose_(pose::canonicalize(maybe_non_canonicalized_lanelet_pose, route_lanelets)),
  lanelet_poses_(pose::alternativeLaneletPoses(maybe_non_canonicalized_lanelet_pose)),
  map_pose_(pose::toMapPose(lanelet_pose_))
{
  adjustOrientationAndOzPosition();
}

CanonicalizedLaneletPose::CanonicalizedLaneletPose(const CanonicalizedLaneletPose & other)
: lanelet_pose_(other.lanelet_pose_),
  lanelet_poses_(other.lanelet_poses_),
  map_pose_(other.map_pose_)
{
}

CanonicalizedLaneletPose::CanonicalizedLaneletPose(CanonicalizedLaneletPose && other) noexcept
: lanelet_pose_(std::move(other.lanelet_pose_)),
  lanelet_poses_(std::move(other.lanelet_poses_)),
  map_pose_(std::move(other.map_pose_))
{
}

auto CanonicalizedLaneletPose::operator=(const CanonicalizedLaneletPose & other)
  -> CanonicalizedLaneletPose &
{
  this->lanelet_pose_ = other.lanelet_pose_;
  this->lanelet_poses_ = other.lanelet_poses_;
  this->map_pose_ = other.map_pose_;
  return *this;
}

auto CanonicalizedLaneletPose::getAlternativeLaneletPoseBaseOnShortestRouteFrom(
  LaneletPose from, bool allow_lane_change) const -> std::optional<LaneletPose>
{
  if (lanelet_poses_.empty()) {
    return std::nullopt;
  }
  auto shortest_route = route::getRoute(from.lanelet_id, lanelet_poses_[0].lanelet_id);
  LaneletPose alternative_lanelet_pose = lanelet_poses_[0];
  for (const auto & laneletPose : lanelet_poses_) {
    const auto route = route::getRoute(from.lanelet_id, laneletPose.lanelet_id, allow_lane_change);
    if (shortest_route.size() > route.size()) {
      shortest_route = route;
      alternative_lanelet_pose = laneletPose;
    }
  }
  return alternative_lanelet_pose;
}

auto CanonicalizedLaneletPose::alignOrientationToLanelet() -> void
{
  using math::geometry::convertEulerAngleToQuaternion;
  using math::geometry::convertQuaternionToEulerAngle;
  const auto spline = route::toSpline({lanelet_pose_.lanelet_id});
  const auto lanelet_quaternion = spline.getPose(lanelet_pose_.s, true).orientation;
  const auto lanelet_rpy = convertQuaternionToEulerAngle(lanelet_quaternion);
  map_pose_.orientation =
    convertEulerAngleToQuaternion(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                    .x(lanelet_rpy.x)
                                    .y(lanelet_rpy.y)
                                    .z(lanelet_rpy.z));
  lanelet_pose_.rpy = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0);
}

auto CanonicalizedLaneletPose::adjustOrientationAndOzPosition() -> void
{
  using math::geometry::convertEulerAngleToQuaternion;
  using math::geometry::convertQuaternionToEulerAngle;
  using math::geometry::getRotation;
  const auto spline = route::toSpline({lanelet_pose_.lanelet_id});
  // adjust Oz position
  if (const auto s_value = spline.getSValue(map_pose_)) {
    map_pose_.position.z = spline.getPoint(s_value.value()).z;
  }
  // adjust pitch
  if (consider_pose_by_road_slope_) {
    const auto lanelet_quaternion = spline.getPose(lanelet_pose_.s, true).orientation;
    const auto lanelet_rpy = convertQuaternionToEulerAngle(lanelet_quaternion);
    const auto entity_rpy = convertQuaternionToEulerAngle(map_pose_.orientation);
    map_pose_.orientation =
      convertEulerAngleToQuaternion(geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                      .x(entity_rpy.x)
                                      .y(lanelet_rpy.y)
                                      .z(entity_rpy.z));
    lanelet_pose_.rpy =
      convertQuaternionToEulerAngle(getRotation(lanelet_quaternion, map_pose_.orientation));
  }
}

}  // namespace lanelet_pose

auto isSameLaneletId(const CanonicalizedLaneletPose & p0, const CanonicalizedLaneletPose & p1)
  -> bool
{
  return static_cast<LaneletPose>(p0).lanelet_id == static_cast<LaneletPose>(p1).lanelet_id;
}

auto isSameLaneletId(const CanonicalizedLaneletPose & p, const lanelet::Id lanelet_id) -> bool
{
  return static_cast<LaneletPose>(p).lanelet_id == lanelet_id;
}
}  // namespace traffic_simulator
