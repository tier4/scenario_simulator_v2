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

#include <geometry/transform.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/lanelet/lane_change.hpp>
#include <traffic_simulator/utils/lanelet/lanelet_map.hpp>
#include <traffic_simulator/utils/lanelet/other.hpp>
#include <traffic_simulator/utils/lanelet/pose.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace lane_change
{
auto canChangeLane(const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id) -> bool
{
  const auto from_lanelet = LaneletMap::map()->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = LaneletMap::map()->laneletLayer.get(to_lanelet_id);
  return LaneletMap::trafficRulesVehicle()->canChangeLane(from_lanelet, to_lanelet);
}

auto getAlongLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose, const double along)
  -> traffic_simulator_msgs::msg::LaneletPose
{
  traffic_simulator_msgs::msg::LaneletPose along_pose = from_pose;
  along_pose.s = along_pose.s + along;
  if (along_pose.s >= 0) {
    while (along_pose.s >= other::getLaneletLength(along_pose.lanelet_id)) {
      auto next_ids = other::getNextLaneletIds(along_pose.lanelet_id, "straight");
      if (next_ids.empty()) {
        next_ids = other::getNextLaneletIds(along_pose.lanelet_id);
        if (next_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + along, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s - other::getLaneletLength(along_pose.lanelet_id);
      along_pose.lanelet_id = next_ids[0];
    }
  } else {
    while (along_pose.s < 0) {
      auto previous_ids = other::getPreviousLaneletIds(along_pose.lanelet_id, "straight");
      if (previous_ids.empty()) {
        previous_ids = other::getPreviousLaneletIds(along_pose.lanelet_id);
        if (previous_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + along, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s + other::getLaneletLength(previous_ids[0]);
      along_pose.lanelet_id = previous_ids[0];
    }
  }
  return along_pose;
}

auto getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction,
  const std::uint8_t shift) -> std::optional<lanelet::Id>
{
  if (shift == 0) {
    return getLaneChangeableLaneletId(
      lanelet_id, traffic_simulator::lane_change::Direction::STRAIGHT);
  } else {
    auto reference_id = lanelet_id;
    for (uint8_t i = 0; i < shift; i++) {
      auto id = getLaneChangeableLaneletId(reference_id, direction);
      if (!id) {
        return std::nullopt;
      } else {
        reference_id = id.value();
      }
      if (i == (shift - 1)) {
        return reference_id;
      }
    }
  }
  return std::nullopt;
}

auto getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction)
  -> std::optional<lanelet::Id>
{
  const auto lanelet = LaneletMap::map()->laneletLayer.get(lanelet_id);
  std::optional<lanelet::Id> target = std::nullopt;
  switch (direction) {
    case traffic_simulator::lane_change::Direction::STRAIGHT:
      target = lanelet.id();
      break;
    case traffic_simulator::lane_change::Direction::LEFT:
      if (LaneletMap::vehicleRoutingGraph()->left(lanelet)) {
        target = LaneletMap::vehicleRoutingGraph()->left(lanelet)->id();
      }
      break;
    case traffic_simulator::lane_change::Direction::RIGHT:
      if (LaneletMap::vehicleRoutingGraph()->right(lanelet)) {
        target = LaneletMap::vehicleRoutingGraph()->right(lanelet)->id();
      }
      break;
  }
  return target;
}

auto getLaneChangeTrajectory(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double longitudinal_distance =
    traffic_simulator::lane_change::Parameter::default_lanechange_distance;
  switch (lane_change_parameter.constraint.type) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      longitudinal_distance = lane_change_parameter.constraint.value;
      break;
    case traffic_simulator::lane_change::Constraint::Type::TIME:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
  }
  const auto along_pose = getAlongLaneletPose(from_pose, longitudinal_distance);
  // clang-format off
  const auto left_point =
    pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset + 5.0)).pose.position;
  const auto right_point =
    pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset - 5.0)).pose.position;
  // clang-format on
  const auto collision_point = other::getCenterPointsSpline(lane_change_parameter.target.lanelet_id)
                                 ->getCollisionPointIn2D(left_point, right_point);
  if (!collision_point) {
    return std::nullopt;
  }
  const auto to_pose = traffic_simulator::helper::constructLaneletPose(
    lane_change_parameter.target.lanelet_id, collision_point.value(),
    lane_change_parameter.target.offset);
  const auto goal_pose_in_map = pose::toMapPose(to_pose).pose;
  const auto from_pose_in_map = pose::toMapPose(from_pose).pose;
  double start_to_goal_distance = std::sqrt(
    std::pow(from_pose_in_map.position.x - goal_pose_in_map.position.x, 2) +
    std::pow(from_pose_in_map.position.y - goal_pose_in_map.position.y, 2) +
    std::pow(from_pose_in_map.position.z - goal_pose_in_map.position.z, 2));

  auto traj = getLaneChangeTrajectory(
    pose::toMapPose(from_pose).pose, to_pose, lane_change_parameter.trajectory_shape,
    start_to_goal_distance * 0.5);
  return std::make_pair(traj, collision_point.value());
}

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double to_length = other::getLaneletLength(lane_change_parameter.target.lanelet_id);
  std::vector<double> evaluation, target_s;
  std::vector<math::geometry::HermiteCurve> curves;

  for (double to_s = 0; to_s < to_length; to_s = to_s + 1.0) {
    auto goal_pose = pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      lane_change_parameter.target.lanelet_id, to_s));
    if (
      math::geometry::getRelativePose(from_pose, goal_pose.pose).position.x <=
      forward_distance_threshold) {
      continue;
    }
    double start_to_goal_distance = std::sqrt(
      std::pow(from_pose.position.x - goal_pose.pose.position.x, 2) +
      std::pow(from_pose.position.y - goal_pose.pose.position.y, 2) +
      std::pow(from_pose.position.z - goal_pose.pose.position.z, 2));
    traffic_simulator_msgs::msg::LaneletPose to_pose;
    to_pose.lanelet_id = lane_change_parameter.target.lanelet_id;
    to_pose.s = to_s;
    auto traj = getLaneChangeTrajectory(
      from_pose, to_pose, lane_change_parameter.trajectory_shape, start_to_goal_distance * 0.5);
    if (traj.getMaximum2DCurvature() < maximum_curvature_threshold) {
      double eval = std::fabs(target_trajectory_length - traj.getLength());
      evaluation.push_back(eval);
      curves.push_back(traj);
      target_s.push_back(to_s);
    }
  }
  if (evaluation.empty()) {
    return std::nullopt;
  }
  std::vector<double>::iterator min_itr = std::min_element(evaluation.begin(), evaluation.end());
  size_t min_index = std::distance(evaluation.begin(), min_itr);
  return std::make_pair(curves[min_index], target_s[min_index]);
}

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const double tangent_vector_size) -> math::geometry::HermiteCurve
{
  geometry_msgs::msg::Vector3 start_vec;
  geometry_msgs::msg::Vector3 to_vec;
  geometry_msgs::msg::Pose goal_pose = pose::toMapPose(to_pose).pose;
  double tangent_vector_size_in_curve = 0.0;
  switch (trajectory_shape) {
    case traffic_simulator::lane_change::TrajectoryShape::CUBIC:
      start_vec = getVectorFromPose(from_pose, tangent_vector_size);
      if (getTangentVector(to_pose.lanelet_id, to_pose.s)) {
        to_vec = getTangentVector(to_pose.lanelet_id, to_pose.s).value();
      } else {
        THROW_SIMULATION_ERROR(
          "Failed to calculate tangent vector at lanelet_id : ", to_pose.lanelet_id,
          " s : ", to_pose.s);
      }
      tangent_vector_size_in_curve = tangent_vector_size;
      break;
    case traffic_simulator::lane_change::TrajectoryShape::LINEAR:
      start_vec.x = (goal_pose.position.x - from_pose.position.x);
      start_vec.y = (goal_pose.position.y - from_pose.position.y);
      start_vec.z = (goal_pose.position.z - from_pose.position.z);
      to_vec = start_vec;
      tangent_vector_size_in_curve = 1;
      break;
  }
  return math::geometry::HermiteCurve(
    from_pose, goal_pose, start_vec,
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(to_vec.x * tangent_vector_size_in_curve)
      .y(to_vec.y * tangent_vector_size_in_curve)
      .z(to_vec.z * tangent_vector_size_in_curve));
}

namespace
{
auto getVectorFromPose(const geometry_msgs::msg::Pose & pose, const double magnitude)
  -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 dir =
    quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
  geometry_msgs::msg::Vector3 vector;
  vector.x = magnitude * std::cos(dir.z);
  vector.y = magnitude * std::sin(dir.z);
  vector.z = 0;
  return vector;
}

auto getTangentVector(const lanelet::Id lanelet_id, const double s)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  return other::getCenterPointsSpline(lanelet_id)->getTangentVector(s);
}
}  // namespace
}  // namespace lane_change
}  // namespace lanelet2
}  // namespace traffic_simulator
