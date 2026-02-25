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

#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/hypot.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lane_change.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace lane_change
{
auto canChangeLane(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id, const RoutingGraphType type)
  -> bool
{
  const auto from_lanelet = LaneletWrapper::map()->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = LaneletWrapper::map()->laneletLayer.get(to_lanelet_id);
  return LaneletWrapper::trafficRules(type)->canChangeLane(from_lanelet, to_lanelet);
}

auto laneChangeableLaneletId(
  const lanelet::Id lanelet_id, const Direction & direction, const RoutingGraphType type)
  -> std::optional<lanelet::Id>
{
  const auto lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  if (direction == Direction::STRAIGHT) {
    return lanelet.id();
  } else if (direction == Direction::LEFT && LaneletWrapper::routingGraph(type)->left(lanelet)) {
    return LaneletWrapper::routingGraph(type)->left(lanelet)->id();
  } else if (direction == Direction::RIGHT && LaneletWrapper::routingGraph(type)->right(lanelet)) {
    return LaneletWrapper::routingGraph(type)->right(lanelet)->id();
  } else {
    return std::nullopt;
  }
}

auto laneChangeableLaneletId(
  const lanelet::Id lanelet_id, const Direction & direction, const std::uint8_t shift,
  const RoutingGraphType type) -> std::optional<lanelet::Id>
{
  if (shift == 0) {
    return laneChangeableLaneletId(lanelet_id, Direction::STRAIGHT, type);
  } else {
    auto reference_id = lanelet_id;
    for (std::size_t i = 0; i < shift; ++i) {
      if (const auto id_opt = laneChangeableLaneletId(reference_id, direction, type); !id_opt) {
        return std::nullopt;
      } else {
        reference_id = id_opt.value();
      }
    }
    return reference_id;
  }
}

auto countLaneChanges(
  const lanelet::Id & from_lanelet_id, const lanelet::Id & to_lanelet_id,
  const RoutingConfiguration & routing_configuration) -> std::optional<std::pair<int, int>>
{
  constexpr bool include_opposite_direction{false};
  const auto route = route::routeFromGraph(from_lanelet_id, to_lanelet_id, routing_configuration);
  if (route.empty()) {
    return std::nullopt;
  } else {
    std::pair<int, int> lane_changes{0, 0};
    for (std::size_t i = 1; i < route.size(); ++i) {
      const auto & previous = route[i - 1];
      const auto & current = route[i];
      if (const auto followings =
            lanelet_map::nextLaneletIds(previous, routing_configuration.routing_graph_type);
          std::find(followings.begin(), followings.end(), current) != followings.end()) {
        continue;
      } else if (const auto lefts = pose::leftLaneletIds(
                   previous, routing_configuration.routing_graph_type, include_opposite_direction);
                 std::find(lefts.begin(), lefts.end(), current) != lefts.end()) {
        lane_changes.first++;
      } else if (const auto rights = pose::rightLaneletIds(
                   previous, routing_configuration.routing_graph_type, include_opposite_direction);
                 std::find(rights.begin(), rights.end(), current) != rights.end()) {
        lane_changes.second++;
      }
    }
    return lane_changes;
  }
}

// Trajectory
auto laneChangeTrajectory(
  const Pose & from_pose, const LaneletPose & to_lanelet_pose,
  const TrajectoryShape & trajectory_shape, const double tangent_vector_size) -> Curve
{
  const auto to_pose = pose::toMapPose(to_lanelet_pose).pose;

  auto vectorFromPose = [](const Pose & pose, const double magnitude) -> Vector3 {
    const auto direction = math::geometry::convertQuaternionToEulerAngle(pose.orientation);
    return geometry_msgs::build<Vector3>()
      .x(magnitude * std::cos(direction.z))
      .y(magnitude * std::sin(direction.z))
      .z(0);
  };

  auto tangentVector = [](const lanelet::Id lanelet_id, const double s) -> std::optional<Vector3> {
    return lanelet_map::centerPointsSpline(lanelet_id)->getTangentVector(s);
  };

  double tangent_vector_size_in_curve{0.0};
  Vector3 from_vector;
  Vector3 to_vector;
  switch (trajectory_shape) {
    case TrajectoryShape::CUBIC:
      tangent_vector_size_in_curve = tangent_vector_size;
      from_vector = vectorFromPose(from_pose, tangent_vector_size);
      if (
        const auto tangent_vector = tangentVector(to_lanelet_pose.lanelet_id, to_lanelet_pose.s)) {
        to_vector = tangent_vector.value();
      } else {
        THROW_SIMULATION_ERROR(
          "Failed to calculate tangent vector at lanelet_id : ", to_lanelet_pose.lanelet_id,
          " s : ", to_lanelet_pose.s);
      }
      break;
    case TrajectoryShape::LINEAR:
      tangent_vector_size_in_curve = 1;
      from_vector.x = (to_pose.position.x - from_pose.position.x);
      from_vector.y = (to_pose.position.y - from_pose.position.y);
      from_vector.z = (to_pose.position.z - from_pose.position.z);
      to_vector = from_vector;
      break;
    default:
      throw std::invalid_argument("Unknown trajectory_shape constraint type");
  }

  return Curve(
    from_pose, to_pose, from_vector,
    geometry_msgs::build<Vector3>()
      .x(to_vector.x * tangent_vector_size_in_curve)
      .y(to_vector.y * tangent_vector_size_in_curve)
      .z(to_vector.z * tangent_vector_size_in_curve));
}

auto laneChangeTrajectory(
  const LaneletPose & from_lanelet_pose, const Parameter & lane_change_parameter)
  -> std::optional<std::pair<Curve, double>>
{
  const double longitudinal_distance =
    lane_change_parameter.constraint.type == Constraint::Type::LONGITUDINAL_DISTANCE
      ? lane_change_parameter.constraint.value
      : Parameter::default_lanechange_distance;

  const auto along_lanelet_pose = pose::alongLaneletPose(from_lanelet_pose, longitudinal_distance);
  auto left_boundary_lanelet_pose = along_lanelet_pose;
  left_boundary_lanelet_pose.offset += 5.0;
  auto right_boundary_lanelet_pose = along_lanelet_pose;
  right_boundary_lanelet_pose.offset -= 5.0;

  const auto left_boundary_point = pose::toMapPose(left_boundary_lanelet_pose).pose.position;
  const auto right_boundary_point = pose::toMapPose(right_boundary_lanelet_pose).pose.position;
  if (const auto to_lanelet_pose_s =
        lanelet_map::centerPointsSpline(lane_change_parameter.target.lanelet_id)
          ->getCollisionPointIn2D(left_boundary_point, right_boundary_point);
      !to_lanelet_pose_s) {
    return std::nullopt;
  } else {
    const auto to_lanelet_pose = helper::constructLaneletPose(
      lane_change_parameter.target.lanelet_id, to_lanelet_pose_s.value(),
      lane_change_parameter.target.offset);
    const auto from_pose = pose::toMapPose(from_lanelet_pose).pose;
    const auto to_pose = pose::toMapPose(to_lanelet_pose).pose;
    const auto euclidean_distance = math::geometry::hypot(from_pose.position, to_pose.position);
    const auto lane_change_trajectory = laneChangeTrajectory(
      from_pose, to_lanelet_pose, lane_change_parameter.trajectory_shape, euclidean_distance * 0.5);
    return std::make_pair(lane_change_trajectory, to_lanelet_pose_s.value());
  }
}

auto laneChangeTrajectory(
  const Pose & from_pose, const Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold) -> std::optional<std::pair<Curve, double>>
{
  std::vector<double> candidates_evaluation;
  std::vector<double> candidates_s;
  std::vector<Curve> candidates_curves;

  const auto lanelet_length = lanelet_map::laneletLength(lane_change_parameter.target.lanelet_id);
  for (double to_lanelet_pose_s = 0; to_lanelet_pose_s < lanelet_length; to_lanelet_pose_s += 1.0) {
    const auto to_lanelet_pose =
      helper::constructLaneletPose(lane_change_parameter.target.lanelet_id, to_lanelet_pose_s, 0.0);
    // skip those poses that are too close
    if (const auto to_pose = pose::toMapPose(to_lanelet_pose).pose;
        math::geometry::getRelativePose(from_pose, to_pose).position.x <=
        forward_distance_threshold) {
      continue;
    } else {
      const auto euclidean_distance = math::geometry::hypot(from_pose.position, to_pose.position);
      if (const auto lane_change_trajectory = laneChangeTrajectory(
            from_pose, to_lanelet_pose, lane_change_parameter.trajectory_shape,
            euclidean_distance * 0.5);
          lane_change_trajectory.getMaximum2DCurvature() < maximum_curvature_threshold) {
        candidates_evaluation.push_back(
          std::fabs(target_trajectory_length - lane_change_trajectory.getLength()));
        candidates_curves.push_back(lane_change_trajectory);
        candidates_s.push_back(to_lanelet_pose_s);
      }
    }
  }

  if (candidates_evaluation.empty()) {
    return std::nullopt;
  } else {
    const auto min_iterator =
      std::min_element(candidates_evaluation.begin(), candidates_evaluation.end());
    const auto min_index = std::distance(candidates_evaluation.begin(), min_iterator);
    return std::make_pair(candidates_curves[min_index], candidates_s[min_index]);
  }
}
}  // namespace lane_change
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
