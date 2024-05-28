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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_LANE_CHANGE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_LANE_CHANGE_HPP_

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet_map/lanelet_map.hpp>

#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace lane_change
{
enum class Direction { STRAIGHT = 0, LEFT = 1, RIGHT = 2 };
enum class TrajectoryShape { CUBIC = 0, LINEAR = 1 };

struct AbsoluteTarget
{
  explicit constexpr AbsoluteTarget(const lanelet::Id lanelet_id = 0, const double offset = 0)
  : lanelet_id(lanelet_id), offset(offset)
  {
  }
  lanelet::Id lanelet_id;
  double offset;
};

struct Constraint
{
  enum class Type { NONE = 0, LATERAL_VELOCITY = 1, LONGITUDINAL_DISTANCE = 2, TIME = 3 };
  enum class Policy { FORCE = 0, BEST_EFFORT = 1 };
  explicit constexpr Constraint(
    const Type type = Type::NONE, const double value = 0, const Policy policy = Policy::FORCE)
  : type(type), value(value), policy(policy)
  {
  }
  Type type;
  double value;
  Policy policy;
};

struct RelativeTarget
{
  explicit RelativeTarget(
    const std::string & entity_name, const Direction direction, const std::uint8_t shift = 0,
    const double offset = 0)
  : entity_name(entity_name), direction(direction), shift(shift), offset(offset)
  {
  }
  std::string entity_name;
  Direction direction;
  std::uint8_t shift;
  double offset;
};

struct Parameter
{
  explicit Parameter(
    const AbsoluteTarget & target = AbsoluteTarget(),
    const TrajectoryShape trajectory_shape = TrajectoryShape::CUBIC,
    const Constraint & constraint = Constraint())
  : target(target), trajectory_shape(trajectory_shape), constraint(constraint)
  {
  }
  AbsoluteTarget target;
  TrajectoryShape trajectory_shape;
  Constraint constraint;
  static inline constexpr auto default_lanechange_distance = 20.0;
};

auto canChangeLane(const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id) -> bool;

auto getAlongLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose, const double along)
  -> traffic_simulator_msgs::msg::LaneletPose;

auto getLaneChangeableLaneletId(const lanelet::Id, const Direction) -> std::optional<lanelet::Id>;

auto getLaneChangeableLaneletId(const lanelet::Id, const Direction, const std::uint8_t shift)
  -> std::optional<lanelet::Id>;

auto getLaneChangeTrajectory(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const Parameter & lane_change_parameter)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose, const Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose, const TrajectoryShape trajectory_shape,
  const double tangent_vector_size) -> math::geometry::HermiteCurve;

// private for lane_change namespace
namespace
{
auto getVectorFromPose(const geometry_msgs::msg::Pose & pose, const double magnitude)
  -> geometry_msgs::msg::Vector3;

auto getTangentVector(const lanelet::Id lanelet_id, const double s)
  -> std::optional<geometry_msgs::msg::Vector3>;
}  // namespace

std::ostream & operator<<(std::ostream &, const Direction &);
std::ostream & operator<<(std::ostream &, const TrajectoryShape &);
std::ostream & operator<<(std::ostream &, const Constraint::Type &);
std::ostream & operator<<(std::ostream &, const Constraint::Policy &);
std::ostream & operator<<(std::ostream &, const Parameter &);
}  // namespace lane_change
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_LANE_CHANGE_HPP_
