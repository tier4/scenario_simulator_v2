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

#include <traffic_simulator/data_type/lane_change.hpp>

namespace traffic_simulator
{
namespace lane_change
{
static_assert(std::is_default_constructible_v<Constraint>);
static_assert(std::is_copy_constructible_v<Constraint>);
static_assert(std::is_move_constructible_v<Constraint>);
static_assert(std::is_copy_assignable_v<Constraint>);
static_assert(std::is_move_assignable_v<Constraint>);

static_assert(std::is_default_constructible_v<AbsoluteTarget>);
static_assert(std::is_copy_constructible_v<AbsoluteTarget>);
static_assert(std::is_move_constructible_v<AbsoluteTarget>);
static_assert(std::is_copy_assignable_v<AbsoluteTarget>);
static_assert(std::is_move_assignable_v<AbsoluteTarget>);

static_assert(not std::is_default_constructible_v<RelativeTarget>);
static_assert(std::is_copy_constructible_v<RelativeTarget>);
static_assert(std::is_move_constructible_v<RelativeTarget>);
static_assert(std::is_copy_assignable_v<RelativeTarget>);
static_assert(std::is_move_assignable_v<RelativeTarget>);

static_assert(std::is_default_constructible_v<Parameter>);
static_assert(std::is_copy_constructible_v<Parameter>);
static_assert(std::is_move_constructible_v<Parameter>);
static_assert(std::is_copy_assignable_v<Parameter>);
static_assert(std::is_move_assignable_v<Parameter>);

std::ostream & operator<<(std::ostream & stream, const Direction & value)
{
  switch (value) {
    default:
    case Direction::straight:
      return stream << "straight";
    case Direction::left:
      return stream << "left";
    case Direction::right:
      return stream << "right";
  }
}

std::ostream & operator<<(std::ostream & stream, const TrajectoryShape & value)
{
  switch (value) {
    default:
    case TrajectoryShape::cubic:
      return stream << "cubic";
    case TrajectoryShape::linear:
      return stream << "linear";
  }
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Policy & value)
{
  switch (value) {
    default:
    case Constraint::Policy::force:
      return stream << "force";
    case Constraint::Policy::best_effort:
      return stream << "best_effort";
  }
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value)
{
  switch (value) {
    default:
    case Constraint::Type::none:
      return stream << "none";
    case Constraint::Type::lateral_velocity:
      return stream << "lateral_velocity";
    case Constraint::Type::longitudinal_distance:
      return stream << "longitudinal_distance";
    case Constraint::Type::time:
      return stream << "time";
  }
}

std::ostream & operator<<(std::ostream & stream, const Parameter & parameter)
{
  return stream << "Parameter:\n"         //
                << "  AbsoluteTarget:\n"  //
                << "    lanelet_id: " << parameter.target.lanelet_id << "\n"
                << "    offset: " << parameter.target.offset << "\n"
                << "  TrajectoryShape: " << parameter.trajectory_shape << "\n"
                << "  Constraint:\n"
                << "    type: " << parameter.constraint.type << "\n"
                << "    value: " << parameter.constraint.value << "\n"
                << "    policy: " << parameter.constraint.policy << std::endl;
}
}  // namespace lane_change
}  // namespace traffic_simulator
