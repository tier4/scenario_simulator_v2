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
std::ostream & operator<<(std::ostream & stream, const Direction & value)
{
  switch (value) {
    default:
    case Direction::STRAIGHT:
      return stream << "STRAIGHT";
    case Direction::LEFT:
      return stream << "LEFT";
    case Direction::RIGHT:
      return stream << "RIGHT";
  }
}

std::ostream & operator<<(std::ostream & stream, const TrajectoryShape & value)
{
  switch (value) {
    default:
    case TrajectoryShape::CUBIC:
      return stream << "CUBIC";
    case TrajectoryShape::LINEAR:
      return stream << "LINEAR";
  }
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Policy & value)
{
  switch (value) {
    default:
    case Constraint::Policy::FORCE:
      return stream << "FORCE";
    case Constraint::Policy::BEST_EFFORT:
      return stream << "BEST_EFFORT";
  }
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value)
{
  switch (value) {
    default:
    case Constraint::Type::NONE:
      return stream << "NONE";
    case Constraint::Type::LATERAL_VELOCITY:
      return stream << "LATERAL_VELOCITY";
    case Constraint::Type::LONGITUDINAL_DISTANCE:
      return stream << "LONGITUDINAL_DISTANCE";
    case Constraint::Type::TIME:
      return stream << "TIME";
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
