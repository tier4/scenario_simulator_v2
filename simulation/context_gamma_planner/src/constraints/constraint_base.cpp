// Copyright 2021 Tier IV, Inc All rights reserved.
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

#include <context_gamma_planner/constraints/constraint_base.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace context_gamma_planner
{
namespace constraints
{
ConstraintBase::ConstraintBase(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const std::string & type,
  const Polygons & polygons)
: type(type), hdmap_utils_ptr_(hdmap_utils), polygons_(polygons)
{
  state_ = State::INACTIVE;
}

ConstraintBase::ConstraintBase(const ConstraintBase & obj)
: type(obj.type),
  hdmap_utils_ptr_(obj.hdmap_utils_ptr_),
  polygons_(obj.getPolygons()),
  state_(obj.state_)
{
}

std::ostream & operator<<(std::ostream & os, const Polygon & polygon)
{
  os << "================= polygon =================" << std::endl;
  size_t index = 0;
  for (const auto & point : polygon) {
    os << "point[" << index << "], (x,y,z) = (" << point.x << "," << point.y << "," << point.z
       << ")" << std::endl;
    index++;
  }
  os << "================= polygon =================" << std::endl;
  return os;
}

std::ostream & operator<<(std::ostream & os, const Polygons & polygons)
{
  size_t index = 0;
  for (const auto & polygon : polygons) {
    os << "polygon [" << index << "]" << std::endl;
    os << polygon;
    index++;
  }
  return os;
}

const Polygons & ConstraintBase::getPolygons() const { return polygons_; }

ConstraintBase ConstraintBase::operator+(ConstraintBase constraint)
{
  if (state_ == State::ACTIVE && constraint.getState() == State::ACTIVE) {
    std::copy(
      constraint.polygons_.begin(), constraint.polygons_.end(),
      std::back_inserter(this->polygons_));
  } else if (constraint.getState() == State::ACTIVE) {
    return ConstraintBase(this->hdmap_utils_ptr_, OVERRAY_CONSTRAINT, constraint.getPolygons());
  }
  return ConstraintBase(this->hdmap_utils_ptr_, OVERRAY_CONSTRAINT, this->polygons_);
}

ConstraintBase ConstraintBase::operator=(ConstraintBase constraint) { return constraint; }

void ConstraintBase::setState(const State & state) { state_ = state; }

const State & ConstraintBase::getState() const { return state_; }

std::ostream & operator<<(std::ostream & os, const ConstraintBase & constraint)
{
  os << "type : " << constraint.type << std::endl;
  if (constraint.getState() == State::ACTIVE) {
    os << "state : active " << std::endl;
  }
  if (constraint.getState() == State::INACTIVE) {
    os << "state : inactive " << std::endl;
  }
  os << constraint.getPolygons();
  return os;
}
}  // namespace constraints
}  // namespace context_gamma_planner
