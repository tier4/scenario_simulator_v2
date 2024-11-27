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

#ifndef CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_BASE_HPP_
#define CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_BASE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

#define OVERRAY_CONSTRAINT "overlay"

namespace context_gamma_planner
{
namespace constraints
{
typedef std::vector<geometry_msgs::msg::Point> Polygon;
std::ostream & operator<<(std::ostream & os, const Polygon & polygon);

typedef std::vector<Polygon> Polygons;
std::ostream & operator<<(std::ostream & os, const Polygons & polygons);

enum class State { ACTIVE = 0, INACTIVE = 1 };

class ConstraintBase
{
public:
  ConstraintBase(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const std::string & type,
    const Polygons & polygons);
  ConstraintBase(const ConstraintBase & obj);
  const Polygons & getPolygons() const;
  const std::string type;
  ConstraintBase operator+(ConstraintBase constraint);
  ConstraintBase operator=(ConstraintBase constraint);
  void setState(const State & state);
  const State & getState() const;

protected:
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

private:
  Polygons polygons_;
  State state_;
};

std::ostream & operator<<(std::ostream & os, const ConstraintBase & constraint);

template <typename Constraint>
void setStateByStopLineId(
  std::vector<Constraint> & constraints, const lanelet::Id lanelet_id,
  const State & state = State::ACTIVE)
{
  std::for_each(constraints.begin(), constraints.end(), [&](auto & constraint) {
    if (constraint.stop_line_id == lanelet_id) {
      constraint.setState(state);
    }
  });
}

template <typename Constraint>
void setStateByStopLineId(
  std::vector<Constraint> & constraints, const std::vector<lanelet::Id> & lanelet_ids,
  const State & state = State::ACTIVE)
{
  std::for_each(lanelet_ids.begin(), lanelet_ids.end(), [&](const auto lanelet_id) {
    setStateByStopLineId(constraints, lanelet_id, state);
  });
}

}  // namespace constraints
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_BASE_HPP_
