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

#ifndef CONTEXT_GAMMA_PLANNER__CONSTRAINTS__LANE_CONSTRAINT_HPP_
#define CONTEXT_GAMMA_PLANNER__CONSTRAINTS__LANE_CONSTRAINT_HPP_

#include <context_gamma_planner/constraints/constraint_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>

#define LANE_CONSTRAINT "lane"

namespace context_gamma_planner
{
namespace constraints
{
class RoadEdgeConstraint : public ConstraintBase
{
public:
  enum class Side { LEFT = 0, RIGHT = 1, FRONT = 2, BACK = 3 };
  RoadEdgeConstraint(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Id lanelet_id,
    const Side & side);
  const lanelet::Id lanelet_id;
  const Side side;
};

void setState(
  std::vector<RoadEdgeConstraint> & constraints, const lanelet::Id lanelet_ids,
  const RoadEdgeConstraint::Side & side, const State & state = State::ACTIVE);

void setState(
  std::vector<RoadEdgeConstraint> & constraints, const std::vector<lanelet::Id> & lanelet_ids,
  const State & state = State::ACTIVE);
}  // namespace constraints
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__CONSTRAINTS__LANE_CONSTRAINT_HPP_
