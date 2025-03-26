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

#include <context_gamma_planner/constraints/road_edge_constraint.hpp>

namespace context_gamma_planner
{
namespace constraints
{
RoadEdgeConstraint::RoadEdgeConstraint(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const lanelet::Id lanelet_id,
  const Side & side)
: ConstraintBase(hdmap_utils, LANE_CONSTRAINT, {[&]() {
                   switch (side) {
                     case Side::RIGHT:
                       return hdmap_utils->getRightBound(lanelet_id);
                     case Side::LEFT:
                       return hdmap_utils->getLeftBound(lanelet_id);
                     case Side::FRONT: {
                       context_gamma_planner::constraints::Polygon front_bound;
                       front_bound.push_back(hdmap_utils->getRightBound(lanelet_id).back());
                       front_bound.push_back(hdmap_utils->getLeftBound(lanelet_id).back());
                       return front_bound;
                     }
                     case Side::BACK: {
                       context_gamma_planner::constraints::Polygon back_bound;
                       back_bound.push_back(hdmap_utils->getRightBound(lanelet_id).front());
                       back_bound.push_back(hdmap_utils->getLeftBound(lanelet_id).front());
                       return back_bound;
                     }
                   }
                   THROW_SIMULATION_ERROR(
                     "Please check side of the lanelet, something completely wrong happened.",
                     "This message is mainly for developers.",
                     "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
                 }()}),
  lanelet_id(lanelet_id),
  side(side)
{
}

// void setState(
//   std::vector<RoadEdgeConstraint> & constraints, const std::vector<lanelet::Id> & lanelet_ids,
//   const RoadEdgeConstraint::Side & side, const State & state)
// {
// }

void setState(
  std::vector<RoadEdgeConstraint> & constraints, const lanelet::Id lanelet_id,
  const RoadEdgeConstraint::Side & side, const State & state)
{
  auto result =
    std::find_if(constraints.begin(), constraints.end(), [lanelet_id, side](RoadEdgeConstraint x) {
      return x.lanelet_id == lanelet_id && x.side == side;
    });
  if (result != constraints.end()) {
    result->setState(state);
  }
}

void setState(
  std::vector<RoadEdgeConstraint> & constraints, const std::vector<lanelet::Id> & lanelet_ids,
  const State & state)
{
  for (const auto & id : lanelet_ids) {
    setState(constraints, id, RoadEdgeConstraint::Side::RIGHT, state);
    setState(constraints, id, RoadEdgeConstraint::Side::LEFT, state);
  }
}

}  // namespace constraints
}  // namespace context_gamma_planner
