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

#include <algorithm>
#include <context_gamma_planner/constraints/constraint_activator_base.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>

namespace context_gamma_planner
{
namespace constraints
{
ConstraintActivatorBase::ConstraintActivatorBase(
  const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
  const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr)
: hd_map_utils_ptr_(hd_map_utils_ptr), traffic_lights_ptr_(traffic_lights_ptr), is_stoped_(false)
{
  for (const auto & id : hd_map_utils_ptr_->getLaneletIds()) {
    lane_constraints_.emplace_back(
      RoadEdgeConstraint(hd_map_utils_ptr, id, RoadEdgeConstraint::Side::RIGHT));
    lane_constraints_.emplace_back(
      RoadEdgeConstraint(hd_map_utils_ptr, id, RoadEdgeConstraint::Side::LEFT));
    lane_constraints_.emplace_back(
      RoadEdgeConstraint(hd_map_utils_ptr, id, RoadEdgeConstraint::Side::FRONT));
    lane_constraints_.emplace_back(
      RoadEdgeConstraint(hd_map_utils_ptr, id, RoadEdgeConstraint::Side::BACK));
  }
}

void ConstraintActivatorBase::deactivateAllConstraints()
{
  deactivateAll(lane_constraints_);
  deactivateAll(traffic_light_constraints_);
  deactivateAll(stop_line_constraints_);
}

std::vector<RoadEdgeConstraint> ConstraintActivatorBase::queryRoadEdgeConstraint(
  const geometry_msgs::msg::Point & p, double distance_threshold, const char subtype[])
{
  const auto ids = hd_map_utils_ptr_->filterLaneletIds(
    hd_map_utils_ptr_->getNearbyLaneletIds(p, distance_threshold), subtype);
  deactivateAll(lane_constraints_);
  setState(lane_constraints_, ids, State::ACTIVE);
  return filter(lane_constraints_, State::ACTIVE);
}

void ConstraintActivatorBase::appendLaneChangeConstraint(
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &,
  const traffic_simulator::lane_change::Parameter &)
{
  THROW_SIMULATION_ERROR(
    "Currently, appendLaneChangeConstraint does not supported in this entity.",
    "This message is mainly for developers.",
    "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
}

void ConstraintActivatorBase::appendStopLineConstraint(
  const lanelet::Ids &, const std::shared_ptr<math::geometry::CatmullRomSpline>,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &, const double, const double)
{
  THROW_SIMULATION_ERROR(
    "Currently, appendStopLineConstraint does not supported in this entity.",
    "This message is mainly for developers.",
    "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
}

void ConstraintActivatorBase::appendTrafficLightConstraint(const lanelet::Ids &)
{
  THROW_SIMULATION_ERROR(
    "Currently, appendTrafficLightConstraint does not supported in this entity.",
    "This message is mainly for developers.",
    "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
}

void ConstraintActivatorBase::appendPedestrianRouteConstraint(
  const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &, const lanelet::Ids &,
  const std::vector<geometry_msgs::msg::Pose> &)
{
  THROW_SIMULATION_ERROR(
    "Currently, appendPedestrianRouteConstraint does not supported in this entity.",
    "This message is mainly for developers.",
    "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
}

void ConstraintActivatorBase::appendRoadEndConstraint(const lanelet::Ids &)
{
  THROW_SIMULATION_ERROR(
    "Currently, appendRoadEndConstraint does not supported in this entity.",
    "This message is mainly for developers.",
    "If you are not developer, please notify to Masaya Kataoka (@hakuturu583)");
}

void ConstraintActivatorBase::appendRoadEdgeConstraint(
  const lanelet::Id lanelet_id, const State & state)
{
  appendRoadEdgeConstraint(lanelet::Ids({lanelet_id}), state);
}

void ConstraintActivatorBase::appendRoadEdgeConstraint(
  const lanelet::Ids & lanelet_ids, const State & state)
{
  setState(lane_constraints_, lanelet_ids, state);
}

void ConstraintActivatorBase::appendPreviousRoadEdgeConstraint(const lanelet::Ids & lanelet_ids)
{
  if (!traffic_simulator::lanelet_wrapper::lanelet_map::previousLaneletIds(lanelet_ids.front())
         .empty()) {
    appendRoadEdgeConstraint(
      traffic_simulator::lanelet_wrapper::lanelet_map::previousLaneletIds(lanelet_ids.front())
        .front(),
      State::ACTIVE);
  }
}

void ConstraintActivatorBase::appendRoadEdgeConstraint(
  const lanelet::Id lanelet_id, const RoadEdgeConstraint::Side & side, const State & state)
{
  setState(lane_constraints_, lanelet_id, side, state);
}
}  // namespace constraints
}  // namespace context_gamma_planner
