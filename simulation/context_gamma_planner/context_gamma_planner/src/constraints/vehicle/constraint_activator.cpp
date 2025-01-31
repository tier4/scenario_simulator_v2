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
#include <context_gamma_planner/constraints/vehicle/constraint_activator.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>

namespace context_gamma_planner
{
namespace vehicle
{
namespace constraints
{
using State = context_gamma_planner::constraints::State;
using Side = context_gamma_planner::constraints::RoadEdgeConstraint::Side;

ConstraintActivator::ConstraintActivator(
  const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
  const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr)
: context_gamma_planner::constraints::ConstraintActivatorBase(hd_map_utils_ptr, traffic_lights_ptr)
{
  for (const auto & id : hd_map_utils_ptr_->getTrafficLightIds()) {
    const auto stop_line_ids = hd_map_utils_ptr_->getTrafficLightStopLineIds(id);
    for (const auto stop_line_id : stop_line_ids) {
      traffic_light_constraints_.emplace_back(
        context_gamma_planner::constraints::TrafficLightConstraint(
          hd_map_utils_ptr_, id, stop_line_id));
    }
  }
  for (const auto & id : hd_map_utils_ptr_->getStopLineIds()) {
    stop_line_constraints_.emplace_back(
      context_gamma_planner::constraints::StopLineConstraint(hd_map_utils_ptr_, id));
  }
}

void ConstraintActivator::appendStopLineConstraint(
  const lanelet::Ids & route_ids,
  const std::shared_ptr<math::geometry::CatmullRomSpline> reference_trajectory,
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & entity_status,
  const double stop_line_enable_threshold, const double stop_velocity_threshold)
{
  const double horizon = std::clamp(entity_status->getTwist().linear.x * 5.0, 20.0, 50.0);
  const auto trajectory = std::make_unique<math::geometry::CatmullRomSubspline>(
    reference_trajectory, entity_status->getLaneletPose().s,
    entity_status->getLaneletPose().s + horizon);
  const auto distance_to_stopline =
    hd_map_utils_ptr_->getDistanceToStopLine(route_ids, *trajectory);
  if (distance_to_stopline) {
    //Enable stop line constraint while the car is moving
    if (not is_stoped_) {
      setStateByStopLineId(
        stop_line_constraints_, hd_map_utils_ptr_->getStopLineIdsOnPath(route_ids),
        context_gamma_planner::constraints::State::ACTIVE);
    }
    //Clear the stop line constraint when the car is paused
    if (
      distance_to_stopline.value() < stop_line_enable_threshold and
      entity_status->getTwist().linear.x < stop_velocity_threshold) {
      is_stoped_ = true;
      return;
    }
    //Allow the stop line to be reactivated once the car leaves the stop line
    if (is_stoped_ and distance_to_stopline.value() > stop_line_enable_threshold) {
      is_stoped_ = false;
    }
  }
}

void ConstraintActivator::appendTrafficLightConstraint(const lanelet::Ids & route_ids)
{
  for (const auto & traffic_light_id : hd_map_utils_ptr_->getTrafficLightIdsOnPath(route_ids)) {
    if (traffic_lights_ptr_->getTrafficLight(traffic_light_id)
          .contains(
            traffic_simulator::TrafficLight::Color::green,
            traffic_simulator::TrafficLight::Status::solid_on,
            traffic_simulator::TrafficLight::Shape::circle)) {
      continue;
    }
    setStateByStopLineId(
      traffic_light_constraints_, hd_map_utils_ptr_->getTrafficLightStopLineIds(traffic_light_id),
      context_gamma_planner::constraints::State::ACTIVE);
  }
}

void ConstraintActivator::appendLaneChangeConstraint(
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & entity_status,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter)
{
  if (!entity_status->isInLanelet()) {
    return;
  }
  std::optional<lanelet::Id> id = entity_status->getLaneletPose().lanelet_id;
  while (true) {
    id = hd_map_utils_ptr_->getLaneChangeableLaneletId(
      id.value(), traffic_simulator::lane_change::Direction::LEFT);
    if (!id) {
      break;
    }
    if (id.value() == lane_change_parameter.target.lanelet_id) {
      appendRoadEdgeConstraint(entity_status->getLaneletPose().lanelet_id, Side::RIGHT);
      appendRoadEdgeConstraint(lane_change_parameter.target.lanelet_id, Side::LEFT);
      return;
    }
  }
  id = entity_status->getLaneletPose().lanelet_id;
  while (true) {
    id = hd_map_utils_ptr_->getLaneChangeableLaneletId(
      id.value(), traffic_simulator::lane_change::Direction::RIGHT);
    if (!id) {
      break;
    }
    if (id.value() == lane_change_parameter.target.lanelet_id) {
      appendRoadEdgeConstraint(lane_change_parameter.target.lanelet_id, Side::RIGHT);
      appendRoadEdgeConstraint(entity_status->getLaneletPose().lanelet_id, Side::LEFT);
      return;
    }
  }
  if (entity_status->getLaneletPose().lanelet_id == lane_change_parameter.target.lanelet_id) {
    appendRoadEdgeConstraint(entity_status->getLaneletPose().lanelet_id);
    return;
  }
}

void ConstraintActivator::appendRoadEndConstraint(const lanelet::Ids & route_ids)
{
  if (traffic_simulator::lanelet_wrapper::lanelet_map::nextLaneletIds(route_ids.back()).empty()) {
    appendRoadEdgeConstraint(route_ids.back(), Side::FRONT);
  }
  if (traffic_simulator::lanelet_wrapper::lanelet_map::previousLaneletIds(route_ids.back())
        .empty()) {
    appendRoadEdgeConstraint(route_ids.back(), Side::BACK);
  }
}

}  // namespace constraints
}  // namespace vehicle
}  // namespace context_gamma_planner
