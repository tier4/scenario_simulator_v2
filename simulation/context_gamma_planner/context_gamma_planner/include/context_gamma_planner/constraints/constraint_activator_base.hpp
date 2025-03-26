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

#ifndef CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
#define CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_

#include <customized_rvo2/RVO.h>

#include <context_gamma_planner/constraints/road_edge_constraint.hpp>
#include <context_gamma_planner/constraints/stop_line_constraint.hpp>
#include <context_gamma_planner/constraints/traffic_light_constraint.hpp>
#include <geometry/spline/catmull_rom_subspline.hpp>
#include <memory>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>

namespace context_gamma_planner
{
namespace constraints
{
template <typename T>
void activateAll(std::vector<T> & constraints)
{
  for (auto & constraint : constraints) {
    constraint.setState(State::ACTIVE);
  }
}

template <typename T>
void deactivateAll(std::vector<T> & constraints)
{
  for (auto & constraint : constraints) {
    constraint.setState(State::INACTIVE);
  }
}

template <typename T>
std::vector<T> filter(std::vector<T> & constraints, const State & state)
{
  std::vector<T> filtered;
  std::for_each(constraints.begin(), constraints.end(), [state, &filtered](T & x) mutable -> void {
    if (x.getState() == state) {
      filtered.emplace_back(x);
    }
  });
  return filtered;
}

class ConstraintActivatorBase
{
public:
  ConstraintActivatorBase(
    const std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr,
    const std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr);
  const auto calculateRVOObstacles() -> std::vector<std::vector<RVO::Vector2> >;
  void deactivateAllConstraints();
  void appendRoadEdgeConstraint(const lanelet::Id lanelet_id, const State & state = State::ACTIVE);
  void appendRoadEdgeConstraint(
    const lanelet::Id lanelet_id, const RoadEdgeConstraint::Side & side,
    const State & state = State::ACTIVE);
  void appendRoadEdgeConstraint(
    const lanelet::Ids & route_ids, const State & state = State::ACTIVE);

  virtual void appendLaneChangeConstraint(
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &,
    const traffic_simulator::lane_change::Parameter &);
  /// @param stop_velocity_threshold [m/s]  Set a small value since a complete stop is not possible.
  virtual void appendStopLineConstraint(
    const lanelet::Ids &, const std::shared_ptr<math::geometry::CatmullRomSpline>,
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> &, const double = 4.0,
    const double = 0.1);
  virtual void appendTrafficLightConstraint(const lanelet::Ids &);
  /// @todo This function needs to be divided into a route planning function and a part that adds constraints.
  virtual void appendPedestrianRouteConstraint(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &, const lanelet::Ids &,
    const std::vector<geometry_msgs::msg::Pose> &);
  virtual void appendRoadEndConstraint(const lanelet::Ids &);
  void appendPreviousRoadEdgeConstraint(const lanelet::Ids &);

private:
  void appendPolygonToRVOObstacles(const Polygon & polygon);
  void appendPolygonsToRVOObstacles(const Polygons & polygon);

protected:
  std::vector<RoadEdgeConstraint> queryRoadEdgeConstraint(
    const geometry_msgs::msg::Point & p, double distance_threshold, const char subtype[]);
  std::shared_ptr<hdmap_utils::HdMapUtils> hd_map_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightsBase> traffic_lights_ptr_;
  std::vector<std::string> lanelet_subtypes_;
  std::vector<TrafficLightConstraint> traffic_light_constraints_;
  std::vector<RoadEdgeConstraint> lane_constraints_;
  std::vector<StopLineConstraint> stop_line_constraints_;
  std::vector<std::vector<RVO::Vector2> > obstacles_;
  lanelet::Ids route_ids_;
  bool is_stoped_;
};
}  // namespace constraints
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__CONSTRAINTS__CONSTRAINT_ACTIVATOR_HPP_
