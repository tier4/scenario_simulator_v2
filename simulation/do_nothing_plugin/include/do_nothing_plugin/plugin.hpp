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

#ifndef DO_NOTHING_PLUGIN__PEDESTRIAN__PLUGIN_HPP_
#define DO_NOTHING_PLUGIN__PEDESTRIAN__PLUGIN_HPP_

#include <traffic_simulator/behavior/behavior_plugin_base.hpp>

namespace entity_behavior
{
class DoNothingBehavior : public BehaviorPluginBase
{
public:
  /**
   * @brief just update timestamp of entity_status_ member variable.
   * @param current_time current time in scenario time
   * @param step_time step time of the simulation, this argument exists for other BehaviorPlugin classes but are not used by this plugin.
   */
  void update(double current_time, double step_time) override;
  /**
   * @brief setup rclcpp::logger for debug output, but there is no debug output in this plugin.
   * @param logger logger for debug output, this argument exists for other BehaviorPlugin classes but are not used by this plugin.
   */
  void configure(const rclcpp::Logger & logger) override;
  /**
   * @brief Get the Current Action object
   * @return const std::string& always return "do_nothing"
   */
  const std::string & getCurrentAction() const override;

/// @note Getters defined by this macro return default values and setters are behaved as no-operation functions.
#define DEFINE_GETTER_SETTER(NAME, TYPE)        \
public:                                         \
  TYPE get##NAME() override { return TYPE(); }; \
  void set##NAME(const TYPE &) override{};
  // clang-format off
  DEFINE_GETTER_SETTER(DebugMarker,                                      std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DefaultMatchingDistanceForLaneletPoseCalculation, double)
  DEFINE_GETTER_SETTER(GoalPoses,                                        std::vector<geometry_msgs::msg::Pose>)
  DEFINE_GETTER_SETTER(LaneChangeParameters,                             traffic_simulator::lane_change::Parameter)
  DEFINE_GETTER_SETTER(Obstacle,                                         std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus,                                EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters,                             traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(ReferenceTrajectory,                              std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_GETTER_SETTER(RouteLanelets,                                    lanelet::Ids)
  DEFINE_GETTER_SETTER(TargetSpeed,                                      std::optional<double>)
  DEFINE_GETTER_SETTER(TrafficLightManager,                              std::shared_ptr<traffic_simulator::TrafficLightManager>)
  DEFINE_GETTER_SETTER(VehicleParameters,                                traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints,                                        traffic_simulator_msgs::msg::WaypointsArray)
  // clang-format on
#undef DEFINE_GETTER_SETTER

/// @note Getters defined by this macro return stored values and setters store values.
#define DEFINE_GETTER_SETTER(NAME, TYPE, FIELD_NAME)                   \
public:                                                                \
  TYPE get##NAME() override { return FIELD_NAME; };                    \
  void set##NAME(const TYPE & value) override { FIELD_NAME = value; }; \
                                                                       \
private:                                                               \
  TYPE FIELD_NAME;
  // clang-format off
  DEFINE_GETTER_SETTER(BehaviorParameter,         traffic_simulator_msgs::msg::BehaviorParameter,                   behavior_parameter_)
  DEFINE_GETTER_SETTER(CanonicalizedEntityStatus, std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>,    canonicalized_entity_status_)
  DEFINE_GETTER_SETTER(CurrentTime,               double,                                                           current_time_)
  DEFINE_GETTER_SETTER(HdMapUtils,                std::shared_ptr<hdmap_utils::HdMapUtils>,                         hdmap_utils_)
  DEFINE_GETTER_SETTER(PolylineTrajectory,        std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>, polyline_trajectory)
  DEFINE_GETTER_SETTER(Request,                   traffic_simulator::behavior::Request,                             request)
  DEFINE_GETTER_SETTER(StepTime,                  double,                                                           step_time_)
  // clang-format on
#undef DEFINE_GETTER_SETTER
};
}  // namespace entity_behavior

#endif  // DO_NOTHING_PLUGIN__PEDESTRIAN__PLUGIN_HPP_
