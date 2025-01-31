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

#ifndef CONTEXT_GAMMA_PLANNER__VEHICLE_PLUGIN_BASE_HPP_
#define CONTEXT_GAMMA_PLANNER__VEHICLE_PLUGIN_BASE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <context_gamma_planner/constraints/vehicle/constraint_activator.hpp>
#include <context_gamma_planner/mock/catalogs.hpp>
#include <context_gamma_planner/transition_events/transition_events.hpp>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>

namespace context_gamma_planner
{
class VehiclePlugin : public entity_behavior::BehaviorPluginBase
{
public:
  void configure(const rclcpp::Logger & logger) override;
  void update(double current_time, double step_time) override;
  const std::string & getCurrentAction() const override;
#define DEFINE_GETTER_SETTER(NAME, TYPE)                                                  \
  TYPE get##NAME() override                                                               \
  {                                                                                       \
    TYPE value;                                                                           \
    try {                                                                                 \
      value = tree_.rootBlackboard()->get<TYPE>(get##NAME##Key());                        \
    } catch (const std::runtime_error & e) {                                              \
      THROW_SIMULATION_ERROR("the value : ", get##NAME##Key(), " is empty.\n", e.what()); \
    }                                                                                     \
    return value;                                                                         \
  }                                                                                       \
  void set##NAME(const TYPE & value) override                                             \
  {                                                                                       \
    tree_.rootBlackboard()->set<TYPE>(get##NAME##Key(), value);                           \
  }

  // clang-format off
  DEFINE_GETTER_SETTER(BehaviorParameter,    traffic_simulator_msgs::msg::BehaviorParameter)
  DEFINE_GETTER_SETTER(CanonicalizedEntityStatus, std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>)
  DEFINE_GETTER_SETTER(CurrentTime,          double)
  DEFINE_GETTER_SETTER(DebugMarker,          std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DefaultMatchingDistanceForLaneletPoseCalculation, double)
  DEFINE_GETTER_SETTER(PolylineTrajectory,   std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>)
  DEFINE_GETTER_SETTER(HdMapUtils,           std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(LaneChangeParameters, traffic_simulator::lane_change::Parameter)
  DEFINE_GETTER_SETTER(Obstacle,             std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus,    entity_behavior::EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters, traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(Request,              traffic_simulator::behavior::Request)
  DEFINE_GETTER_SETTER(RouteLanelets,        std::vector<lanelet::Id>)
  DEFINE_GETTER_SETTER(ReferenceTrajectory,  std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_GETTER_SETTER(StepTime,             double)
  DEFINE_GETTER_SETTER(TargetSpeed,          std::optional<double>)
  DEFINE_GETTER_SETTER(TrafficLights,        std::shared_ptr<traffic_simulator::TrafficLightsBase>)
  DEFINE_GETTER_SETTER(VehicleParameters,    traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints,            traffic_simulator_msgs::msg::WaypointsArray)
  DEFINE_GETTER_SETTER(GoalPoses,            std::vector<geometry_msgs::msg::Pose>)
  // clang-format on

#undef DEFINE_GETTER_SETTER

// The blackboard value not defined in entity_behavior::BehaviorPluginBase is defined below.
#define DEFINE_GETTER_SETTER(NAME, KEY, TYPE)                                             \
  auto get##NAME##Key() const -> const std::string &                                      \
  {                                                                                       \
    static const std::string key = KEY;                                                   \
    return key;                                                                           \
  }                                                                                       \
  TYPE get##NAME()                                                                        \
  {                                                                                       \
    TYPE value;                                                                           \
    try {                                                                                 \
      value = tree_.rootBlackboard()->get<TYPE>(get##NAME##Key());                        \
    } catch (const std::runtime_error & e) {                                              \
      THROW_SIMULATION_ERROR("the value : ", get##NAME##Key(), " is empty.\n", e.what()); \
    }                                                                                     \
    return value;                                                                         \
  }                                                                                       \
  void set##NAME(const TYPE & value) { tree_.rootBlackboard()->set<TYPE>(get##NAME##Key(), value); }
  // clang-format off
  DEFINE_GETTER_SETTER(ConstraintActivator, "activator", std::shared_ptr<context_gamma_planner::constraints::ConstraintActivatorBase>)
  DEFINE_GETTER_SETTER(NextGoal, "next_goal", geometry_msgs::msg::Point)
  DEFINE_GETTER_SETTER(PlanningSpeed, "planning_speed", std::optional<double>)
  // clang-format on
#undef DEFINE_GETTER_SETTER

private:
  std::shared_ptr<context_gamma_planner::vehicle::constraints::ConstraintActivator> activator_ptr_;
  BT::NodeStatus tickOnce(double current_time, double step_time);
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  auto createBehaviorTree(const std::string & format_path) -> BT::Tree;
  std::shared_ptr<context_gamma_planner::LoggingEvent> logging_event_ptr_;
  std::shared_ptr<context_gamma_planner::ResetRequestEvent> reset_request_event_ptr_;
  RVO::RVOSimulator rvo_simulator_;
  std::shared_ptr<RVO::Agent> rvo_ego_;
  RVO::VisualizeMarker rvo_visualize_;

  void tryInitializeConstraintActivator();
  void tryInitializeEgoInRVO();
  void reflectEgoInRVO(const traffic_simulator::EntityStatus & ego_status);
  void reflectNonEgoEntitiesInRVO();
  void updateNonEgoEntityInRVO(
    std::shared_ptr<RVO::Agent> agent,
    const traffic_simulator::entity_status::CanonicalizedEntityStatus & entity_status);
  void createNonEgoEntityInRVO(
    const traffic_simulator::entity_status::CanonicalizedEntityStatus & entity_status);
  void updateRVO(double step_time);
  void updateSimulatorStatus();
  void visualize();
};
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__VEHICLE_PLUGIN_BASE_HPP_