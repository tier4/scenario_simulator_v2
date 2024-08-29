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

#ifndef BEHAVIOR_TREE_PLUGIN__VEHICLE__BEHAVIOR_TREE_HPP_
#define BEHAVIOR_TREE_PLUGIN__VEHICLE__BEHAVIOR_TREE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <behavior_tree_plugin/transition_events/transition_events.hpp>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace entity_behavior
{
class VehicleBehaviorTree : public BehaviorPluginBase
{
public:
  auto update(const double current_time, const double step_time) -> void override;
  void configure(const rclcpp::Logger & logger) override;
  const std::string & getCurrentAction() const override;

  auto getBehaviorParameter() -> traffic_simulator_msgs::msg::BehaviorParameter override;

  auto setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &)
    -> void override;

#define DEFINE_GETTER_SETTER(NAME, TYPE)                                                    \
  TYPE get##NAME() override { return tree_.rootBlackboard()->get<TYPE>(get##NAME##Key()); } \
  void set##NAME(const TYPE & value) override                                               \
  {                                                                                         \
    tree_.rootBlackboard()->set<TYPE>(get##NAME##Key(), value);                             \
  }

  // clang-format off
  DEFINE_GETTER_SETTER(CanonicalizedEntityStatus,                        std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>)
  DEFINE_GETTER_SETTER(CurrentTime,                                      double)
  DEFINE_GETTER_SETTER(DebugMarker,                                      std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DefaultMatchingDistanceForLaneletPoseCalculation, double)
  DEFINE_GETTER_SETTER(GoalPoses,                                        std::vector<geometry_msgs::msg::Pose>)
  DEFINE_GETTER_SETTER(HdMapUtils,                                       std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(LaneChangeParameters,                             traffic_simulator::lane_change::Parameter)
  DEFINE_GETTER_SETTER(Obstacle,                                         std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus,                                EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters,                             traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(PolylineTrajectory,                               std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>)
  DEFINE_GETTER_SETTER(ReferenceTrajectory,                              std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_GETTER_SETTER(Request,                                          traffic_simulator::behavior::Request)
  DEFINE_GETTER_SETTER(RouteLanelets,                                    lanelet::Ids)
  DEFINE_GETTER_SETTER(StepTime,                                         double)
  DEFINE_GETTER_SETTER(TargetSpeed,                                      std::optional<double>)
  DEFINE_GETTER_SETTER(TrafficLightManager,                              std::shared_ptr<traffic_simulator::TrafficLightManager>)
  DEFINE_GETTER_SETTER(VehicleParameters,                                traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints,                                        traffic_simulator_msgs::msg::WaypointsArray)
  // clang-format on
#undef DEFINE_GETTER_SETTER

private:
  auto tickOnce(const double current_time, const double step_time) -> BT::NodeStatus;
  auto createBehaviorTree(const std::string & format_path) -> BT::Tree;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<behavior_tree_plugin::LoggingEvent> logging_event_ptr_;
  std::unique_ptr<behavior_tree_plugin::ResetRequestEvent> reset_request_event_ptr_;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__VEHICLE__BEHAVIOR_TREE_HPP_
