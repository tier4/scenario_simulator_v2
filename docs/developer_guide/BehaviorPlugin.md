# Behavior Plugin

Behavior plugin enables support multiple type of NPC behaviors.
Default behavior plugin implementation is [here](https://github.com/tier4/scenario_simulator_v2/tree/master/simulation/behavior_tree_plugin).

Behavior plugin use pluginlib and plugin class should be inherited [this base class](https://tier4.github.io/scenario_simulator_v2-api-docs/classentity__behavior_1_1BehaviorPluginBase.html).

Example of the behavior plugin class is below.

```C++
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

#ifndef BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__BEHAVIOR_TREE_HPP_
#define BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__BEHAVIOR_TREE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <behavior_tree_plugin/pedestrian/follow_lane_action.hpp>
#include <behavior_tree_plugin/pedestrian/walk_straight_action.hpp>
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
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace entity_behavior
{
class PedestrianBehaviorTree : public BehaviorPluginBase
{
public:
  void configure(const rclcpp::Logger & logger) override;
  void update(double current_time, double step_time) override;
  const std::string & getCurrentAction() const override;
#define DEFINE_GETTER_SETTER(NAME, TYPE)                                                    \
  TYPE get##NAME() override { return tree_.rootBlackboard()->get<TYPE>(get##NAME##Key()); } \
  void set##NAME(const TYPE & value) override                                               \
  {                                                                                         \
    tree_.rootBlackboard()->set<TYPE>(get##NAME##Key(), value);                             \
  }

  // clang-format off
  DEFINE_GETTER_SETTER(CurrentTime, double)
  DEFINE_GETTER_SETTER(DebugMarker, std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DriverModel, traffic_simulator_msgs::msg::DriverModel)
  DEFINE_GETTER_SETTER(EntityStatus, traffic_simulator_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(EntityTypeList, EntityTypeDict)
  DEFINE_GETTER_SETTER(HdMapUtils, std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(Obstacle, std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus, EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters, traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(Request, std::string)
  DEFINE_GETTER_SETTER(RouteLanelets, std::vector<std::int64_t>)
  DEFINE_GETTER_SETTER(ReferenceTrajectory, std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_GETTER_SETTER(StepTime, double)
  DEFINE_GETTER_SETTER(TargetSpeed, std::optional<double>)
  DEFINE_GETTER_SETTER(ToLaneletId, std::int64_t)
  DEFINE_GETTER_SETTER(TrafficLightManager,std::shared_ptr<traffic_simulator::TrafficLightManager>)
  DEFINE_GETTER_SETTER(UpdatedStatus, traffic_simulator_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(VehicleParameters, traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints, traffic_simulator_msgs::msg::WaypointsArray)
  // clang-format on

#undef DEFINE_GETTER_SETTER

private:
  BT::NodeStatus tickOnce(double current_time, double step_time);
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<behavior_tree_plugin::LoggingEvent> logging_event_ptr_;
  std::shared_ptr<behavior_tree_plugin::ResetRequestEvent> reset_request_event_ptr_;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__BEHAVIOR_TREE_HPP_
```

If you want to develop behavior plugin, first, you have to add three functions below.

```C++
void configure(const rclcpp::Logger & logger) override;
```
configure plugin class, this function only calls once at the start of simulation.
```C++
void update(double current_time, double step_time) override;
```
update plugin class, this function calls once in every frame in simulation.
```C++
const std::string & getCurrentAction() const override;
```
return current actions of NPC, this result only use for visualization.

After that, you have to define getter and setters for each data.  
You can get unique key for each data by calling `BehaviorPluginBase::get(FOO)Key` function, such as `BehaviorPluginBase::getCurrentTimeKey()`.  
Getter are named as get(FOO), such as `BehaviorPluginClass::getDriverModel();`.  
Setters are named as set(Foo), such as `BehaviorPluginClass::setDriverModel(traffic_simulator_msgs::msg::DriverModel);`.

| Name                 | Description                                           | Type                                                            |
|----------------------|-------------------------------------------------------|-----------------------------------------------------------------|
| DebugMarker          | Rviz marker for debugging NPC behavior                | `std::vector<visualization_msgs::msg::Marker>`                  |
| DriverModel          | Driver behavior parameters                            | `traffic_simulator_msgs::msg::DriverModel`                      |
| EntityStatus         | Entity status of the NPC you want to control          | `std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>` |
| EntityTypeList       | Dictionary of NPC name and it's type.                 | `EntityTypeDict`                                                |
| GoalPoses            | Goal poses of entity.                                 | `std::vector<geometry_msgs::msg::Pose>`                         |
| HdMapUtils           | Shared pointer of HdMapUtils class.                   | `std::shared_ptr<hdmap_utils::HdMapUtils>`                      |
| Obstacle             | Target obstacle of your NPC.                          | `std::optional<traffic_simulator_msgs::msg::Obstacle>`          |
| OtherEntityStatus    | Dictionary of other entity status.                    | `EntityStatusDict`                                              |
| PedestrianParameters | Entity parameters for pedestrian.                     | `traffic_simulator_msgs::msg::PedestrianParameters`             |
| Request              | Request to the NPC you want to control                | `std::string`                                                   |
| RouteLanelets        | Lanelet ids on entity route                           | `std::vector<std::int64_t>`                                     |
| ReferenceTrajectory  | Trajectory precalculated for RouteLanelets.           | `std::shared_ptr<math::geometry::CatmullRomSpline>`             |
| StepTime             | Step time of the simulation.                          | `double`                                                        |
| TargetSpeed          | Target speed of the NPC you want to control           | `std::optional<double>`                                         |
| ToLaneletId          | Goal lanelet ID                                       | `std::int64_t`                                                  |
| TrafficLightManager  | Shared pointer to the traffic light manager           | `std::shared_ptr<traffic_simulator::TrafficLightManager>`       |
| UpdatedStatus        | Updated entity status of the NPC you want to control. | `std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>` |
| VehicleParameters    | Parameters of vehicle entity.                         | `traffic_simulator_msgs::msg::VehicleParameters`                |
| Waypoints            | Waypoints of the NPC you want to control.             | `traffic_simulator_msgs::msg::WaypointsArray`                   |
