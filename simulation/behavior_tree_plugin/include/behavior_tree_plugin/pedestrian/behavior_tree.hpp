// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <memory>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <vector>

namespace entity_behavior
{
class PedestrianBehaviorTree : public BehaviorPluginBase
{
public:
  void configure() override;
  void update(double current_time, double step_time) override;
#define DEFINE_GETTER_SETTER(GETTER, SETTER, TYPE)                                    \
  TYPE GETTER() override { return tree_.rootBlackboard()->get<TYPE>(GETTER##Key()); } \
  void SETTER(const TYPE & value) override                                            \
  {                                                                                   \
    tree_.rootBlackboard()->set<TYPE>(GETTER##Key(), value);                          \
  }
  DEFINE_GETTER_SETTER(getWaypoints, setWaypoints, openscenario_msgs::msg::WaypointsArray)
  DEFINE_GETTER_SETTER(getObstacle, setObstacle, boost::optional<openscenario_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(getUpdatedStatus, setUpdatedStatus, openscenario_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(getRequest, setRequest, std::string)
  DEFINE_GETTER_SETTER(getHdMapUtils, setHdMapUtils, std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(getEntityTypeList, setEntityTypeList, EntityTypeDict)
  DEFINE_GETTER_SETTER(
    getTrafficLightManager, setTrafficLightManager,
    std::shared_ptr<traffic_simulator::TrafficLightManager>)
  DEFINE_GETTER_SETTER(
    getPedestrianParameters, setPedestrianParameters, openscenario_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(getDriverModel, setDriverModel, openscenario_msgs::msg::DriverModel)
  DEFINE_GETTER_SETTER(
    getVehicleParameters, setVehicleParameters, openscenario_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(getOtherEntityStatus, setOtherEntityStatus, EntityStatusDict)
  DEFINE_GETTER_SETTER(getToLaneletId, setToLaneletId, std::int64_t)
  DEFINE_GETTER_SETTER(getEntityStatus, setEntityStatus, openscenario_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(getTargetSpeed, setTargetSpeed, boost::optional<double>)
  DEFINE_GETTER_SETTER(getRouteLanelets, setRouteLanelets, std::vector<std::int64_t>)
  DEFINE_GETTER_SETTER(getCurrentTime, setCurrentTime, double)
  DEFINE_GETTER_SETTER(getStepTime, setStepTime, double)
#undef DEFINE_GETTER_SETTER

private:
  BT::NodeStatus tickOnce(double current_time, double step_time);
  std::shared_ptr<BT::StdCoutLogger> logger_cout_ptr_;
  void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
    BT::NodeStatus status);
  void setupLogger();
  BT::TimestampType type_;
  BT::TimePoint first_timestamp_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
  std::string current_action_;
  std::string request_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__BEHAVIOR_TREE_HPP_
