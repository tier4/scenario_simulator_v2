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

#ifndef BEHAVIOR_TREE_PLUGIN__ACTION_NODE_HPP_
#define BEHAVIOR_TREE_PLUGIN__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <boost/algorithm/clamp.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <memory>
#include <string>
#include <traffic_simulator/data_type/data_types.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <unordered_map>
#include <vector>

namespace entity_behavior
{
class ActionNode : public BT::ActionNodeBase
{
public:
  ActionNode(const std::string & name, const BT::NodeConfiguration & config);
  ~ActionNode() override = default;
  bool foundConflictingEntity(const std::vector<std::int64_t> & following_lanelets) const;
  std::optional<double> getDistanceToConflictingEntity(
    const std::vector<std::int64_t> & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline);
  std::optional<std::string> getFrontEntityName(
    const math::geometry::CatmullRomSplineInterface & spline);
  double calculateStopDistance(double deceleration) const;
  std::optional<double> getDistanceToFrontEntity(
    const math::geometry::CatmullRomSplineInterface & spline);
  std::optional<double> getDistanceToStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints);
  std::optional<double> getDistanceToTrafficLightStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline);
  std::vector<traffic_simulator_msgs::msg::EntityStatus> getRightOfWayEntities();
  std::vector<traffic_simulator_msgs::msg::EntityStatus> getRightOfWayEntities(
    const std::vector<std::int64_t> & following_lanelets);
  std::optional<double> getYieldStopDistance(
    const std::vector<std::int64_t> & following_lanelets);
  std::vector<traffic_simulator_msgs::msg::EntityStatus> getOtherEntityStatus(
    std::int64_t lanelet_id);
  traffic_simulator_msgs::msg::EntityStatus stopAtEndOfRoad();
  double getHorizon() const;

  /// throws if the derived class return RUNNING.
  BT::NodeStatus executeTick() override;

  void halt() override final { setStatus(BT::NodeStatus::IDLE); }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<traffic_simulator::behavior::Request>("request"),
      BT::InputPort<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils"),
      BT::InputPort<traffic_simulator_msgs::msg::EntityStatus>("entity_status"),
      BT::InputPort<double>("current_time"),
      BT::InputPort<double>("step_time"),
      BT::InputPort<std::optional<double>>("target_speed"),
      BT::OutputPort<traffic_simulator_msgs::msg::EntityStatus>("updated_status"),
      BT::OutputPort<traffic_simulator::behavior::Request>("request"),
      BT::InputPort<std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus>>(
        "other_entity_status"),
      BT::InputPort<std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>>(
        "entity_type_list"),
      BT::InputPort<std::vector<std::int64_t>>("route_lanelets"),
      BT::InputPort<std::shared_ptr<traffic_simulator::TrafficLightManagerBase>>(
        "traffic_light_manager"),
      BT::OutputPort<std::optional<traffic_simulator_msgs::msg::Obstacle>>("obstacle"),
      BT::OutputPort<traffic_simulator_msgs::msg::WaypointsArray>("waypoints")};
  }
  void getBlackBoardValues();
  traffic_simulator::behavior::Request request;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  std::shared_ptr<traffic_simulator::TrafficLightManagerBase> traffic_light_manager;
  traffic_simulator_msgs::msg::EntityStatus entity_status;
  double current_time;
  double step_time;
  std::optional<double> target_speed;
  traffic_simulator_msgs::msg::EntityStatus updated_status;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> other_entity_status;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> entity_type_list;
  std::vector<std::int64_t> route_lanelets;
  traffic_simulator_msgs::msg::EntityStatus getEntityStatus(const std::string target_name) const;
  std::optional<double> getDistanceToTargetEntityPolygon(
    const math::geometry::CatmullRomSplineInterface & spline, const std::string target_name,
    double width_extension_right = 0.0, double width_extension_left = 0.0,
    double length_extension_front = 0.0, double length_extension_rear = 0.0);

private:
  std::optional<double> getDistanceToTargetEntityOnCrosswalk(
    const math::geometry::CatmullRomSplineInterface & spline,
    const traffic_simulator_msgs::msg::EntityStatus & status);
  std::optional<double> getDistanceToTargetEntityPolygon(
    const math::geometry::CatmullRomSplineInterface & spline,
    const traffic_simulator_msgs::msg::EntityStatus & status, double width_extension_right = 0.0,
    double width_extension_left = 0.0, double length_extension_front = 0.0,
    double length_extension_rear = 0.0);
  std::optional<traffic_simulator_msgs::msg::EntityStatus> getConflictingEntityStatus(
    const std::vector<std::int64_t> & following_lanelets) const;
  std::vector<traffic_simulator_msgs::msg::EntityStatus> getConflictingEntityStatusOnCrossWalk(
    const std::vector<std::int64_t> & route_lanelets) const;
  std::vector<traffic_simulator_msgs::msg::EntityStatus> getConflictingEntityStatusOnLane(
    const std::vector<std::int64_t> & route_lanelets) const;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__ACTION_NODE_HPP_
