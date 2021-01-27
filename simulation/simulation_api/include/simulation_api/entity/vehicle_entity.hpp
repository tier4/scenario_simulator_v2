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

#ifndef SIMULATION_API__ENTITY__VEHICLE_ENTITY_HPP_
#define SIMULATION_API__ENTITY__VEHICLE_ENTITY_HPP_

#include <simulation_api/entity/entity_base.hpp>
#include <simulation_api/entity/vehicle_parameter.hpp>
#include <simulation_api/behavior/vehicle/lane_change_action.hpp>
#include <simulation_api/behavior/vehicle/behavior_tree.hpp>
#include <simulation_api/behavior/route_planner.hpp>

#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>

#include <rclcpp/rclcpp.hpp>

// headers in pugixml
#include <pugixml.hpp>

#include <boost/optional.hpp>
#include <memory>
#include <vector>
#include <string>

namespace simulation_api
{
namespace entity
{
class VehicleEntity : public EntityBase
{
public:
  VehicleEntity(
    std::string name, const openscenario_msgs::msg::EntityStatus & initial_state,
    const pugi::xml_node & xml);
  VehicleEntity(
    std::string name, const openscenario_msgs::msg::EntityStatus & initial_state,
    VehicleParameters parameters);
  VehicleEntity(std::string name, const pugi::xml_node & xml);
  VehicleEntity(std::string name, VehicleParameters parameters);
  const VehicleParameters parameters;
  void onUpdate(double current_time, double step_time) override;
  void requestAcquirePosition(openscenario_msgs::msg::LaneletPose lanelet_pose);
  void requestLaneChange(std::int64_t to_lanelet_id);
  void cancelRequest();
  void setDriverModel(const openscenario_msgs::msg::DriverModel & model)
  {
    tree_ptr_->setValueToBlackBoard("driver_model", model);
  }
  void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr)
  {
    hdmap_utils_ptr_ = ptr;
    route_planner_ptr_ = std::make_shared<simulation_api::RoutePlanner>(ptr);
    tree_ptr_->setValueToBlackBoard("hdmap_utils", hdmap_utils_ptr_);
  }
  void setTargetSpeed(double target_speed, bool continuous);
  const openscenario_msgs::msg::BoundingBox getBoundingBox() const override
  {
    return parameters.bounding_box.toRosMsg();
  }
  const std::string getCurrentAction() const
  {
    return tree_ptr_->getCurrentAction();
  }
  openscenario_msgs::msg::WaypointsArray getWaypoints()
  {
    return tree_ptr_->getWaypoints();
  }
  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle()
  {
    return tree_ptr_->getObstacle();
  }
  std::vector<std::int64_t> getRouteLanelets(double horizon = 100)
  {
    if (!status_) {
      return {};
    }
    if (!status_->lanelet_pose_valid) {
      return {};
    }
    return route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, horizon);
  }

private:
  std::shared_ptr<entity_behavior::vehicle::BehaviorTree> tree_ptr_;
  BT::NodeStatus action_status_;
  std::int64_t to_lanelet_id_;
  boost::optional<double> target_speed_;
  std::shared_ptr<simulation_api::RoutePlanner> route_planner_ptr_;
};
}  // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__VEHICLE_ENTITY_HPP_
