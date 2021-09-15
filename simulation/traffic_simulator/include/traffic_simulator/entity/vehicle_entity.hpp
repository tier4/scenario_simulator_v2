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

#ifndef TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_

#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/behavior/target_speed_planner.hpp>
#include <traffic_simulator/behavior/vehicle/behavior_tree.hpp>
#include <traffic_simulator/behavior/vehicle/lane_change_action.hpp>
#include <traffic_simulator/entity/entity_base.hpp>

// headers in pugixml
#include <boost/optional.hpp>
#include <memory>
#include <pugixml.hpp>
#include <string>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class VehicleEntity : public EntityBase
{
public:
  VehicleEntity(
    const std::string & name, const openscenario_msgs::msg::VehicleParameters & parameters);

  ~VehicleEntity() override = default;

  const openscenario_msgs::msg::VehicleParameters parameters;

  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "VehicleEntity";
    return result;
  }

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose);

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestLaneChange(const std::int64_t to_lanelet_id);

  void cancelRequest() override;

  const boost::optional<openscenario_msgs::msg::VehicleParameters> getVehicleParameters() const
  {
    return parameters;
  }

  void setDriverModel(const openscenario_msgs::msg::DriverModel & model) override
  {
    tree_ptr_->setValueToBlackBoard("driver_model", model);
  }

  void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr) override
  {
    EntityBase::setHdMapUtils(ptr);
    route_planner_ptr_ = std::make_shared<traffic_simulator::RoutePlanner>(ptr);
    tree_ptr_->setValueToBlackBoard("hdmap_utils", hdmap_utils_ptr_);
  }

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr) override
  {
    EntityBase::setTrafficLightManager(ptr);
    tree_ptr_->setValueToBlackBoard("traffic_light_manager", traffic_light_manager_);
  }

  void setTargetSpeed(double target_speed, bool continuous) override;

  const openscenario_msgs::msg::BoundingBox getBoundingBox() const override
  {
    return parameters.bounding_box;
  }

  void requestAssignRoute(
    const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  const std::string getCurrentAction() const { return tree_ptr_->getCurrentAction(); }

  const openscenario_msgs::msg::WaypointsArray getWaypoints() override
  {
    try {
      return tree_ptr_->getWaypoints();
    } catch (const std::runtime_error & e) {
      if (!status_) {
        THROW_SIMULATION_ERROR("Entity : ", name, " status is empty.");
      }
      if (status_ && status_->lanelet_pose_valid == false) {
        THROW_SIMULATION_ERROR(
          "Failed to calculate waypoints in NPC logics, please check Entity : ", name,
          " is in a lane coordinate.");
      }
      THROW_SIMULATION_ERROR("Failed to calculate waypoint in NPC logics.");
    }
  }

  std::vector<openscenario_msgs::msg::LaneletPose> getGoalPoses() override
  {
    return route_planner_ptr_->getGoalPoses();
  }

  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle() override
  {
    return tree_ptr_->getObstacle();
  }

  std::vector<std::int64_t> getRouteLanelets(double horizon = 100) override
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
  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;
  traffic_simulator::behavior::TargetSpeedPlanner target_speed_planner_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
