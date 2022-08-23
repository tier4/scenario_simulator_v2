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

#ifndef TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_

#include <boost/optional.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class VehicleEntity : public EntityBase
{
public:
  struct BuiltinBehavior
  {
    static auto behaviorTree() noexcept -> const std::string &
    {
      static const std::string name = "behavior_tree_plugin/VehicleBehaviorTree";
      return name;
    }

    static auto contextGamma() noexcept -> const std::string &
    {
      static const std::string name = "TODO";
      return name;
    }

    static auto defaultBehavior() noexcept -> const std::string & { return behaviorTree(); }
  };

  explicit VehicleEntity(
    const std::string & name,                                //
    const traffic_simulator_msgs::msg::VehicleParameters &,  //
    const std::string & = BuiltinBehavior::defaultBehavior());

  ~VehicleEntity() override = default;

  const traffic_simulator_msgs::msg::VehicleParameters parameters;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "VehicleEntity";
    return result;
  }

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose);

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestLaneChange(const std::int64_t to_lanelet_id) override;

  void requestLaneChange(const traffic_simulator::lane_change::Parameter & parameter) override;

  void cancelRequest() override;

  const boost::optional<traffic_simulator_msgs::msg::VehicleParameters> getVehicleParameters() const
  {
    return parameters;
  }

  void setDriverModel(const traffic_simulator_msgs::msg::DriverModel & model) override
  {
    behavior_plugin_ptr_->setDriverModel(model);
  }

  void setAccelerationLimit(double acceleration) override;

  void setDecelerationLimit(double deceleration) override;

  auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel override;

  void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr) override
  {
    EntityBase::setHdMapUtils(ptr);
    route_planner_ptr_ = std::make_shared<traffic_simulator::RoutePlanner>(ptr);
    behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
  }

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> & ptr) override
  {
    EntityBase::setTrafficLightManager(ptr);
    behavior_plugin_ptr_->setTrafficLightManager(traffic_light_manager_);
  }

  const traffic_simulator_msgs::msg::BoundingBox getBoundingBox() const override
  {
    return parameters.bounding_box;
  }

  void requestAssignRoute(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  const std::string getCurrentAction() const { return behavior_plugin_ptr_->getCurrentAction(); }

  const traffic_simulator_msgs::msg::WaypointsArray getWaypoints() override
  {
    try {
      return behavior_plugin_ptr_->getWaypoints();
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

  std::vector<traffic_simulator_msgs::msg::LaneletPose> getGoalPoses() override
  {
    return route_planner_ptr_->getGoalPoses();
  }

  boost::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return behavior_plugin_ptr_->getObstacle();
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
  const std::string plugin_name;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;
  std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;
  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;

  std::vector<std::int64_t> previous_route_lanelets_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
