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

#ifndef TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_

#include <boost/optional.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <pugixml.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/behavior/target_speed_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class PedestrianEntity : public EntityBase
{
public:
  struct BuiltinBehavior
  {
    static auto behaviorTree() noexcept -> const std::string &
    {
      static const std::string name = "behavior_tree_plugin/PedestrianBehaviorTree";
      return name;
    }

    static auto contextGamma() noexcept -> const std::string &
    {
      static const std::string name = "context_gamma_planner/PedestrianPlugin";
      return name;
    }

    static auto defaultBehavior() noexcept -> const std::string & { return behaviorTree(); }
  };

  explicit PedestrianEntity(
    const std::string & name,                                   //
    const traffic_simulator_msgs::msg::PedestrianParameters &,  //
    const std::string & = BuiltinBehavior::defaultBehavior());

  const traffic_simulator_msgs::msg::PedestrianParameters parameters;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "PedestrianEntity";
    return result;
  }

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestWalkStraight() override;

  void cancelRequest() override;

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

  void setTargetSpeed(double target_speed, bool continuous) override;

  const traffic_simulator_msgs::msg::BoundingBox getBoundingBox() const override
  {
    return parameters.bounding_box;
  }

  auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel;

  void setDriverModel(const traffic_simulator_msgs::msg::DriverModel &);

  void requestAssignRoute(
    const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  void requestSpeedChange(
    const double target_speed, const SpeedChangeTransition transition,
    const SpeedChangeConstraint constraint, const bool continuous) override;

  const std::string getCurrentAction() const override
  {
    return behavior_plugin_ptr_->getCurrentAction();
  }

  std::vector<std::int64_t> getRouteLanelets(double horizon = 100) override
  {
    if (status_ and status_->lanelet_pose_valid) {
      return route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, horizon);
    } else {
      return {};
    }
  }

  boost::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return boost::none;
  }

  std::vector<traffic_simulator_msgs::msg::LaneletPose> getGoalPoses() override
  {
    return route_planner_ptr_->getGoalPoses();
  }

  const traffic_simulator_msgs::msg::WaypointsArray getWaypoints() override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  };

  const std::string plugin_name;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;
  std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;
  traffic_simulator::behavior::TargetSpeedPlanner target_speed_planner_;
  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
