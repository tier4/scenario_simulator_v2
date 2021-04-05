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

#ifndef SIMULATION_API__ENTITY__PEDESTRIAN_ENTITY_HPP_
#define SIMULATION_API__ENTITY__PEDESTRIAN_ENTITY_HPP_

#include <simulation_api/entity/entity_base.hpp>
#include <simulation_api/entity/pedestrian_parameter.hpp>
#include <simulation_api/behavior/pedestrian/behavior_tree.hpp>
#include <simulation_api/behavior/route_planner.hpp>

#include <openscenario_msgs/msg/pedestrian_parameters.hpp>

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
class PedestrianEntity : public EntityBase
{
public:
  PedestrianEntity(
    std::string name, const openscenario_msgs::msg::EntityStatus & initial_state,
    openscenario_msgs::msg::PedestrianParameters parameters);
  PedestrianEntity(std::string name, openscenario_msgs::msg::PedestrianParameters parameters);
  const openscenario_msgs::msg::PedestrianParameters parameters;
  void onUpdate(double current_time, double step_time) override;
  void requestAcquirePosition(openscenario_msgs::msg::LaneletPose lanelet_pose);
  void requestWalkStraight();
  // void requestLaneChange(std::int64_t to_lanelet_id);
  void cancelRequest();
  void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr)
  {
    hdmap_utils_ptr_ = ptr;
    route_planner_ptr_ = std::make_shared<simulation_api::RoutePlanner>(ptr);
    tree_ptr_->setValueToBlackBoard("hdmap_utils", hdmap_utils_ptr_);
  }
  void setTrafficLightManager(std::shared_ptr<simulation_api::TrafficLightManager> ptr)
  {
    traffic_light_manager_ = ptr;
    tree_ptr_->setValueToBlackBoard("traffic_light_manager", traffic_light_manager_);
  }
  void setTargetSpeed(double target_speed, bool continuous);
  const openscenario_msgs::msg::BoundingBox getBoundingBox() const override
  {
    return parameters.bounding_box;
  }
  void requestAssignRoute(const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints)
  override;
  const std::string getCurrentAction() const
  {
    return tree_ptr_->getCurrentAction();
  }
  std::vector<std::int64_t> getRouteLanelets(double horizon = 100)
  {
    if (!status_) {
      return {};
    }
    if (status_->lanelet_pose_valid) {
      return {};
    }
    return route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, horizon);
  }

private:
  std::shared_ptr<entity_behavior::pedestrian::BehaviorTree> tree_ptr_;
  BT::NodeStatus action_status_;
  boost::optional<double> target_speed_;
  std::shared_ptr<simulation_api::RoutePlanner> route_planner_ptr_;
};
}  // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__PEDESTRIAN_ENTITY_HPP_
