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

#include <openscenario_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator/behavior/pedestrian/behavior_tree.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/pedestrian_parameter.hpp>

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
class PedestrianEntity : public EntityBase
{
public:
  PedestrianEntity(
    const std::string & name, const openscenario_msgs::msg::EntityStatus & initial_state,
    const openscenario_msgs::msg::PedestrianParameters & parameters);

  PedestrianEntity(
    const std::string & name, const openscenario_msgs::msg::PedestrianParameters & parameters);

  const openscenario_msgs::msg::PedestrianParameters parameters;

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose) override;

  void requestWalkStraight() override;

  void cancelRequest();

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

  const std::string getCurrentAction() const override { return tree_ptr_->getCurrentAction(); }

  std::vector<std::int64_t> getRouteLanelets(double horizon = 100) override
  {
    if (status_ and status_->lanelet_pose_valid) {
      return route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, horizon);
    } else {
      return {};
    }
  }

  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle() override { return boost::none; }

  const openscenario_msgs::msg::WaypointsArray getWaypoints() override
  {
    return openscenario_msgs::msg::WaypointsArray();
  };

private:
  std::shared_ptr<entity_behavior::pedestrian::BehaviorTree> tree_ptr_;
  BT::NodeStatus action_status_;
  boost::optional<double> target_speed_;
  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
