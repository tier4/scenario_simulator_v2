// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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
#include <simulation_api/color_utils/color_utils.hpp>

#include <simulation_api/behavior/pedestrian/behavior_tree.hpp>

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
    std::string name, const EntityStatus & initial_state,
    const pugi::xml_node & xml);
  PedestrianEntity(
    std::string name, const EntityStatus & initial_state,
    PedestrianParameters parameters);
  PedestrianEntity(std::string name, const pugi::xml_node & xml);
  PedestrianEntity(std::string name, PedestrianParameters parameters);
  visualization_msgs::msg::MarkerArray generateMarker(
    rclcpp::Time stamp,
    std_msgs::msg::ColorRGBA color = color_utils::makeColorMsg("forestgreen", 0.8)) const;
  const PedestrianParameters parameters;
  void onUpdate(double current_time, double step_time) override;
  void requestAcquirePosition(int lanelet_id, double s, double offset);
  // void requestLaneChange(int to_lanelet_id);
  void cancelRequest();
  void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr)
  {
    hdmap_utils_ptr_ = ptr;
    tree_ptr_->setValueToBlackBoard("hdmap_utils", hdmap_utils_ptr_);
  }
  void setTargetSpeed(double target_speed, bool continuous);

private:
  std::shared_ptr<entity_behavior::pedestrian::BehaviorTree> tree_ptr_;
  BT::NodeStatus action_status_;
  // entity_behavior::pedestrian::LaneChangeParameter lane_change_params_;
  boost::optional<double> target_speed_;
};
}  // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__PEDESTRIAN_ENTITY_HPP_
