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

#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/quaternion/slerp.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <reactive_agent_plugin/plugin.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>

namespace entity_behavior
{

void ReactiveAgentBehavior::configure(const rclcpp::Logger &) {}

void ReactiveAgentBehavior::update(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  const auto poses = getGoalPoses();
  std::cout << poses.size() << std::endl;
  // if(poly){
  //   std::cout << poly->shape.vertices.size() << std::endl;
  // }else{
  //   std::cout <<"NULL" << std::endl;
  // }
  // for(const auto & vertex: poly->shape.vertices){
  //   std::cout << vertex.position.position.x << "," << vertex.position.position.y << std::endl;
  // }
}
auto ReactiveAgentBehavior::getCurrentAction() -> const std::string &
{
  static const std::string behavior = "reactive_agent";
  return behavior;
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::ReactiveAgentBehavior, entity_behavior::BehaviorPluginBase)
