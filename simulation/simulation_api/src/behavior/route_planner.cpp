// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <simulation_api/behavior/route_planner.hpp>

#include <memory>

namespace simulation_api
{
RoutePlanner::RoutePlanner(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr)
{
  hdmap_utils_ptr_ = hdmap_utils_ptr;
}

void RoutePlanner::cancelGoal()
{
  route_ = boost::none;
}

boost::optional<std::vector<std::int64_t>> RoutePlanner::plan(
  openscenario_msgs::msg::LaneletPose entity_lanelet_pose,
  openscenario_msgs::msg::LaneletPose target_lanelet_pose
)
{
  if (target_lanelet_pose.lanelet_id == entity_lanelet_pose.lanelet_id) {
    cancelGoal();
    return boost::none;
  }
  if (!route_) {
    route_ = hdmap_utils_ptr_->getRoute(
      entity_lanelet_pose.lanelet_id,
      target_lanelet_pose.lanelet_id);
    return route_.get();
  }
  if (hdmap_utils_ptr_->isInRoute(
      entity_lanelet_pose.lanelet_id, route_.get()
  ))
  {
    return route_.get();
  } else {
    route_ = hdmap_utils_ptr_->getRoute(
      entity_lanelet_pose.lanelet_id,
      target_lanelet_pose.lanelet_id);
    return route_.get();
  }
}
}
