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

#include <concealer/get_parameter.hpp>
#include <concealer/path_with_lane_id.hpp>

namespace concealer
{
#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>)
template <>
auto available<autoware_internal_planning_msgs::msg::PathWithLaneId>() -> bool
{
  if (const auto architecture_type =
        getParameter<std::string>("architecture_type", default_architecture_type);
      architecture_type.find("awf/universe") != std::string::npos) {
    return "awf/universe/20250130" <= architecture_type;
  } else {
    return false;
  }
}
#endif

#if __has_include(<tier4_planning_msgs/msg/path_with_lane_id.hpp>)
template <>
auto available<tier4_planning_msgs::msg::PathWithLaneId>() -> bool
{
  if (const auto architecture_type =
        getParameter<std::string>("architecture_type", default_architecture_type);
      architecture_type.find("awf/universe") != std::string::npos) {
    return architecture_type < "awf/universe/20250130";
  } else {
    return false;
  }
}
#endif

#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>) and \
    __has_include(<tier4_planning_msgs/msg/path_with_lane_id.hpp>)
template <>
auto convert(const tier4_planning_msgs::msg::PathWithLaneId & from)
  -> autoware_internal_planning_msgs::msg::PathWithLaneId
{
  auto to = autoware_internal_planning_msgs::msg::PathWithLaneId();
  to.header = from.header;
  for (const auto & from_point : from.points) {
    auto to_point = decltype(to.points)::value_type();
    to_point.point = from_point.point;
    to_point.lane_ids = from_point.lane_ids;
    to.points.push_back(to_point);
  }
  to.left_bound = from.left_bound;
  to.right_bound = from.right_bound;
  return to;
}
#endif
}  // namespace concealer
