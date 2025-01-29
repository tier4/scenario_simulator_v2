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

#ifndef CONCEALER__PATH_WITH_LANE_ID_HPP_
#define CONCEALER__PATH_WITH_LANE_ID_HPP_

#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>)
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#endif

#if __has_include(<tier4_planning_msgs/msg/path_with_lane_id.hpp>)
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#endif

#include <concealer/convert.hpp>

namespace concealer
{
namespace priority
{
using PathWithLaneId = decltype(std::tuple_cat(
  std::declval<std::tuple<
#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>)
    autoware_internal_planning_msgs::msg::PathWithLaneId
#endif
    > >(),
  std::declval<std::tuple<
#if __has_include(<tier4_planning_msgs/msg/path_with_lane_id.hpp>)
    tier4_planning_msgs::msg::PathWithLaneId
#endif
    > >()));

static_assert(0 < std::tuple_size_v<PathWithLaneId>);
}  // namespace priority

#if __has_include(<autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>) and \
    __has_include(<tier4_planning_msgs/msg/path_with_lane_id.hpp>)
template <>
auto convert(const tier4_planning_msgs::msg::PathWithLaneId & from)
  -> autoware_internal_planning_msgs::msg::PathWithLaneId;
#endif
}  // namespace concealer

#endif  // CONCEALER__PATH_WITH_LANE_ID_HPP_
