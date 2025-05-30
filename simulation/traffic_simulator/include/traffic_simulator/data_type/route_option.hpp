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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__ROUTE_OPTION_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__ROUTE_OPTION_HPP_

namespace traffic_simulator
{
inline namespace route_option
{
inline namespace v2
{
struct RouteOption
{
  bool allow_goal_modification = false;
  bool use_lane_ids_for_routing = false;
};
}  // namespace v2

namespace v1
{
struct RouteOption
{
  bool allow_goal_modification = false;

  // conversion without loss of information
  operator v2::RouteOption() const
  {
    v2::RouteOption v2;
    v2.allow_goal_modification = allow_goal_modification;
    v2.use_lane_ids_for_routing = false;
    return v2;
  }
};
}  // namespace v1
}  // namespace route_option
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ROUTE_OPTION_HPP_
