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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__BEHAVIOR_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__BEHAVIOR_HPP_

#include <iostream>
#include <scenario_simulator_exception/exception.hpp>

namespace traffic_simulator
{
namespace behavior
{
enum class Request {
  NONE,
  LANE_CHANGE,
  FOLLOW_LANE,
  FOLLOW_POLYLINE_TRAJECTORY,
  FOLLOW_CLOTHOID_TRAJECTORY,
  FOLLOW_NURBS_TRAJECTORY,
  WALK_STRAIGHT,
};

std::string getRequestString(const Request &);

std::ostream & operator<<(std::ostream &, const Request &);
}  // namespace behavior
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__BEHAVIOR_HPP_
