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

#include <traffic_simulator/data_type/behavior.hpp>

namespace traffic_simulator
{
namespace behavior
{
std::ostream & operator<<(std::ostream & stream, const Request & value)
{
  switch (value) {
    default:
    case Request::none:
      return stream << "none";
    case Request::lane_change:
      return stream << "lane_change";
    case Request::follow_lane:
      return stream << "follow_lane";
    case Request::follow_polyline_trajectory:
      return stream << "follow_polyline_trajectory";
    case Request::follow_clothoid_trajectory:
      return stream << "follow_clothoid_trajectory";
    case Request::follow_nurbs_trajectory:
      return stream << "follow_nurbs_trajectory";
    case Request::walk_straight:
      return stream << "walk_straight";
  }
}

std::string getRequestString(const Request & request)
{
  switch (request) {
    case Request::none:
      return "none";
    case Request::lane_change:
      return "lane_change";
    case Request::follow_lane:
      return "follow_lane";
    case Request::follow_polyline_trajectory:
      return "follow_polyline_trajectory";
    case Request::follow_clothoid_trajectory:
      return "follow_clothoid_trajectory";
    case Request::follow_nurbs_trajectory:
      return "follow_nurbs_trajectory";
    case Request::walk_straight:
      return "walk_straight";
    default:
      THROW_SEMANTIC_ERROR(request, " is invalid");
  }
}
}  // namespace behavior
}  // namespace traffic_simulator
