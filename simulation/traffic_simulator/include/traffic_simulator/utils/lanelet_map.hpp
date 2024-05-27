// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <traffic_simulator/utils/lanelet/lanelet_map.hpp>
#include <traffic_simulator/utils/lanelet/other.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
template <typename... Ts>
auto activate(Ts &&... xs)
{
  return lanelet2::LaneletMap::activate(std::forward<decltype(xs)>(xs)...);
}

inline auto yaw(const lanelet::Id & lanelet_id, const geometry_msgs::msg::Point & point)
  -> std::tuple<double, geometry_msgs::msg::Point, geometry_msgs::msg::Point>
{
  /// @note Copied from motion_util::findNearestSegmentIndex
  const auto centerline_points = traffic_simulator::lanelet2::other::getCenterPoints(lanelet_id);
  auto find_nearest_segment_index = [](
                                      const std::vector<geometry_msgs::msg::Point> & points,
                                      const geometry_msgs::msg::Point & point) {
    assert(not points.empty());
    double min_distance = std::numeric_limits<double>::max();
    size_t min_index = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto distance = [](const auto point1, const auto point2) {
        const auto dx = point1.x - point2.x;
        const auto dy = point1.y - point2.y;
        return dx * dx + dy * dy;
      }(points.at(i), point);
      if (distance < min_distance) {
        min_distance = distance;
        min_index = i;
      }
    }
    return min_index;
  };
  const size_t segment_index = find_nearest_segment_index(centerline_points, point);
  const auto & previous_point = centerline_points.at(segment_index);
  const auto & next_point = centerline_points.at(segment_index + 1);
  return std::make_tuple(
    std::atan2(next_point.y - previous_point.y, next_point.x - previous_point.x), previous_point, next_point);
}
}  // namespace lanelet_map
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
