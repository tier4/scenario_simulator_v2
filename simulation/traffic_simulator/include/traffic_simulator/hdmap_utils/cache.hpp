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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_

#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <unordered_map>
#include <vector>

namespace hdmap_utils
{
class RouteCache
{
public:
  bool exists(std::int64_t from, std::int64_t to) const
  {
    std::pair<std::int64_t, std::int64_t> key;
    key.first = from;
    key.second = to;
    if (data_.find(key) == data_.end()) {
      return false;
    }
    return true;
  }
  std::vector<std::int64_t> getRoute(std::int64_t from, std::int64_t to) const
  {
    if (!exists(from, to)) {
      THROW_SIMULATION_ERROR(
        "route from : ", from, " to : ", to, " does not exists on route chache.");
    }
    return data_.at({from, to});
  }
  void appendData(std::int64_t from, std::int64_t to, const std::vector<std::int64_t> & route)
  {
    data_[{from, to}] = route;
  }

private:
  std::unordered_map<std::pair<std::int64_t, std::int64_t>, std::vector<std::int64_t> > data_;
};

class CenterPointsCache
{
public:
  bool exists(std::int64_t lanelet_id) const
  {
    if (data_.find(lanelet_id) == data_.end()) {
      return false;
    }
    return true;
  }
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::int64_t lanelet_id) const
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR(
        "center point from : ", lanelet_id, " does not exists on route chache.");
    }
    return data_.at(lanelet_id);
  }
  void appendData(std::int64_t lanelet_id, const std::vector<geometry_msgs::msg::Point> & route)
  {
    data_[lanelet_id] = route;
  }

private:
  std::unordered_map<std::int64_t, std::vector<geometry_msgs::msg::Point> > data_;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_