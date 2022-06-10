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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_

#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <scenario_simulator_exception/exception.hpp>
#include <geometry_math/catmull_rom_spline.hpp>
#include <unordered_map>
#include <vector>

namespace hdmap_utils
{
class RouteCache
{
public:
  bool exists(std::int64_t from, std::int64_t to)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::pair<std::int64_t, std::int64_t> key;
    key.first = from;
    key.second = to;
    if (data_.find(key) == data_.end()) {
      return false;
    }
    return true;
  }
  std::vector<std::int64_t> getRoute(std::int64_t from, std::int64_t to)
  {
    if (!exists(from, to)) {
      THROW_SIMULATION_ERROR(
        "route from : ", from, " to : ", to, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto ret = data_.at({from, to});
    return ret;
  }
  void appendData(std::int64_t from, std::int64_t to, const std::vector<std::int64_t> & route)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[{from, to}] = route;
  }

private:
  std::unordered_map<std::pair<std::int64_t, std::int64_t>, std::vector<std::int64_t>> data_;
  std::mutex mutex_;
};

class CenterPointsCache
{
public:
  bool exists(std::int64_t lanelet_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.find(lanelet_id) == data_.end()) {
      return false;
    }
    return true;
  }
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::int64_t lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.at(lanelet_id);
  }
  std::shared_ptr<geometry_math::CatmullRomSpline> getCenterPointsSpline(
    std::int64_t lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return splines_[lanelet_id];
  }
  void appendData(std::int64_t lanelet_id, const std::vector<geometry_msgs::msg::Point> & route)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = route;
    splines_[lanelet_id] = std::make_shared<geometry_math::CatmullRomSpline>(route);
  }

private:
  std::unordered_map<std::int64_t, std::vector<geometry_msgs::msg::Point>> data_;
  std::unordered_map<std::int64_t, std::shared_ptr<geometry_math::CatmullRomSpline>>
    splines_;
  std::mutex mutex_;
};

class LaneletLengthCache
{
public:
  bool exists(std::int64_t lanelet_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data_.find(lanelet_id) == data_.end()) {
      return false;
    }
    return true;
  }
  double getLength(std::int64_t lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("length of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return data_[lanelet_id];
  }
  void appendData(std::int64_t lanelet_id, double length)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_[lanelet_id] = length;
  }

private:
  std::unordered_map<std::int64_t, double> data_;
  std::mutex mutex_;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__CACHE_HPP_
