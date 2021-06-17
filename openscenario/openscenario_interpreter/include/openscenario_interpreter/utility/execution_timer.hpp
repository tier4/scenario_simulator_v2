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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__TIME_STAT_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__TIME_STAT_HPP_

#include <chrono>
#include <cmath>
#include <functional>
#include <unordered_map>

#include "raii.hpp"

namespace openscenario_interpreter
{
inline namespace utility
{
template <typename ClockType = std::chrono::system_clock>
struct ExecutionTimer
{
  struct Statistics
  {
    template <typename Duration>
    void add(Duration diff)
    {
      std::int64_t diff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count();
      count_++;
      ns_max = std::max(ns_max, diff_ns);
      ns_min = std::min(ns_max, diff_ns);
      ns_sum += diff_ns;
      ns_sq_sum += diff_ns * diff_ns;
    }

    int count() const { return count_; }
    std::chrono::nanoseconds max() const { return std::chrono::nanoseconds(ns_max); }
    std::chrono::nanoseconds min() const { return std::chrono::nanoseconds(ns_min); }
    std::chrono::nanoseconds mean() const { return std::chrono::nanoseconds(ns_sum / count_); }
    std::chrono::nanoseconds stddev() const
    {
      std::int64_t mean_of_sq = ns_sq_sum / count_;
      std::int64_t sq_of_mean = std::pow(ns_sum / count_, 2);
      std::int64_t var = mean_of_sq - sq_of_mean;
      double stddev_ = std::sqrt(var);
      return std::chrono::nanoseconds(static_cast<std::int64_t>(stddev_));
    }

  private:
    std::int64_t ns_max = 0;
    std::int64_t ns_min = std::numeric_limits<std::int64_t>::max();
    std::int64_t ns_sum = 0;
    std::int64_t ns_sq_sum = 0;
    int count_ = 0;
  };

  template <typename F, typename... Args>
  auto invoke(const std::string & tag, F && func, Args &&... args) -> std::enable_if_t<
    std::is_same<typename std::result_of<F(Args...)>::type, void>::value,
    typename ClockType::duration>
  {
    auto start = ClockType::now();
    std::forward<F>(func)(std::forward<Args>(args)...);  // use std::invoke (c++17)
    auto end = ClockType::now();
    statistics_map[tag].add(end - start);
    return end - start;
  }

  template <typename F, typename... Args>
  auto invoke(const std::string & tag, F && func, Args &&... args) -> std::enable_if_t<
    std::is_same<typename std::result_of<F(Args...)>::type, bool>::value,
    typename ClockType::duration>
  {
    auto start = ClockType::now();
    bool flag = std::forward<F>(func)(std::forward<Args>(args)...);  // use std::invoke (c++17)
    auto end = ClockType::now();
    if (flag) {
      statistics_map[tag].add(end - start);
    }
    return end - start;
  }

  void clear() { statistics_map.clear(); }

  const Statistics & get_statistics(const std::string & tag) { return statistics_map[tag]; }
  auto begin() const { return statistics_map.begin(); }
  auto end() const { return statistics_map.end(); }

private:
  std::unordered_map<std::string, Statistics> statistics_map;
};
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__TIME_STAT_HPP_
