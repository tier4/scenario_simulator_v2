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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_

#include <chrono>
#include <cmath>
#include <functional>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace utility
{
template <typename Clock = std::chrono::system_clock>
class ExecutionTimer
{
  class Statistics
  {
    std::int64_t ns_max = 0;

    std::int64_t ns_min = std::numeric_limits<std::int64_t>::max();

    std::int64_t ns_sum = 0;

    std::int64_t ns_sq_sum = 0;

    int count_ = 0;

  public:
    template <typename Duration>
    auto add(Duration diff) -> void
    {
      std::int64_t diff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count();
      count_++;
      ns_max = std::max(ns_max, diff_ns);
      ns_min = std::min(ns_max, diff_ns);
      ns_sum += diff_ns;
      ns_sq_sum += diff_ns * diff_ns;
    }

    auto count() const { return count_; }

    auto max() const { return std::chrono::nanoseconds(ns_max); }

    auto min() const { return std::chrono::nanoseconds(ns_min); }

    auto mean() const { return std::chrono::nanoseconds(ns_sum / count_); }

    auto standardDeviation() const
    {
      std::int64_t mean_of_sq = ns_sq_sum / count_;
      std::int64_t sq_of_mean = std::pow(ns_sum / count_, 2);
      std::int64_t var = mean_of_sq - sq_of_mean;
      double standard_deviation = std::sqrt(var);
      return std::chrono::nanoseconds(static_cast<std::int64_t>(standard_deviation));
    }
  };

  std::unordered_map<std::string, Statistics> statistics_map;

public:
  template <typename Thunk, typename... Ts>
  auto invoke(const std::string & tag, Thunk && thunk) -> typename std::enable_if<
    std::is_same<typename std::result_of<Thunk()>::type, void>::value,
    typename Clock::duration>::type
  {
    const auto begin = Clock::now();

    thunk();

    const auto end = Clock::now();

    statistics_map[tag].add(end - begin);

    return end - begin;
  }

  template <typename Thunk, typename... Ts>
  auto invoke(const std::string & tag, Thunk && thunk) -> typename std::enable_if<
    std::is_same<typename std::result_of<Thunk()>::type, bool>::value,
    typename Clock::duration>::type
  {
    const auto begin = Clock::now();

    const auto result = thunk();

    const auto end = Clock::now();

    if (result) {
      statistics_map[tag].add(end - begin);
    }

    return end - begin;
  }

  auto clear() { statistics_map.clear(); }

  auto getStatistics(const std::string & tag) -> const auto & { return statistics_map[tag]; }

  auto begin() const { return statistics_map.begin(); }

  auto end() const { return statistics_map.end(); }
};
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_
