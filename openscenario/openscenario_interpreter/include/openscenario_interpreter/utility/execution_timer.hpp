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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <nlohmann/json.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace utility
{
template <typename Clock = std::chrono::system_clock>
class ExecutionTimer
{
  using Statistics = boost::accumulators::accumulator_set<
    std::int64_t,
    boost::accumulators::stats<
      boost::accumulators::tag::min, boost::accumulators::tag::max, boost::accumulators::tag::mean,
      boost::accumulators::tag::variance, boost::accumulators::tag::count>>;

  std::unordered_map<std::string, Statistics> statistics_map;

public:
  template <typename Thunk, typename... Ts>
  auto invoke(const std::string & tag, Thunk && thunk)
  {
    const auto begin = Clock::now();

    thunk();

    const auto end = Clock::now();

    statistics_map[tag](std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());

    return end - begin;
  }

  void saveStatistics(boost::filesystem::path output_file)
  {
    nlohmann::json json_data;
    for (const auto & [name, statistics] : statistics_map) {
      json_data[name + "/min"] = boost::accumulators::extract::min(statistics) * 1e-9;
      json_data[name + "/max"] = boost::accumulators::extract::max(statistics) * 1e-9;
      json_data[name + "/mean"] = boost::accumulators::extract::mean(statistics) * 1e-9;
      json_data[name + "/stddev"] =
        std::sqrt(boost::accumulators::extract::variance(statistics)) * 1e-9;
      json_data[name + "/count"] = boost::accumulators::extract::count(statistics);
    }

    std::ofstream file(output_file);
    file << json_data.dump(4);
    file.close();
  }

  auto clear() { statistics_map.clear(); }

  auto getStatistics(const std::string & tag) -> const auto & { return statistics_map[tag]; }

  auto begin() const { return statistics_map.begin(); }

  auto end() const { return statistics_map.end(); }
};
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__EXECUTION_TIMER_HPP_
