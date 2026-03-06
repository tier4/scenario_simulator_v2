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

#ifndef JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
#define JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_

#include <filesystem>
#include <numeric>
#include <pugixml.hpp>
#include <simple_junit/test_suite.hpp>
#include <string>

namespace junit
{
class TestSuites
{
  const std::string timestamp_;

  std::unordered_map<std::string, TestSuite> test_suites_;

public:
  explicit TestSuites();

  void write(const std::filesystem::path & path);

  auto contains(const std::string & suite_name, const std::string & case_name) const
  {
    const auto iter = test_suites_.find(suite_name);

    if (iter != std::end(test_suites_)) {
      return std::any_of(
        std::begin(iter->second), std::end(iter->second),
        [&](const auto & each_case) { return each_case.name == case_name; });
    } else {
      return false;
    }
  }

  auto getTotalTime() const
  {
    return std::accumulate(
      std::cbegin(test_suites_), std::cend(test_suites_), 0,
      [](const auto sum, const auto & each_suite) {
        const auto & each_cases = std::get<1>(each_suite);

        return sum + std::accumulate(
                       std::cbegin(each_cases), std::cend(each_cases), 0.0,
                       [](const auto sum, const auto & each_case) { return sum + each_case.time; });
      });
  }

  template <typename... Ts>
  void addTestCase(
    const std::string & suite_name,  //
    const std::string & case_name,   //
    Ts &&... xs)
  {
    if (not contains(suite_name, case_name)) {
      test_suites_[suite_name].emplace_back(
        case_name, suite_name, std::forward<decltype(xs)>(xs)...);
    }
  }
};
}  // namespace junit

#endif  // JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
