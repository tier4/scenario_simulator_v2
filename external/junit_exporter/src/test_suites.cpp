// Copyright 2015-2019 Tier IV, Inc. All rights reserved.
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

#include <algorithm>
#include <junit_exporter/test_suites.hpp>
#include <numeric>
#include <string>
#include <vector>

namespace junit_exporter
{
double TestSuites::getTime() const
{
  return std::accumulate(
    std::cbegin(test_cases_), std::cend(test_cases_), 0,
    [](const auto & lhs, const auto & each) { return lhs + each.time; });
}

std::vector<std::string> TestSuites::getTestSuites() const
{
  std::vector<std::string> ret;
  for (const auto & test_case : test_cases_) {
    if (std::count(ret.begin(), ret.end(), test_case.test_suite) == 0) {
      ret.emplace_back(test_case.test_suite);
    }
  }
  return ret;
}

std::vector<TestCase> TestSuites::getTestSuite(const std::string & test_suite)
{
  std::vector<TestCase> ret;
  for (const auto & test_case : test_cases_) {
    if (test_case.test_suite == test_suite) {
      ret.emplace_back(test_case);
    }
  }
  return ret;
}

bool TestSuites::existTestCase(const std::string & name, const std::string & test_suite)
{
  return std::any_of(std::cbegin(test_cases_), std::cend(test_cases_), [&](const auto & each) {
    return each.test_suite == test_suite and each.name == name;
  });
}
}  // namespace junit_exporter
