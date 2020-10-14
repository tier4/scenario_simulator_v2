// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <junit_exporter/test_suites.hpp>

#include <string>
#include <vector>
#include <algorithm>

namespace junit_exporter
{
TestSuites::TestSuites() {}

void TestSuites::addTestCase(const TestCase & test_case)
{
  test_cases_.emplace_back(test_case);
}

double TestSuites::getTime() const
{
  double ret = 0;
  for (const auto & test_case : test_cases_) {
    ret = ret + test_case.time;
  }
  return ret;
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

bool TestSuites::testCaseExists(
  const std::string & name,
  const std::string & test_suite
)
{
  for (const auto & test_case : test_cases_) {
    if (test_case.test_suite == test_suite && test_case.name == name) {
      return true;
    }
  }
  return false;
}
}  // namespace junit_exporter
