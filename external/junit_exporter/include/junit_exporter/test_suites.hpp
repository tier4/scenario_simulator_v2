// Copyright 2015-2019 TierIV.inc. All rights reserved.
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

#ifndef JUNIT_EXPORTER__TEST_SUITES_HPP_
#define JUNIT_EXPORTER__TEST_SUITES_HPP_

#include <vector>
#include <string>

namespace junit_exporter
{
enum class TestResult
{
  SUCCESS,
  FAILURE,
  ERROR,
};

struct TestCase
{
  TestCase(
    const std::string & name,
    const std::string & test_suite,
    const std::string & classname,
    const double & time,
    const TestResult & result)
  : name(name),
    test_suite(test_suite),
    classname(classname),
    time(time),
    result(result),
    type("type"),
    description("") {}
  TestCase(
    const std::string & name,
    const std::string & test_suite,
    const std::string & classname,
    const double & time,
    const TestResult & result,
    const std::string & type,
    const std::string & description)
  : name(name),
    test_suite(test_suite),
    classname(classname),
    time(time),
    result(result),
    type(type),
    description(description) {}
  const std::string name;
  const std::string test_suite;
  const std::string classname;
  const double time;
  const TestResult result;
  const std::string type;
  const std::string description;
};

class TestSuites
{
public:
  TestSuites();
  void addTestCase(const TestCase & test_case);
  double getTime() const;
  std::vector<std::string> getTestSuites() const;
  std::vector<TestCase> getTestSuite(const std::string & test_suite);
  bool testCaseExists(
    const std::string & name,
    const std::string & test_suite
  );

private:
  std::vector<TestCase> test_cases_;
};
}  // namespace junit_exporter

#endif  // JUNIT_EXPORTER__TEST_SUITES_HPP_
