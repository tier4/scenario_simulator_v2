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

#ifndef JUNIT_EXPORTER__TEST_CASE_HPP_
#define JUNIT_EXPORTER__TEST_CASE_HPP_

#include <simple_junit/test_result.hpp>
#include <string>
#include <unordered_map>

namespace junit
{
struct TestCase
{
  explicit TestCase(
    const std::string & name,              //
    const std::string & classname,         //
    const double & time,                   //
    const TestResult & result,             //
    const std::string & type = "untyped",  //
    const std::string & description = "")
  : name(name),
    classname(classname),
    time(time),
    result(result),
    type(type),
    description(description)
  {
  }

  const std::string name;
  const std::string test_suite;
  const std::string classname;
  const double time;
  const TestResult result;
  const std::string type;
  const std::string description;
};
}  // namespace junit

#endif  // JUNIT_EXPORTER__TEST_CASE_HPP_
