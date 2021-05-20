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

#ifndef JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
#define JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_

#include <boost/filesystem/path.hpp>
#include <junit_exporter/test_suites.hpp>
#include <pugixml.hpp>
#include <string>
#include <vector>

namespace junit_exporter
{
class JunitExporter
{
  const std::string timestamp_;

  TestSuites test_suites_;

public:
  JunitExporter();

  void write(const boost::filesystem::path & path);

  template <typename... Ts>
  void addTestCase(
    const std::string & suite_name,  //
    const std::string & case_name,   //
    Ts &&... xs)
  {
    if (not test_suites_.existTestCase(case_name, suite_name)) {
      test_suites_.emplaceTestCase(
        case_name, suite_name, suite_name, std::forward<decltype(xs)>(xs)...);
    }
  }
};
}  // namespace junit_exporter

#endif  // JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
