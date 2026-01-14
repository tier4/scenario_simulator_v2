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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <simple_junit/test_suites.hpp>
#include <string>

namespace junit
{
TestSuites::TestSuites()
: timestamp_(boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time()))
{
}

void TestSuites::write(const std::filesystem::path & destination)
{
  if (std::filesystem::exists(destination)) {
    std::filesystem::remove(destination);
  }

  pugi::xml_document document;

  pugi::xml_node node = document.append_child("testsuites");

  node.append_attribute("timestamp") = timestamp_.c_str();

  document.child("testsuites").append_attribute("time") = getTotalTime();
  document.child("testsuites").append_attribute("tests") = test_suites_.size();

  for (const auto & each : test_suites_) {
    const auto & suite_name = std::get<0>(each);
    const auto & test_cases = std::get<1>(each);

    auto testsuite_node = document.child("testsuites").append_child("testsuite");
    testsuite_node.append_attribute("name") = suite_name.c_str();
    testsuite_node.append_attribute("tests") = test_cases.size();

    for (const auto & test_case : test_cases) {
      auto testcase_node = testsuite_node.append_child("testcase");
      testcase_node.append_attribute("classname") = test_case.classname.c_str();
      testcase_node.append_attribute("name") = test_case.name.c_str();
      testcase_node.append_attribute("time") = test_case.time;

      switch (test_case.result) {
        case TestResult::FAILURE:
          testcase_node.append_child("failure");
          testcase_node.child("failure").append_attribute("message") = "test failed";
          testcase_node.child("failure")
            .append_child(pugi::node_pcdata)
            .set_value(test_case.description.c_str());
          break;

        case TestResult::ERROR:
          testcase_node.append_child("error");
          testcase_node.child("error").append_attribute("message") = "error on test";
          testcase_node.child("error")
            .append_child(pugi::node_pcdata)
            .set_value(test_case.description.c_str());
          break;

        default:
          break;
      }
    }
  }

  document.save_file(destination.c_str());
}
}  // namespace junit
