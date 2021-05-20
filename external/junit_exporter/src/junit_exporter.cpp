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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <junit_exporter/junit_exporter.hpp>
#include <string>

namespace junit_exporter
{
JunitExporter::JunitExporter()
: timestamp_(boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::universal_time()))
{
}

void JunitExporter::write(const std::string & path)
{
  const boost::filesystem::path junit_path(path);
  boost::system::error_code error;
  const bool result = boost::filesystem::exists(junit_path, error);
  if (result) {
    boost::filesystem::remove(junit_path);
  }
  pugi::xml_document doc;
  pugi::xml_node node = doc.append_child("testsuites");

  node.append_attribute("timestamp") = timestamp_.c_str();
  doc.child("testsuites").append_attribute("time") = test_suites_.getTime();
  doc.child("testsuites").append_attribute("tests") = test_suites_.getTestSuites().size();
  for (const auto & test_suite : test_suites_.getTestSuites()) {
    pugi::xml_node testsuite_node = doc.child("testsuites").append_child("testsuite");
    testsuite_node.append_attribute("name") = test_suite.c_str();
    testsuite_node.append_attribute("tests") = test_suites_.getTestSuite(test_suite).size();
    for (const auto & test_case : test_suites_.getTestSuite(test_suite)) {
      auto testcase_node = testsuite_node.append_child("testcase");
      testcase_node.append_attribute("classname") = test_case.classname.c_str();
      testcase_node.append_attribute("name") = test_case.name.c_str();
      testcase_node.append_attribute("time") = test_case.time;
      switch (test_case.result) {
        case TestResult::SUCCESS: {
          break;
        }
        case TestResult::FAILURE: {
          testcase_node.append_child("failure");
          testcase_node.child("failure").append_attribute("message") = "failure detected";
          testcase_node.child("failure")
            .append_child(pugi::node_pcdata)
            .set_value(test_case.description.c_str());
          break;
        }
        case TestResult::ERROR: {
          testcase_node.append_child("error");
          testcase_node.child("error").append_attribute("message") = "error detected";
          testcase_node.child("error")
            .append_child(pugi::node_pcdata)
            .set_value(test_case.description.c_str());
          break;
        }
      }
    }
  }
  doc.save_file(path.c_str());
}

void JunitExporter::addTestCase(
  const std::string & case_name,   //
  const std::string & suite_name,  //
  const double time,               //
  const TestResult & result)
{
  if (not test_suites_.existTestCase(case_name, suite_name)) {
    auto test_case = TestCase(case_name, suite_name, suite_name, time, result);
    test_suites_.emplaceTestCase(test_case);
  }
}

void JunitExporter::addTestCase(
  const std::string & name,        //
  const std::string & test_suite,  //
  const double & time,             //
  const TestResult & result,       //
  const std::string & type,        //
  const std::string & description)
{
  if (not test_suites_.existTestCase(name, test_suite)) {
    auto test_case = TestCase(name, test_suite, test_suite, time, result, type, description);
    test_suites_.emplaceTestCase(test_case);
  }
}
}  // namespace junit_exporter
