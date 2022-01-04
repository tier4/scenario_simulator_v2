// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__ERROR_REPORTER_HPP
#define RANDOM_TEST_RUNNER__ERROR_REPORTER_HPP

#include <spdlog/fmt/fmt.h>

#include <boost/filesystem.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "random_test_runner/data_types.hpp"
#include "simple_junit/junit5.hpp"

class JunitXmlReporterTestCase
{
public:
  explicit JunitXmlReporterTestCase(common::SimpleTestCase & testcase) : testcase_(testcase) {}

  void reportCollision(const NPCDescription & npc, double time)
  {
    reportError("collision", fmt::format("{} and ego collided at {}s", npc.name, time).c_str());
  }

  void reportStandStill() { reportError("stand still", "Ego seems to be stuck"); }

  void reportTimeout() { reportError("timeout", "Ego failed to reach goal within timeout"); }

private:
  void reportError(const std::string & error_type, const std::string & message)
  {
    const common::junit::Error result(error_type, message);
    testcase_.error.push_back(result);
  }

  common::SimpleTestCase & testcase_;
};

class JunitXmlReporter
{
public:
  explicit JunitXmlReporter(rclcpp::Logger logger) : logger_(logger) {}

  void init(const std::string & output_directory) { output_directory_ = output_directory; }

  JunitXmlReporterTestCase spawnTestCase(
    const std::string & testsuite_name, const std::string & testcase_name)
  {
    return JunitXmlReporterTestCase(results_.testsuite(testsuite_name).testcase(testcase_name));
  }

  void write()
  {
    std::string message = fmt::format("Saving results to {}", output_directory_);
    RCLCPP_INFO_STREAM(logger_, message);
    results_.write_to(
      (boost::filesystem::path(output_directory_) / "result.junit.xml").c_str(), "  ");
  }

private:
  common::JUnit5 results_;
  std::string output_directory_;
  rclcpp::Logger logger_;
};

#endif  // RANDOM_TEST_RUNNER__ERROR_REPORTER_HPP
