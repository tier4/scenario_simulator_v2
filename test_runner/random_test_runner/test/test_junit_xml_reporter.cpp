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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include <gtest/gtest.h>

#include <random_test_runner/file_interactions/junit_xml_reporter.hpp>

#include "test_utils.hpp"

std::string readFile(const std::string & path = "/tmp/result.junit.xml")
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

TEST(JunitXmlReporter, reportCollision)
{
  JunitXmlReporter reporter(rclcpp::get_logger("test_junit_xml_reporter"));
  reporter.init("/tmp");
  JunitXmlReporterTestCase testcase = reporter.spawnTestCase("testsuite", "testcase");

  testcase.reportCollision(makeNPCDescription("npc", 1.0, makeLaneletPose(123, 5.0)), 10.0);

  reporter.write();

  std::string report = readFile();
  std::string ans = R"(<?xml version="1.0"?>
<testsuites failures="0" errors="1" tests="1">
  <testsuite name="testsuite" failures="0" errors="1" tests="1">
    <testcase name="testcase">
      <error type="collision" message="npc and ego collided at 10s" />
    </testcase>
  </testsuite>
</testsuites>
)";
  EXPECT_STREQ(report.c_str(), ans.c_str());
}

TEST(JunitXmlReporter, reportStandStill)
{
  JunitXmlReporter reporter(rclcpp::get_logger("test_junit_xml_reporter"));
  reporter.init("/tmp");
  JunitXmlReporterTestCase testcase = reporter.spawnTestCase("testsuite", "testcase");

  testcase.reportStandStill();

  reporter.write();

  std::string report = readFile();
  std::string ans = R"(<?xml version="1.0"?>
<testsuites failures="0" errors="1" tests="1">
  <testsuite name="testsuite" failures="0" errors="1" tests="1">
    <testcase name="testcase">
      <error type="stand still" message="Ego seems to be stuck" />
    </testcase>
  </testsuite>
</testsuites>
)";
  EXPECT_STREQ(report.c_str(), ans.c_str());
}

TEST(JunitXmlReporter, reportTimeout)
{
  JunitXmlReporter reporter(rclcpp::get_logger("test_junit_xml_reporter"));
  reporter.init("/tmp");
  JunitXmlReporterTestCase testcase = reporter.spawnTestCase("testsuite", "testcase");

  testcase.reportTimeout();

  reporter.write();

  std::string report = readFile();
  std::string ans = R"(<?xml version="1.0"?>
<testsuites failures="0" errors="1" tests="1">
  <testsuite name="testsuite" failures="0" errors="1" tests="1">
    <testcase name="testcase">
      <error type="timeout" message="Ego failed to reach goal within timeout" />
    </testcase>
  </testsuite>
</testsuites>
)";
  EXPECT_STREQ(report.c_str(), ans.c_str());
}

TEST(JunitXmlReporter, reportException)
{
  JunitXmlReporter reporter(rclcpp::get_logger("test_junit_xml_reporter"));
  reporter.init("/tmp");
  JunitXmlReporterTestCase testcase = reporter.spawnTestCase("testsuite", "testcase");

  testcase.reportException("exception_type", "Exception message");

  reporter.write();

  std::string report = readFile();
  std::string ans = R"(<?xml version="1.0"?>
<testsuites failures="0" errors="1" tests="1">
  <testsuite name="testsuite" failures="0" errors="1" tests="1">
    <testcase name="testcase">
      <error type="exception_type" message="Exception message" />
    </testcase>
  </testsuite>
</testsuites>
)";
  EXPECT_STREQ(report.c_str(), ans.c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
