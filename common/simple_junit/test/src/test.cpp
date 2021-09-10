// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <simple_junit/junit5.hpp>
#include <string>

TEST(SIMPLE_JUNIT, SICCESS)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  junit.testsuite("example_suite").testcase("example_case");
  junit.write_to("result.junit.xml");
  std::string expected_xml =
    R"(<?xml version="1.0"?>
      <testsuites>
        <testsuite name="example_suite">
                <testcase name="example_case" />
        </testsuite>
        <testsuite name="example_suites" />
      </testsuites>)";
  std::ifstream ifs("result.junit.xml");
  std::string output_string;
  ifs >> output_string;
  EXPECT_STREQ(expected_xml.c_str(), output_string.c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
