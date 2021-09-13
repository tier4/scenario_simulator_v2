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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <simple_junit/junit5.hpp>
#include <string>

std::string readFromFile(const std::string & path)
{
  std::ifstream ifs(path);
  std::string string = "";
  std::string line_string;
  while (getline(ifs, line_string)) {
    string = string + line_string;
  }
  return string;
}

std::string trimCharacter(const std::string & string, const std::string & character)
{
  std::string result = string;
  size_t c;
  while ((c = result.find_first_of(character)) != std::string::npos) {
    result.erase(c, 1);
  }
  return result;
}

std::string trim(const std::string & string)
{
  std::string result;
  result = trimCharacter(string, " ");
  result = trimCharacter(result, "\t");
  result = trimCharacter(result, "\n");
  return result;
}

#define EXPECT_TEXT_FILE_EQ(FILE0, FILE1)             \
  const std::string str0 = trim(readFromFile(FILE0)); \
  const std::string str1 = trim(readFromFile(FILE1)); \
  EXPECT_STREQ(str0.c_str(), str1.c_str());

TEST(SIMPLE_JUNIT, PASS)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  junit.testsuite("example_suite").testcase("example_case");
  junit.write_to("result.junit.xml", " ");
  EXPECT_TEXT_FILE_EQ(
    "result.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/pass.junit.xml");
}

TEST(SIMPLE_JUNIT, FAILURE)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Failure failure_case("example_failure", "failure_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
  junit.write_to("result.junit.xml", " ");
  EXPECT_TEXT_FILE_EQ(
    "result.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/failure.junit.xml");
}

TEST(SIMPLE_JUNIT, ERROR)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Error error_case("example_error", "error_test_case");
  junit.testsuite("example_suite").testcase("example_case").error.push_back(error_case);
  junit.write_to("result.junit.xml", " ");
  EXPECT_TEXT_FILE_EQ(
    "result.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/error.junit.xml");
}

TEST(SIMPLE_JUNIT, COMPLEX)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Error error_case("example_error", "error_test_case");
  junit.testsuite("example_suite").testcase("example_case").error.push_back(error_case);
  common::junit::Failure failure_case("example_failure", "failure_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
  junit.write_to("result.junit.xml", " ");
  EXPECT_TEXT_FILE_EQ(
    "result.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/complex.junit.xml");
}

TEST(SIMPLE_JUNIT, ASSERTION)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Failure failure_case("example_failure", "failure_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
