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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
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

void cleanup(const std::string & filename)
{
  const boost::filesystem::path path(filename);
  boost::system::error_code error;
  const bool result = boost::filesystem::exists(path, error);
  if (!result || error) {
    throw std::runtime_error("file : " + filename + " does not found.");
  }
  boost::filesystem::remove(path);
}

#define EXPECT_TEXT_FILE_EQ(FILE0, FILE1)             \
  const std::string str0 = trim(readFromFile(FILE0)); \
  const std::string str1 = trim(readFromFile(FILE1)); \
  EXPECT_STREQ(str0.c_str(), str1.c_str());

TEST(SIMPLE_JUNIT, PASS)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  junit.testsuite("example_suite").testcase("example_case").pass.push_back(common::junit::Pass());
  junit.write_to("result_pass.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_pass.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/pass.junit.xml");
  cleanup("result_pass.junit.xml");
}

TEST(SIMPLE_JUNIT, FAILURE)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Failure failure_case("example_failure", "failure_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
  junit.write_to("result_failure.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_failure.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/failure.junit.xml");
  cleanup("result_failure.junit.xml");
}

TEST(SIMPLE_JUNIT, ERROR)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Error error_case("example_error", "error_test_case");
  junit.testsuite("example_suite").testcase("example_case").error.push_back(error_case);
  junit.write_to("result_error.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_error.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/error.junit.xml");
  cleanup("result_error.junit.xml");
}

TEST(SIMPLE_JUNIT, COMPLEX)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Error error_case("example_error", "error_test_case");
  junit.testsuite("example_suite").testcase("example_case").error.push_back(error_case);
  common::junit::Failure failure_case("example_failure", "failure_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
  junit.write_to("result_complex.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_complex.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") + "/expected/complex.junit.xml");
  cleanup("result_complex.junit.xml");
}

TEST(SIMPLE_JUNIT, ATTRIBUTES)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  common::junit::Failure failure_case("example_attributes", "attributes_test_case");
  junit.testsuite("example_suite").testcase("example_case").failure.push_back(failure_case);
  junit.testsuite("example_suite").testcase("example_case").assertions = "example_assertion";
  junit.testsuite("example_suite").testcase("example_case").time = "10";
  junit.testsuite("example_suite").testcase("example_case").classname = "example_class";
  junit.testsuite("example_suite").testcase("example_case").status = "failure";
  junit.write_to("result_attribute.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_attribute.junit.xml", ament_index_cpp::get_package_share_directory("simple_junit") +
                                    "/expected/attributes.junit.xml");
  cleanup("result_attribute.junit.xml");
}

TEST(SIMPLE_JUNIT, TESTSUITES_NAME)
{
  common::junit::JUnit5 junit;
  junit.testsuite("example_suites");
  junit.name = "example_name";
  junit.write_to("result_testsuites_name.junit.xml", "  ");
  EXPECT_TEXT_FILE_EQ(
    "result_testsuites_name.junit.xml",
    ament_index_cpp::get_package_share_directory("simple_junit") +
      "/expected/testsuites_name.junit.xml");
  cleanup("result_testsuites_name.junit.xml");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
