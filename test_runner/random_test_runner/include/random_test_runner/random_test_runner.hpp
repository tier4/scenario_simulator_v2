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

#ifndef RANDOM_TEST_RUNNER__RANDOM_TEST_RUNNER_HPP
#define RANDOM_TEST_RUNNER__RANDOM_TEST_RUNNER_HPP

#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "random_test_runner/data_types.hpp"
#include "random_test_runner/file_interactions/junit_xml_reporter.hpp"
#include "random_test_runner/test_executor.hpp"

namespace traffic_simulator
{
class API;
}
class LaneletUtils;

class RandomTestRunner : public rclcpp::Node
{
public:
  explicit RandomTestRunner(const rclcpp::NodeOptions & option);

private:
  TestControlParameters collectAndValidateTestControlParameters();
  TestSuiteParameters collectTestSuiteParameters();
  TestCaseParameters collectTestCaseParameters();

  static TestSuiteParameters validateParameters(
    const TestSuiteParameters & test_parameters, std::shared_ptr<LaneletUtils> hdmap_utils);

  void update();
  void start();
  void stop();

  std::random_device seed_randomization_device_;

  std::vector<TestExecutor<traffic_simulator::API>> test_executors_;
  std::vector<TestExecutor<traffic_simulator::API>>::iterator current_test_executor_;

  JunitXmlReporter error_reporter_;

  std::shared_ptr<traffic_simulator::API> api_;

  rclcpp::TimerBase::SharedPtr update_timer_;
};
#endif  // RANDOM_TEST_RUNNER__RANDOM_TEST_RUNNER_HPP
