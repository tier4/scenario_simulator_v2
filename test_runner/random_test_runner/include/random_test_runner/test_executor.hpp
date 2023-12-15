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

#ifndef RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP
#define RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP

#include <memory>
#include <rclcpp/logger.hpp>

#include "random_test_runner/data_types.hpp"
#include "random_test_runner/file_interactions/junit_xml_reporter.hpp"
#include "random_test_runner/metrics/almost_standstill_metric.hpp"
#include "random_test_runner/metrics/ego_collision_metric.hpp"
#include "random_test_runner/metrics/goal_reached_metric.hpp"
#include "traffic_simulator/api/api.hpp"

class TestExecutor
{
public:
  TestExecutor(
    std::shared_ptr<traffic_simulator::API> api, TestDescription description,
    JunitXmlReporterTestCase test_case_reporter, SimulatorType simulator_type,
    ArchitectureType architecture_type, rclcpp::Logger logger);

  void initialize();
  void update();
  void deinitialize();
  bool scenarioCompleted();

private:
  void executeWithErrorHandling(std::function<void()> && func);

  std::shared_ptr<traffic_simulator::API> api_;
  TestDescription test_description_;
  const std::string ego_name_ = "ego";

  AlmostStandstillMetric almost_standstill_metric_;
  GoalReachedMetric goal_reached_metric_;
  EgoCollisionMetric ego_collision_metric_;
  JunitXmlReporterTestCase error_reporter_;

  SimulatorType simulator_type_;
  ArchitectureType architecture_type_;

  bool scenario_completed_ = false;

  rclcpp::Logger logger_;
};

#endif  // RANDOM_TEST_RUNNER__TEST_EXECUTOR_HPP
