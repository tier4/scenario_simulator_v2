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

#include <spdlog/fmt/fmt.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/optional/optional_io.hpp>
#include <memory>
#include <random_test_runner/file_interactions/yaml_test_params_saver.hpp>
#include <random_test_runner/lanelet_utils.hpp>
#include <random_test_runner/random_test_runner.hpp>
#include <random_test_runner/test_randomizer.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <vector>

RandomTestRunner::RandomTestRunner(const rclcpp::NodeOptions & option)
: Node("random_test_runner", option), error_reporter_(get_logger())
{}

void RandomTestRunner::initialize()
{
  TestControlParameters test_control_parameters = collectAndValidateTestControlParameters();
  std::string message = fmt::format("test control parameters: {}", test_control_parameters);
  RCLCPP_INFO_STREAM(get_logger(), message);

  random_test_runner::Params::TestSuite test_suite_params;
  std::vector<TestCaseParameters> test_case_parameters_vector;
  switch (test_control_parameters.random_test_type) {
    case RANDOM_RUN: {
      auto param_listener = std::make_shared<random_test_runner::ParamListener>(shared_from_this());
      auto random_test_runner_params = param_listener->get_params();
      test_suite_params = random_test_runner_params.test_suite;
      TestCaseParameters test_case_parameters = collectTestCaseParameters();

      for (int test_id = 0; test_id < test_control_parameters.test_count; test_id++) {
        TestCaseParameters current_test_case_parameters = test_case_parameters;
        if (current_test_case_parameters.seed < 0) {
          current_test_case_parameters.seed = seed_randomization_device_();
        }
        test_case_parameters_vector.emplace_back(current_test_case_parameters);
      }
    } break;
    case REPLAY:
      YamlTestParamsIO yaml_params_reader(get_logger(), test_control_parameters.input_dir);
      std::tie(test_suite_params, test_case_parameters_vector) = yaml_params_reader.read();
      break;
  }

  std::string map_path =
    ament_index_cpp::get_package_share_directory(test_suite_params.map_name) + "/map";

  message = fmt::format("Map path found: {}", map_path);
  RCLCPP_INFO_STREAM(get_logger(), message);

  traffic_simulator::Configuration configuration(map_path);
  configuration.simulator_host = test_control_parameters.simulator_host;
  auto lanelet_utils = std::make_shared<LaneletUtils>(configuration.lanelet2_map_path());

  auto validated_params = validateParameters(test_suite_params, lanelet_utils);

  message = fmt::format("Test suite parameters {}", validated_params);
  RCLCPP_INFO_STREAM(get_logger(), message);
  for (size_t test_id = 0; test_id < test_case_parameters_vector.size(); test_id++) {
    std::string message =
      fmt::format("Test case {} parameters:", test_id, test_case_parameters_vector[test_id]);
    RCLCPP_INFO_STREAM(get_logger(), message);
  }

  error_reporter_.init(test_control_parameters.output_dir);

  YamlTestParamsIO yaml_test_params_saver(get_logger(), test_control_parameters.output_dir);

  yaml_test_params_saver.addTestSuite(validated_params, validated_params.test_name);

  for (size_t test_id = 0; test_id < test_case_parameters_vector.size(); test_id++) {
    std::string message =
      fmt::format("Generating test {}/{}", test_id + 1, test_case_parameters_vector.size());
    RCLCPP_INFO_STREAM(get_logger(), message);
    test_executors_.emplace_back(
      std::make_shared<traffic_simulator::API>(this, configuration, 1.0, 20),
      TestRandomizer(
        get_logger(), validated_params, test_case_parameters_vector[test_id], lanelet_utils)
        .generate(),
      error_reporter_.spawnTestCase(validated_params.test_name, std::to_string(test_id)),
      test_control_parameters.simulator_type, test_control_parameters.architecture_type,
      get_logger());
    yaml_test_params_saver.addTestCase(test_case_parameters_vector[test_id], validated_params.test_name);
  }

  current_test_executor_ = test_executors_.begin();
  yaml_test_params_saver.write();

  start();
}

TestCaseParameters RandomTestRunner::collectTestCaseParameters()
{
  return {this->declare_parameter<int64_t>("seed", -1)};
}

TestControlParameters RandomTestRunner::collectAndValidateTestControlParameters()
{
  TestControlParameters tp;
  tp.input_dir = this->declare_parameter<std::string>("input_dir", "");
  tp.output_dir = this->declare_parameter<std::string>("output_dir", "/tmp");
  tp.random_test_type = tp.input_dir.empty() ? RandomTestType::RANDOM_RUN : RandomTestType::REPLAY;
  tp.test_count = this->declare_parameter<int>("test_count", 5);
  tp.simulator_type = simulatorTypeFromString(
    this->declare_parameter<std::string>("simulator_type", "simple_sensor_simulator"));
  tp.architecture_type =
    architectureTypeFromString(this->declare_parameter<std::string>("architecture_type", ""));
  tp.simulator_host = this->declare_parameter<std::string>("simulator_host", "localhost");

  if (!tp.input_dir.empty() && !boost::filesystem::is_directory(tp.input_dir)) {
    throw std::runtime_error(
      fmt::format("Input directory {} does not exists or is not a directory", tp.input_dir));
  }

  if (tp.output_dir.empty() || !boost::filesystem::is_directory(tp.output_dir)) {
    throw std::runtime_error(fmt::format(
      "Output directory {} is empty, does not exists or is not a directory", tp.output_dir));
  }

  return tp;
}

random_test_runner::Params::TestSuite RandomTestRunner::validateParameters(
  const random_test_runner::Params::TestSuite & parameters, std::shared_ptr<LaneletUtils> lanelet_utils)
{
  auto tp = parameters;

  if (tp.ego_goal_lanelet_id == -1) {
    return tp;
  }

  auto lanelet_ids = lanelet_utils->getLaneletIds();

  if (std::none_of(lanelet_ids.begin(), lanelet_ids.end(), [&tp](int64_t lanelet_id) {
        return tp.ego_goal_lanelet_id == lanelet_id;
      })) {
    throw std::runtime_error(fmt::format("Lanelet {} does not exists.", tp.ego_goal_lanelet_id));
  }

  if (!lanelet_utils->isInLanelet(tp.ego_goal_lanelet_id, tp.ego_goal_s)) {
    throw std::runtime_error(fmt::format(
      "Invalid position inside lanelet: {}. Lanelet {} has length of {}.", tp.ego_goal_s,
      tp.ego_goal_lanelet_id, lanelet_utils->getLaneletLength(tp.ego_goal_lanelet_id)));
  }
  return tp;
}

void RandomTestRunner::update()
{
  if (current_test_executor_->scenarioCompleted()) {
    current_test_executor_->deinitialize();
    current_test_executor_++;
    if (current_test_executor_ == test_executors_.end()) {
      stop();
      return;
    }
    std::string message = fmt::format(
      "Running test {}/{}", std::distance(test_executors_.begin(), current_test_executor_) + 1,
      test_executors_.size());
    RCLCPP_INFO_STREAM(get_logger(), message);
    current_test_executor_->initialize();
  }

  current_test_executor_->update();
}

void RandomTestRunner::start()
{
  std::string message = fmt::format(
    "Running test {}/{}", std::distance(test_executors_.begin(), current_test_executor_) + 1,
    test_executors_.size());
  RCLCPP_INFO_STREAM(get_logger(), message);
  current_test_executor_->initialize();
  update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&RandomTestRunner::update, this));
}

void RandomTestRunner::stop()
{
  error_reporter_.write();
  update_timer_->cancel();
  rclcpp::shutdown();
}
