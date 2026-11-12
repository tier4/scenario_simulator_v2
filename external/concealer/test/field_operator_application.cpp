// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include <atomic>
#include <chrono>
#include <concealer/field_operator_application.hpp>
#include <get_parameter/get_parameter.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <thread>

namespace
{
using FieldOperatorApplication = concealer::FieldOperatorApplication;
using InitializeLocalization = FieldOperatorApplication::InitializeLocalization;
using ChangeOperationMode = FieldOperatorApplication::ChangeOperationMode;

class LocalizationInitialization : public testing::Test
{
protected:
  std::atomic<std::size_t> localization_requests{0};
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<InitializeLocalization>::SharedPtr initialize_localization;
  rclcpp::Service<ChangeOperationMode>::SharedPtr change_to_stop;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  std::thread server_thread;
  std::unique_ptr<FieldOperatorApplication> application;
  std::thread application_thread;

  static auto SetUpTestSuite() -> void { rclcpp::init(0, nullptr); }

  static auto TearDownTestSuite() -> void { rclcpp::shutdown(); }

  auto SetUp() -> void override
  {
    if (not rclcpp::ok()) {
      GTEST_SKIP() << "ROS context was shut down after a service failed to finish";
    }

    server = std::make_shared<rclcpp::Node>("localization_initialization_test");
    initialize_localization = server->create_service<InitializeLocalization>(
      "/api/localization/initialize", [this](
                                        InitializeLocalization::Request::SharedPtr,
                                        InitializeLocalization::Response::SharedPtr response) {
        ++localization_requests;
        // A zero timeout must not accidentally pass because a response arrives immediately.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        response->status.success = true;
      });
    change_to_stop = server->create_service<ChangeOperationMode>(
      "/api/operation_mode/change_to_stop",
      [](
        ChangeOperationMode::Request::SharedPtr,
        ChangeOperationMode::Response::SharedPtr response) { response->status.success = true; });
    server_executor.add_node(server);
    server_thread = std::thread([this] { server_executor.spin(); });
  }

  auto TearDown() -> void override
  {
    destroyApplication();
    server_executor.cancel();
    if (server_thread.joinable()) {
      server_thread.join();
    }
  }

  auto destroyApplication() -> void
  {
    if (application) {
      // The constructor queues a change-to-stop request. Keep both executors alive until it
      // finishes; otherwise destruction would wait for the service availability timeout.
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (not application->task_queue.empty() and std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (not application->task_queue.empty()) {
        ADD_FAILURE() << "The constructor's change-to-stop request did not finish";
        rclcpp::shutdown();
      }
      application->executor.cancel();
      application_thread.join();
      application.reset();
    }
  }

  auto createApplication() -> void
  {
    application = std::make_unique<FieldOperatorApplication>(0);
    application_thread = std::thread([this] { application->executor.spin(); });
  }

  auto requestInitialPose() -> void
  {
    application->requestInitialPose(
      std::make_shared<InitializeLocalization::Request>(), 1, std::chrono::seconds(5));
  }
};

TEST_F(LocalizationInitialization, timeout_defaults_overrides_and_validation)
{
  auto & parameters = common::getParameterNode();
  {
    SCOPED_TRACE("omitted timeout waits for a delayed response");
    ASSERT_FALSE(parameters.has_parameter("initialize_localization"));
    ASSERT_NO_THROW(createApplication());
    EXPECT_EQ(common::getParameter<int>("initialize_localization"), 10);
    EXPECT_NO_THROW(requestInitialPose());
    EXPECT_EQ(localization_requests.load(), 1U);
    destroyApplication();
  }
  {
    SCOPED_TRACE("positive timeout waits for a delayed response");
    ASSERT_TRUE(
      parameters.set_parameter(rclcpp::Parameter("initialize_localization", 1)).successful);
    ASSERT_NO_THROW(createApplication());
    EXPECT_EQ(common::getParameter<int>("initialize_localization"), 1);
    EXPECT_NO_THROW(requestInitialPose());
    EXPECT_EQ(localization_requests.load(), 2U);
    destroyApplication();
  }
  {
    SCOPED_TRACE("zero timeout is rejected before requesting localization");
    ASSERT_TRUE(
      parameters.set_parameter(rclcpp::Parameter("initialize_localization", 0)).successful);
    EXPECT_THROW(createApplication(), common::SemanticError);
    EXPECT_EQ(localization_requests.load(), 2U);
    destroyApplication();
  }
  {
    SCOPED_TRACE("negative timeout is rejected before requesting localization");
    ASSERT_TRUE(
      parameters.set_parameter(rclcpp::Parameter("initialize_localization", -1)).successful);
    EXPECT_THROW(createApplication(), common::SemanticError);
    EXPECT_EQ(localization_requests.load(), 2U);
    destroyApplication();
  }
}
}  // namespace
