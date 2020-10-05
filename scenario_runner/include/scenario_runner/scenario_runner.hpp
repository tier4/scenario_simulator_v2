// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_
#define SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_runner/syntax/open_scenario.hpp>
#include <scenario_simulator_msgs/srv/launcher_msg.hpp>

#include <memory>
#include <string>

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SCENARIO_RUNNER_EXPORT __attribute__ ((dllexport))
    #define SCENARIO_RUNNER_IMPORT __attribute__ ((dllimport))
  #else
    #define SCENARIO_RUNNER_EXPORT __declspec(dllexport)
    #define SCENARIO_RUNNER_IMPORT __declspec(dllimport)
  #endif

  #ifdef SCENARIO_RUNNER_BUILDING_DLL
    #define SCENARIO_RUNNER_PUBLIC SCENARIO_RUNNER_EXPORT
  #else
    #define SCENARIO_RUNNER_PUBLIC SCENARIO_RUNNER_IMPORT
  #endif

  #define SCENARIO_RUNNER_PUBLIC_TYPE SCENARIO_RUNNER_PUBLIC
  #define SCENARIO_RUNNER_LOCAL
#else
  #define SCENARIO_RUNNER_EXPORT __attribute__ ((visibility("default")))
  #define SCENARIO_RUNNER_IMPORT

  #if __GNUC__ >= 4
    #define SCENARIO_RUNNER_PUBLIC __attribute__ ((visibility("default")))
    #define SCENARIO_RUNNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SCENARIO_RUNNER_PUBLIC
    #define SCENARIO_RUNNER_LOCAL
  #endif

  #define SCENARIO_RUNNER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

namespace scenario_runner
{
class ScenarioRunner
  : public rclcpp_lifecycle::LifecycleNode
{
  using GetScenario = scenario_simulator_msgs::srv::LauncherMsg;  // CurrentScenario

  const rclcpp::Client<GetScenario>::SharedPtr service_client;

  int port;

  std::string scenario;

  std::string address {"127.0.0.1"};

  Element evaluate;

  std::shared_ptr<rclcpp::TimerBase> timer;

public:
  SCENARIO_RUNNER_PUBLIC
  explicit ScenarioRunner(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override;

  Result on_activate(const rclcpp_lifecycle::State &) override;

  Result on_deactivate(const rclcpp_lifecycle::State &) override;

  Result on_cleanup(const rclcpp_lifecycle::State &) override;

  Result on_shutdown(const rclcpp_lifecycle::State &) override;

  Result on_error(const rclcpp_lifecycle::State &) override;
};
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_
