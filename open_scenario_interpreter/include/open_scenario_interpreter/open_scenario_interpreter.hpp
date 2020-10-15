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

#ifndef OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_
#define OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_

#include <junit_exporter/junit_exporter.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <open_scenario_interpreter/syntax/open_scenario.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

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
    #define OPEN_SCENARIO_INTERPRETER_EXPORT __attribute__ ((dllexport))
    #define OPEN_SCENARIO_INTERPRETER_IMPORT __attribute__ ((dllimport))
  #else
    #define OPEN_SCENARIO_INTERPRETER_EXPORT __declspec(dllexport)
    #define OPEN_SCENARIO_INTERPRETER_IMPORT __declspec(dllimport)
  #endif

  #ifdef OPEN_SCENARIO_INTERPRETER_BUILDING_DLL
    #define OPEN_SCENARIO_INTERPRETER_PUBLIC OPEN_SCENARIO_INTERPRETER_EXPORT
  #else
    #define OPEN_SCENARIO_INTERPRETER_PUBLIC OPEN_SCENARIO_INTERPRETER_IMPORT
  #endif

  #define OPEN_SCENARIO_INTERPRETER_PUBLIC_TYPE OPEN_SCENARIO_INTERPRETER_PUBLIC
  #define OPEN_SCENARIO_INTERPRETER_LOCAL
#else
  #define OPEN_SCENARIO_INTERPRETER_EXPORT __attribute__ ((visibility("default")))
  #define OPEN_SCENARIO_INTERPRETER_IMPORT

  #if __GNUC__ >= 4
    #define OPEN_SCENARIO_INTERPRETER_PUBLIC __attribute__ ((visibility("default")))
    #define OPEN_SCENARIO_INTERPRETER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OPEN_SCENARIO_INTERPRETER_PUBLIC
    #define OPEN_SCENARIO_INTERPRETER_LOCAL
  #endif

  #define OPEN_SCENARIO_INTERPRETER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

namespace open_scenario_interpreter
{
class ScenarioRunner
  : public rclcpp_lifecycle::LifecycleNode
{
  std::string scenario;

  std::string expect;

  std::string log_path;

  Element evaluate;

  std::shared_ptr<rclcpp::TimerBase> timer;

  junit_exporter::JunitExporter exporter;

public:
  OPEN_SCENARIO_INTERPRETER_PUBLIC
  explicit ScenarioRunner(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override;

  Result on_activate(const rclcpp_lifecycle::State &) override;

  Result on_deactivate(const rclcpp_lifecycle::State &) override;

  Result on_cleanup(const rclcpp_lifecycle::State &) override;

  Result on_shutdown(const rclcpp_lifecycle::State &) override;

  Result on_error(const rclcpp_lifecycle::State &) override;
};
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_
