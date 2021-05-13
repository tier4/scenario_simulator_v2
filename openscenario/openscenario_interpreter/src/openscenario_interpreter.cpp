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

#define OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK
#define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#define AUTOWARE_ARCHITECTURE_PROPOSAL

// #undef NDEBUG

#include <boost/filesystem.hpp>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace openscenario_interpreter
{
Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("openscenario_interpreter", options),
  intended_result("success"),
  local_frame_rate(30),
  local_real_time_factor(1.0),
  osc_path(""),
  output_directory("/tmp")
{
#define DECLARE_PARAMETER(IDENTIFIER) \
  declare_parameter<decltype(IDENTIFIER)>(#IDENTIFIER, IDENTIFIER)

  DECLARE_PARAMETER(intended_result);
  DECLARE_PARAMETER(local_frame_rate);
  DECLARE_PARAMETER(local_real_time_factor);
  DECLARE_PARAMETER(osc_path);
  DECLARE_PARAMETER(output_directory);

#undef DECLARE_PARAMETER
}

void Interpreter::report(
  const junit_exporter::TestResult & result,  //
  const std::string & type,                   //
  const std::string & what)
{
  std::stringstream message;
  {
    message << (result == SUCCESS ? "\x1b[1;32m" : "\x1b[1;31m") << type.c_str();

    if (not what.empty()) {
      message << " (" << what.c_str() << ")";
    }

    message << "\x1b[0m";

    RCLCPP_INFO_STREAM(get_logger(), message.str());
  }

  exporter.addTestCase(  // XXX DIRTY HACK!!!
    script.as<OpenScenario>().scope.scenario.string(), "scenario_testing", 0, result, type, what);

  exporter.write(output_directory + "/result.junit.xml");

  script.reset();

  while (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  deactivate();
}

Interpreter::Result Interpreter::on_configure(const rclcpp_lifecycle::State &)
try {
  std::this_thread::sleep_for(std::chrono::seconds(1));

#define GET_PARAMETER(IDENTIFIER) get_parameter(#IDENTIFIER, IDENTIFIER)

  GET_PARAMETER(intended_result);
  GET_PARAMETER(local_frame_rate);
  GET_PARAMETER(local_real_time_factor);
  GET_PARAMETER(osc_path);
  GET_PARAMETER(output_directory);

#undef GET_PARAMETER

  script.rebind<OpenScenario>(osc_path);

  connect(
    shared_from_this(),                                       //
    boost::filesystem::path(osc_path).replace_extension(""),  // NOTE: /path/to/lanelet2_map.osm
    script.as<OpenScenario>().scope.logic_file.string(),      //
    30);

  initialize(
    local_real_time_factor,
    1 / local_frame_rate * local_real_time_factor);  // interval_upper_bound

  return Interpreter::Result::SUCCESS;
} catch (const openscenario_interpreter::SyntaxError & error) {
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[1;31m" << error.what() << "\x1b[0m");
  return Interpreter::Result::FAILURE;
}

Interpreter::Result Interpreter::on_activate(const rclcpp_lifecycle::State &)
{
  timer = create_wall_timer(
    std::chrono::milliseconds(static_cast<unsigned int>(1 / local_frame_rate * 1000)), [this]() {
      withExceptionHandler([this]() {
        if (script) {
          if (!script.as<OpenScenario>().complete()) {
            script.as<OpenScenario>().evaluate();
#ifndef NDEBUG
            RCLCPP_INFO_STREAM(
              get_logger(),
              "[" << (openscenario_interpreter::standby_state.use_count() - 1) << " standby (=> "
                  << (openscenario_interpreter::start_transition.use_count() - 1) << ") => "
                  << (openscenario_interpreter::running_state.use_count() - 1) << " running (=> "
                  << (openscenario_interpreter::stop_transition.use_count() - 1) << ") => "
                  << (openscenario_interpreter::complete_state.use_count() - 1) << " complete]");
#endif
          }
        }
      });
    });

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_cleanup(const rclcpp_lifecycle::State &)
{
  connection.~API();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_error(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Interpreter)
