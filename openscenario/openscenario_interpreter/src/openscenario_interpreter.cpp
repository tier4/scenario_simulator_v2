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

// #define AUTOWARE_ARCHITECTURE_PROPOSAL

// #undef NDEBUG

#include <algorithm>
#include <boost/filesystem.hpp>
#include <concealer/execute.hpp>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace openscenario_interpreter
{
#define INTERPRETER_INFO_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[32m" << __VA_ARGS__ << "\x1b[0m")

// NOTE: Error on simulation is not error of the interpreter; so we print error messages into INFO_STREAM.
#define INTERPRETER_ERROR_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[1;31m" << __VA_ARGS__ << "\x1b[0m")

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

  reset();
}

void Interpreter::report(
  const junit::TestResult & result,  //
  const std::string & type,          //
  const std::string & what)
{
  current_result = result;
  current_error_type = type;
  current_error_what = what;

  INTERPRETER_INFO_STREAM("Deactivate myself.");
  deactivate();
  INTERPRETER_INFO_STREAM("Deactivated myself.");
}

pid_t record_process_id = 0;

template <typename... Ts>
auto record_start(Ts &&... xs)
{
  record_process_id = fork();

  const std::vector<std::string> argv{
    "python3", "/opt/ros/foxy/bin/ros2", "bag", "record", std::forward<decltype(xs)>(xs)...};

  if (record_process_id < 0) {
    throw std::system_error(errno, std::system_category());
  } else if (record_process_id == 0 and concealer::execute(argv) < 0) {
    std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    std::exit(EXIT_FAILURE);
  } else {
    return record_process_id;
  }
}

auto record_end()
{
  int status = 0;

  if (::kill(record_process_id, SIGINT) or waitpid(record_process_id, &status, 0) < 0) {
    std::exit(EXIT_FAILURE);
  }
}

Interpreter::Result Interpreter::on_configure(const rclcpp_lifecycle::State &)
try {
  INTERPRETER_INFO_STREAM("Configuring.");

  // NOTE: Wait for parameters to be set.
  std::this_thread::sleep_for(std::chrono::seconds(1));

#define GET_PARAMETER(IDENTIFIER) get_parameter(#IDENTIFIER, IDENTIFIER)

  GET_PARAMETER(intended_result);
  GET_PARAMETER(local_frame_rate);
  GET_PARAMETER(local_real_time_factor);
  GET_PARAMETER(osc_path);
  GET_PARAMETER(output_directory);

#undef GET_PARAMETER

  record_start(
    "-a",  //
    "-o", boost::filesystem::path(osc_path).replace_extension("").string());

  script.rebind<OpenScenario>(osc_path);

  const auto with_autoware = std::any_of(
    std::begin(script.as<OpenScenario>().scope.entities),
    std::end(script.as<OpenScenario>().scope.entities), [](auto & each) {
      return std::get<1>(each).template as<ScenarioObject>().object_controller.isEgo();
    });

  connect(
    shared_from_this(),                                       //
    boost::filesystem::path(osc_path).replace_extension(""),  // NOTE: /path/to/lanelet2_map.osm
    script.as<OpenScenario>().scope.logic_file.string(),      //
    with_autoware ? 30 : 0,
    false  // auto-sink
  );

  initialize(
    local_real_time_factor,
    1 / local_frame_rate * local_real_time_factor);  // interval_upper_bound

  return Interpreter::Result::SUCCESS;
} catch (const openscenario_interpreter::SyntaxError & error) {
  INTERPRETER_ERROR_STREAM(error.what());
  return Interpreter::Result::FAILURE;
}

Interpreter::Result Interpreter::on_activate(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("Activating.");

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
        } else {
          throw ImplementationFault("No script evaluable");
        }
      });
    });

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_deactivate(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("Deactivating.");

  timer.reset();  // Deactivate scenario evaluation

  connection.~API();  // Deactivate simulator

  std::stringstream message;
  {
    message << (current_result == SUCCESS ? "\x1b[32m" : "\x1b[1;31m")
            << current_error_type.c_str();

    if (not current_error_what.empty()) {
      message << " (" << current_error_what.c_str() << ")";
    }

    message << "\x1b[0m";
  }

  // NOTE: Error on simulation is not error of the interpreter; so we print error messages into INFO_STREAM.
  RCLCPP_INFO_STREAM(get_logger(), message.str());

  record_end();

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_cleanup(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("CleaningUp.");

  test_suites.addTestCase(
    script.as<OpenScenario>().scope.scenario.parent_path().stem().string(),
    script.as<OpenScenario>().scope.scenario.string(),  // case-name (XXX: DIRTY HACK!!!)
    0,                                                  // time
    current_result,                                     //
    current_error_type,                                 //
    current_error_what);

  test_suites.write(output_directory + "/result.junit.xml");

  script.reset();

  reset();

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_shutdown(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("ShuttingDown.");
  timer.reset();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_error(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("ErrorProcessing.");
  timer.reset();
  return Interpreter::Result::SUCCESS;
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Interpreter)
