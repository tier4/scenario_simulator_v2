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

// clang-format off (NOTE: ament-clang-format does not respect the include order)
#include <concealer/autoware_def.hpp>
// clang-format on

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <concealer/execute.hpp>
#include <memory>
#include <nlohmann/json.hpp>
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

#define DECLARE_PARAMETER(IDENTIFIER) \
  declare_parameter<decltype(IDENTIFIER)>(#IDENTIFIER, IDENTIFIER)

Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("openscenario_interpreter", options),
  publisher_of_context(create_publisher<Context>("context", rclcpp::QoS(1).transient_local())),
  intended_result("success"),
  local_frame_rate(30),
  local_real_time_factor(1.0),
  osc_path(""),
  output_directory("/tmp")
{
  DECLARE_PARAMETER(intended_result);
  DECLARE_PARAMETER(local_frame_rate);
  DECLARE_PARAMETER(local_real_time_factor);
  DECLARE_PARAMETER(osc_path);
  DECLARE_PARAMETER(output_directory);

  reset();
}

#undef DECLARE_PARAMETER

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
    "python3", boost::algorithm::replace_all_copy(concealer::dollar("which ros2"), "\n", ""), "bag",
    "record", std::forward<decltype(xs)>(xs)...};

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

#define GET_PARAMETER(IDENTIFIER) get_parameter(#IDENTIFIER, IDENTIFIER)

Interpreter::Result Interpreter::on_configure(const rclcpp_lifecycle::State &)
try {
  INTERPRETER_INFO_STREAM("Configuring.");

  std::this_thread::sleep_for(std::chrono::seconds(1));  // NOTE: Wait for parameters to be set.

  GET_PARAMETER(intended_result);
  GET_PARAMETER(local_frame_rate);
  GET_PARAMETER(local_real_time_factor);
  GET_PARAMETER(osc_path);
  GET_PARAMETER(output_directory);

  record_start(
    "-a",  //
    "-o", boost::filesystem::path(osc_path).replace_extension("").string());

  script.rebind<OpenScenario>(osc_path);

  auto configuration = traffic_simulator::Configuration(
    boost::filesystem::is_directory(script.as<OpenScenario>().logic_file)
      ? script.as<OpenScenario>().logic_file
      : script.as<OpenScenario>().logic_file.parent_path());
  {
    configuration.auto_sink = false;

    configuration.initialize_duration =
      std::any_of(
        std::cbegin(script.as<OpenScenario>().entities),
        std::cend(script.as<OpenScenario>().entities),
        [](auto && each) {
          return std::get<1>(each).template as<ScenarioObject>().object_controller.isEgo();
        })
        ? 30
        : 0;

    configuration.scenario_path = osc_path;

    // XXX DIRTY HACK!!!
    if (
      not boost::filesystem::is_directory(script.as<OpenScenario>().logic_file) and
      script.as<OpenScenario>().logic_file.extension() == ".osm") {
      configuration.lanelet2_map_file = script.as<OpenScenario>().logic_file.filename().string();
    }
  }

  connect(shared_from_this(), configuration);

  initialize(
    local_real_time_factor,
    1 / local_frame_rate * local_real_time_factor);  // interval_upper_bound

  return Interpreter::Result::SUCCESS;
} catch (const openscenario_interpreter::SyntaxError & error) {
  INTERPRETER_ERROR_STREAM(error.what());
  return Interpreter::Result::FAILURE;
}

#undef GET_PARAMETER

Interpreter::Result Interpreter::on_activate(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("Activating.");

  const auto period =
    std::chrono::milliseconds(static_cast<unsigned int>(1 / local_frame_rate * 1000));

  execution_timer.clear();

  (*publisher_of_context).on_activate();

  timer = create_wall_timer(period, [this, period]() {
    guard([this, period]() {
      if (script) {
        if (not script.as<OpenScenario>().complete()) {
          const auto evaluate_time = execution_timer.invoke("evaluate", [&] {
            script.as<OpenScenario>().evaluate();

            Context context;
            {
              nlohmann::json json;
              {
                json << script.as<OpenScenario>();

                // std::cout << json.dump(2) << std::endl;  // DEBUG
              }

              context.stamp = now();
              context.data = json.dump();
            }

            if ((*publisher_of_context).is_activated()) {
              (*publisher_of_context).publish(context);
            } else {
              throw Error("Interpreter's publisher has not been activated yet");
            }

            return getCurrentTime() >= 0;  // statistics only if getCurrentTime() >= 0
          });

          if (getCurrentTime() >= 0 && evaluate_time > period) {
            using namespace std::chrono;
            const auto time_ms = duration_cast<milliseconds>(evaluate_time).count();
            const auto & time_statistics = execution_timer.getStatistics("evaluate");
            // clang-format off
            RCLCPP_WARN_STREAM(get_logger(),
              "The execution time of evaluate() (" <<  time_ms << " ms) is not in time. " <<
              "The current local frame rate (" << local_frame_rate << " Hz) (period = " << period.count() << " ms) is too high. " <<
              "If the frame rate is less than " << static_cast<unsigned int>(1.0 / time_ms * 1e3) << " Hz, you will make it. " <<
              "(Statistics: " <<
              "count = " << time_statistics.count() << ", " <<
              "mean = " << duration_cast<milliseconds>(time_statistics.mean()).count() << " ms, " <<
              "max = " << duration_cast<milliseconds>(time_statistics.max()).count() << " ms, " <<
              "standard deviation = " << duration_cast<microseconds>(time_statistics.standardDeviation()).count() / 1000.0 << " ms)"
            );
            // clang-format on
          }
        }
      } else {
        throw Error("No script evaluable");
      }
    });
  });

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_deactivate(const rclcpp_lifecycle::State &)
{
  INTERPRETER_INFO_STREAM("Deactivating.");

  timer.reset();  // Deactivate scenario evaluation

  (*publisher_of_context).on_deactivate();

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
    script.as<OpenScenario>().pathname.parent_path().stem().string(),
    script.as<OpenScenario>().pathname.string(),  // case-name (XXX: DIRTY HACK!!!)
    0,                                            // time
    current_result,                               //
    current_error_type,                           //
    current_error_what);

  test_suites.write(output_directory + "/result.junit.xml");

  {
    const auto suite_name = script.as<OpenScenario>().pathname.parent_path().stem().string();

    const auto case_name = script.as<OpenScenario>().pathname.stem().string();

    switch (current_result) {
      case junit::TestResult::ERROR:
        simple_test_suites.testsuite(suite_name)
          .testcase(case_name)
          .error.emplace_back(current_error_type, current_error_what);
        break;

      default:
        break;
    }

    simple_test_suites.write_to("/tmp/scenario_test_runner/new-result.junit.xml", "  ");
  }

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
