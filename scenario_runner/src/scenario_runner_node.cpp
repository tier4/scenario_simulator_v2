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

#define SCENARIO_RUNNER_ALLOW_ATTRIBUTES_TO_BE_BLANK
// #define SCENARIO_RUNNER_NO_EXTENSION

#include <rclcpp/rclcpp.hpp>
#include <scenario_runner/scenario_runner.hpp>

#include <cstdlib>
#include <string>
#include <thread>
#include <type_traits>
#include <memory>

// enum class Result
// {
//   success = EXIT_SUCCESS,  // 0
//   failure = EXIT_FAILURE,  // any non-0 value
//
//   syntax_error,
//   implementation_error,
//   unexpected_error,
// };

static_assert(EXIT_SUCCESS < EXIT_FAILURE);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor {};

  const auto node {std::make_shared<scenario_runner::ScenarioRunner>()};

  executor.add_node((*node).get_node_base_interface());

  (*node).configure(); // XXX DIRTY HACK
  (*node).activate(); // XXX DIRTY HACK

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;

  // rclcpp::NodeOptions options {};
  //
  // auto node {rclcpp::Node::make_shared("scenario_runner_node")};
  //
  // std::string scenario {""};
  // (*node).declare_parameter<decltype(scenario)>("scenario", scenario);
  // (*node).get_parameter("scenario", scenario);
  //
  // bool verbose {false};
  // (*node).declare_parameter<decltype(verbose)>("verbose", verbose);
  // (*node).get_parameter("verbose", verbose);
  //
  // int port {8080};
  // (*node).declare_parameter<decltype(port)>("port", port);
  // (*node).get_parameter("port", port);
  //
  // try {
  //   scenario_runner::OpenSCENARIO osc {scenario, "127.0.0.1", 8080};
  //
  //   rclcpp::Rate rate {50};
  //
  //   for (osc.init(); !osc.complete(); rate.sleep()) {
  //     const auto result {osc.evaluate()};
  //
  //     std::cout << "[Storyboard: " << result << "]" << std::endl;
  //
  //     std::cout << "[" <<
  //     (scenario_runner::standby_state.use_count() - 1) << " standby, " <<
  //     (scenario_runner::running_state.use_count() - 1) << " running, " <<
  //     (scenario_runner::complete_state.use_count() - 1) << " complete, and " <<
  //     (scenario_runner::stop_transition.use_count() - 1) << " stopping " <<
  //       " (" <<
  //     (scenario_runner::start_transition.use_count() +
  //     scenario_runner::end_transition.use_count() - 2) <<
  //       " in transition)" <<
  //       "]\n" <<
  //       std::endl;
  //   }
  // } catch (const scenario_runner::Command & command) {
  //   switch (command) {
  //     case scenario_runner::Command::exitSuccess:
  //       RCLCPP_INFO((*node).get_logger(), "Simulation succeeded.");
  //       return static_cast<std::underlying_type<Result>::type>(Result::success);
  //
  //     default:
  //     case scenario_runner::Command::exitFailure:
  //       RCLCPP_INFO((*node).get_logger(), "Simulation failed.");
  //       return static_cast<std::underlying_type<Result>::type>(Result::failure);
  //   }
  // } catch (const scenario_runner::SyntaxError & error) {
  //   RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
  //   return static_cast<std::underlying_type<Result>::type>(Result::syntax_error);
  // } catch (const scenario_runner::ImplementationFault & error) {
  //   RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
  //   return static_cast<std::underlying_type<Result>::type>(Result::implementation_error);
  // } catch (const std::exception & error) {
  //   RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
  //   return static_cast<std::underlying_type<Result>::type>(Result::unexpected_error);
  // }
  //
  // RCLCPP_INFO((*node).get_logger(), "Simulation succeeded.");
  // return static_cast<std::underlying_type<Result>::type>(Result::success);
}
