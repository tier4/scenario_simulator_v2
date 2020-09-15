#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <type_traits>

#define SCENARIO_RUNNER_ALLOW_ATTRIBUTES_TO_BE_BLANK
// #define SCENARIO_RUNNER_NO_EXTENSION

#include <scenario_runner/syntax/open_scenario.hpp>

enum class Result
{
  success = EXIT_SUCCESS, // 0
  failure = EXIT_FAILURE, // any non-0 value

  syntax_error,
  implementation_error,
  unexpected_error,
};

static_assert(EXIT_SUCCESS < EXIT_FAILURE);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options {};

  auto node {rclcpp::Node::make_shared("scenario_runner_node")};

  std::string scenario {};
  // handle.getParam("scenario", scenario);
  //
  // bool verbose { false };
  // handle.getParam("verbose", verbose);
  //
  // int port {};
  // handle.param<int>("port", port, 8080);

  try {
    using namespace scenario_runner;

    OpenSCENARIO osc {scenario, "127.0.0.1", 8080};

    rclcpp::Rate rate {50};

    for (osc.init(); not osc.complete(); rate.sleep()) {
      const auto result {osc.evaluate()};

      std::cout << "[Storyboard: " << result << "]" << std::endl;

      std::cout << "[" <<
      (  standby_state.use_count() - 1) << " standby, " <<
      (  running_state.use_count() - 1) << " running, " <<
      ( complete_state.use_count() - 1) << " complete, and " <<
      (stop_transition.use_count() - 1) << " stopping " <<
        " (" << (start_transition.use_count() + end_transition.use_count() - 2) <<
        " in transition)" <<
        "]\n" <<
        std::endl;
    }
  } catch (const scenario_runner::Command & command) {
    switch (command) {
      case scenario_runner::Command::exitSuccess:
        RCLCPP_INFO((*node).get_logger(), "Simulation succeeded.");
        return static_cast<std::underlying_type<Result>::type>(Result::success);

      default:
      case scenario_runner::Command::exitFailure:
        RCLCPP_INFO((*node).get_logger(), "Simulation failed.");
        return static_cast<std::underlying_type<Result>::type>(Result::failure);
    }
  } catch (const scenario_runner::SyntaxError & error) {
    RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
    return static_cast<std::underlying_type<Result>::type>(Result::syntax_error);
  } catch (const scenario_runner::ImplementationFault & error) {
    RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
    return static_cast<std::underlying_type<Result>::type>(Result::implementation_error);
  } catch (const std::exception & error) {
    RCLCPP_ERROR((*node).get_logger(), "%s.", error.what());
    return static_cast<std::underlying_type<Result>::type>(Result::unexpected_error);
  }

  RCLCPP_INFO((*node).get_logger(), "Simulation succeeded.");
  return static_cast<std::underlying_type<Result>::type>(Result::success);
}
