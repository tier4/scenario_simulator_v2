#ifndef SCENARIO_RUNNER__CONSOLE__IS_CONSOLE_HPP_
#define SCENARIO_RUNNER__CONSOLE__IS_CONSOLE_HPP_

#include <iostream>
#include <unistd.h>

namespace scenario_runner
{inline namespace console
{
auto is_console = [](const auto & os)
  {
    if (os.rdbuf() == std::cout.rdbuf()) {
      static const auto result {static_cast<bool>(::isatty(STDOUT_FILENO))};
      return result;
    } else if (os.rdbuf() == std::cerr.rdbuf()) {
      static const auto result {static_cast<bool>(::isatty(STDERR_FILENO))};
      return result;
    } else {
      return false;
    }
  };
}}  // namespace scenario_runner::console

#endif  // SCENARIO_RUNNER__CONSOLE__IS_CONSOLE_HPP_
