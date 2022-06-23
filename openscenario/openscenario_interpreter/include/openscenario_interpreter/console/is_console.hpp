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

#ifndef OPENSCENARIO_INTERPRETER__CONSOLE__IS_CONSOLE_HPP_
#define OPENSCENARIO_INTERPRETER__CONSOLE__IS_CONSOLE_HPP_

#include <unistd.h>

#include <iostream>

namespace openscenario_interpreter
{
inline namespace console
{
inline auto is_console = [](const auto & os) {
  if (os.rdbuf() == std::cout.rdbuf()) {
    static const auto result{static_cast<bool>(::isatty(STDOUT_FILENO))};
    return result;
  } else if (os.rdbuf() == std::cerr.rdbuf()) {
    static const auto result{static_cast<bool>(::isatty(STDERR_FILENO))};
    return result;
  } else {
    return false;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__CONSOLE__IS_CONSOLE_HPP_
