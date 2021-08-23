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

#ifndef OPENSCENARIO_INTERPRETER__RECORD_HPP_
#define OPENSCENARIO_INTERPRETER__RECORD_HPP_

#include <unistd.h>

#include <boost/algorithm/string.hpp>  // boost::algorithm::replace_all_copy
#include <concealer/execute.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace openscenario_interpreter
{
namespace record
{
extern pid_t process_id;

template <typename... Ts>
auto start(Ts &&... xs) -> pid_t
{
  const std::vector<std::string> command{
    "python3", boost::algorithm::replace_all_copy(concealer::dollar("which ros2"), "\n", ""), "bag",
    "record", std::forward<decltype(xs)>(xs)...};

  if ((process_id = fork()) < 0) {
    throw std::system_error(errno, std::system_category());
  } else if (process_id == 0 and concealer::execute(command) < 0) {
    std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    std::exit(EXIT_FAILURE);
  } else {
    return process_id;
  }
}

auto stop() -> void;
}  // namespace record
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__RECORD_HPP_
