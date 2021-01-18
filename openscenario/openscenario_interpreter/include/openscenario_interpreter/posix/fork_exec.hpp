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

#ifndef OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_
#define OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_

#include <unistd.h>

#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace posix
{
auto execvp(const std::vector<std::string> & args)
{
  std::vector<std::vector<char>> buffer {};

  buffer.resize(args.size());

  std::vector<std::add_pointer<char>::type> argv {};

  argv.reserve(args.size());

  for (const auto & each : args) {
    #ifndef NDEBUG
    std::cout << std::quoted(each) << std::endl;
    #endif
    buffer.emplace_back(std::begin(each), std::end(each));
    buffer.back().push_back('\0');

    argv.push_back(buffer.back().data());
  }

  argv.emplace_back(static_cast<std::add_pointer<char>::type>(0));

  return ::execvp(argv[0], argv.data());
}
}  // namespace posix
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_
