// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <sys/types.h>
#include <sys/wait.h>

#include <iostream>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/string/split.hpp>
#include <system_error>

namespace openscenario_interpreter
{
inline namespace posix
{
int execvp(const std::vector<std::string> & f_xs)
{
  std::vector<std::vector<char>> buffer;

  buffer.resize(f_xs.size());

  std::vector<std::add_pointer<char>::type> argv{};

  argv.reserve(f_xs.size());

  for (const auto & each : f_xs) {
    buffer.emplace_back(std::begin(each), std::end(each));
    buffer.back().push_back('\0');

    argv.push_back(buffer.back().data());
  }

  argv.emplace_back(static_cast<std::add_pointer<char>::type>(0));

  return ::execvp(argv[0], argv.data());
}

pid_t fork_exec(const std::vector<std::string> & f_xs)
{
  const auto pid = fork();

  if (pid < 0) {
    throw std::system_error(errno, std::system_category());
  } else {
    int status = 0;

    switch (pid) {
      case 0:
        if (execvp(f_xs) < 0) {
          std::cerr << std::system_error(errno, std::system_category()).what() << std::endl;
          std::exit(EXIT_FAILURE);
        }
        break;

      default:
        do {
          ::waitpid(pid, &status, WUNTRACED);
        } while (!WIFEXITED(status) && !WIFSIGNALED(status));
    }

    return pid;
  }
}

pid_t fork_exec(const std::string & f_xs) { return fork_exec(split(f_xs)); }

pid_t fork_exec(const std::string & f, const std::string & xs)
{
  return fork_exec(xs.empty() ? f : f + " " + xs);
}
}  // namespace posix
}  // namespace openscenario_interpreter
