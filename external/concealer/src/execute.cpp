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

#include <err.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <concealer/execute.hpp>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <type_traits>

namespace concealer
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  If you can't explain the difference between char * and char [], don't edit
 *  this function even if it looks strange.
 *
 * -------------------------------------------------------------------------- */
auto execute(const std::vector<std::string> & f_xs) -> int
{
  std::vector<std::vector<char>> buffer;

  buffer.resize(f_xs.size());

  std::vector<std::add_pointer<char>::type> argv;

  argv.reserve(f_xs.size());

  for (const auto & each : f_xs) {
    buffer.emplace_back(std::begin(each), std::end(each));
    buffer.back().push_back('\0');
    argv.push_back(buffer.back().data());
  }

  argv.emplace_back(static_cast<std::add_pointer<char>::type>(0));

  return ::execvp(argv[0], argv.data());
}

auto dollar(const std::string & command) -> std::string
{
  std::array<char, 128> buffer;

  std::string result;

  std::unique_ptr<FILE, decltype(&pclose)> pipe{::popen(command.c_str(), "r"), pclose};

  if (not pipe) {
    throw std::system_error(errno, std::system_category());
  } else {
    while (std::fgets(buffer.data(), buffer.size(), pipe.get())) {
      result += buffer.data();
    }
    return result;
  }
}

void sudokill(const pid_t process_id)
{
  const auto process_str = std::to_string(process_id);

  const pid_t pid = fork();

  switch (pid) {
    case -1:
      std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
      break;
    case 0:
      ::execlp("sudo", "sudo", "kill", "-2", process_str.c_str(), static_cast<char *>(nullptr));
      std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
      break;
  }
}

}  // namespace concealer
