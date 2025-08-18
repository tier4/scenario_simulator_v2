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

#ifndef OPENSCENARIO_INTERPRETER__RECORD_HPP_
#define OPENSCENARIO_INTERPRETER__RECORD_HPP_

#include <unistd.h>

#ifndef OPENSCENARIO_INTERPRETER_VERBOSE_RECORD
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif  // OPENSCENARIO_INTERPRETER_RECORD_QUIETLY

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

inline auto start(const std::vector<std::string> & args) -> pid_t
{
  std::vector<std::string> command{
    "python3", boost::algorithm::replace_all_copy(concealer::dollar("which ros2"), "\n", ""), "bag",
    "record"};

  command.insert(command.end(), args.begin(), args.end());

  switch (process_id = fork()) {
    case -1:
      throw std::system_error(errno, std::system_category());

    case 0:
#ifndef OPENSCENARIO_INTERPRETER_VERBOSE_RECORD
      if (const auto fd = ::open("/dev/null", O_WRONLY)) {
        ::dup2(fd, STDOUT_FILENO);
        ::close(fd);
      }
#endif
      if (concealer::execute(command) < 0) {
        std::cerr << std::system_error(errno, std::system_category()).what() << std::endl;
        std::exit(EXIT_FAILURE);
      }
      return 0;

    default:
      return process_id;
  }
}

auto stop() -> void;
}  // namespace record
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__RECORD_HPP_
