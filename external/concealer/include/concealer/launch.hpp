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

#ifndef CONCEALER__LAUNCH_HPP_
#define CONCEALER__LAUNCH_HPP_

#include <sys/prctl.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <concealer/execute.hpp>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

namespace concealer
{
template <typename Parameters>
auto ros2_launch(
  const std::string & package, const std::string & file, const Parameters & parameters)
{
  const auto argv = [&]() {
    auto argv = std::vector<std::string>();

    argv.push_back("python3");
    argv.push_back(boost::algorithm::replace_all_copy(dollar("which ros2"), "\n", ""));
    argv.push_back("launch");
    argv.push_back(package);
    argv.push_back(file);

    for (const auto & parameter : parameters) {
      argv.push_back(parameter);
    }

    return argv;
  }();

  const auto parent_process_id = getpid();

  if (const auto process_id = fork(); process_id < 0) {
    throw std::system_error(errno, std::system_category());
  } else if (process_id == 0) {
    /*
       NOTE: Prepare the child so that the Autoware nodes the child is about
       to become can always be terminated: by the destructor of
       `FieldOperatorApplication` while this process lives, and by
       `PR_SET_PDEATHSIG` if this process dies first. `PR_SET_PDEATHSIG`
       watches the thread that called `fork`, so `ros2_launch` must be called
       from the main thread.
    */
    if (setpgid(0, 0) < 0) {
      // `killpg` sends to a process group, so the child needs a group of its own.
      std::cout << "setpgid: " << std::system_error(errno, std::system_category()).what()
                << std::endl;
    } else if (prctl(PR_SET_PDEATHSIG, SIGINT) < 0 or getppid() != parent_process_id) {
      // SIGINT, because ROS 2 launch orphans the Autoware nodes on SIGTERM.
      // `getppid` catches a parent that died before `prctl` armed the signal.
      std::cout << "prctl: " << std::system_error(errno, std::system_category()).what()
                << std::endl;
    } else if (execute(argv) < 0) {
      std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    }
    // Neither unwind the stack nor run the atexit handlers inherited from the parent.
    // `FieldOperatorApplication::spinSome` turns this exit status into an error.
    std::_Exit(EXIT_FAILURE);
  } else if (setpgid(process_id, process_id) < 0 and errno != EACCES) {
    // Repeated here in case the child has not reached its own `setpgid` yet.
    // EACCES means the child reached `setpgid` first.
    throw std::system_error(errno, std::system_category());
  } else {
    return process_id;
  }
}
}  // namespace concealer

#endif  // CONCEALER__LAUNCH_HPP_
