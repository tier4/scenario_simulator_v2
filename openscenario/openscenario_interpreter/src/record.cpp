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

#include <sys/wait.h>

#include <chrono>
#include <openscenario_interpreter/record.hpp>
#include <thread>

namespace openscenario_interpreter
{
namespace record
{
pid_t process_id = 0;

auto stop() -> void
{
  if (int status = 0; process_id) {
    /*
       NOTE: If the interval between running `record::start(...)` and
       `record::stop()` is too short, the bag file will not be saved
       successfully. In such cases, the ros2 bag record command is not
       recording, but before it, that is, booting. So SIGINT doesn't end the
       record, it just kills the Python interpreter.
    */
    std::this_thread::sleep_for(std::chrono::seconds(3));  // DIRTY HACK!!!

    if (::kill(process_id, SIGINT) or waitpid(process_id, &status, 0) < 0) {
      std::exit(EXIT_FAILURE);
    }
  }
}
}  // namespace record
}  // namespace openscenario_interpreter
