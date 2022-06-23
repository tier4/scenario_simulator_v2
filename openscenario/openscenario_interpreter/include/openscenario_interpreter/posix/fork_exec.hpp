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

#ifndef OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_
#define OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_

#include <unistd.h>  // pid_t

#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace posix
{
int execvp(const std::vector<std::string> &);

pid_t fork_exec(const std::vector<std::string> &);

pid_t fork_exec(const std::string &);

pid_t fork_exec(const std::string &, const std::string &);
}  // namespace posix
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__POSIX__FORK_EXEC_HPP_
