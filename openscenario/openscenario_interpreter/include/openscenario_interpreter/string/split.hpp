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

#ifndef OPENSCENARIO_INTERPRETER__STRING__SPLIT_HPP_
#define OPENSCENARIO_INTERPRETER__STRING__SPLIT_HPP_

#include <boost/algorithm/string.hpp>

#include <string>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace string
{
auto split(const std::string & target)
{
  std::vector<std::string> result {};
  boost::split(result, target, boost::is_space());
  return result;
}
}  // namespace string
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__STRING__SPLIT_HPP_
