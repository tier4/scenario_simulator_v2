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

#ifndef OPENSCENARIO_INTERPRETER__READER__NAMEREF_HPP_
#define OPENSCENARIO_INTERPRETER__READER__NAMEREF_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace reader
{
template <typename Node, typename Scope, typename Candidates>
auto readNameRef(
  const std::string & name, const Node & node, const Scope & scope, const Candidates & candidates)
  -> String
{
  auto nameRef = readAttribute<String>(name, node, scope);
  if (auto referenced = std::find(std::begin(candidates), std::end(candidates), nameRef);
      referenced != std::end(candidates)) {
    return nameRef;
  } else {
    throw common::SyntaxError(
      "Reference to ", std::quoted(nameRef),
      " is invalid in this context; maybe it is undefined or forbidden");
  }
}
}  // namespace reader
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__NAMEREF_HPP_
