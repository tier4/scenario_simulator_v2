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

#ifndef OPENSCENARIO_INTERPRETER__READER__CONTENT_HPP_
#define OPENSCENARIO_INTERPRETER__READER__CONTENT_HPP_

#include <boost/algorithm/string/trim.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/pugi_extension.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace reader
{
template <typename T, typename Node, typename Scope>
T readContent(const Node & node, const Scope &)
{
  const std::string text{node.text().get()};
  return boost::lexical_cast<T>(boost::algorithm::trim_copy(text));
}
}  // namespace reader
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__CONTENT_HPP_
