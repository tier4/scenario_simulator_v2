// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_

#include <openscenario_interpreter/scope.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== Directory ============================================================
 *
 * <xsd:complexType name="Directory">
 *   <xsd:attribute name="path" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Directory
{
  const String path;

  template<typename Node, typename Scope>
  explicit Directory(const Node & node, Scope & outer_scope)
  : path{readAttribute<String>("path", node, outer_scope)}
  {}
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_
