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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_

#include <filesystem>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
struct Scope;
inline namespace syntax
{
/* ---- Directory --------------------------------------------------------------
 *
 *  <xsd:complexType name="Directory">
 *    <xsd:attribute name="path" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Directory
{
  const std::filesystem::path path;

  explicit Directory(const pugi::xml_node &, Scope &);

  static auto ls(const Directory & dir)
  {
    using dir_it = std::filesystem::directory_iterator;
    return std::vector<std::filesystem::path>(dir_it(dir.path), dir_it());
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTORY_HPP_
