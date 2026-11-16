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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HPP_

#include <filesystem>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- File -------------------------------------------------------------------
 *
 *  <xsd:complexType name="File">
 *    <xsd:attribute name="filepath" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct File
{
  const std::filesystem::path filepath;

  explicit File();

  explicit File(const std::string &);

  explicit File(const pugi::xml_node &, Scope &);

  auto isDirectory() const -> bool;

  operator std::filesystem::path() const;

  operator String() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HPP_
