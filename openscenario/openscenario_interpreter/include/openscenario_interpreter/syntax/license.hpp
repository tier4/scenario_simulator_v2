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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LICENSE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LICENSE_HPP_

#include <iostream>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- License ----------------------------------------------------------------
 *
 * <xsd:complexType name="License">
 *   <xsd:simpleContent>
 *     <xsd:extension base="xsd:string">
 *       <xsd:attribute name="name" type="String" use="required"/>
 *       <xsd:attribute name="resource" type="String"/>
 *       <xsd:attribute name="spdxId" type="String"/>
 *     </xsd:extension>
 *   </xsd:simpleContent>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct License
{
  const String name;

  const String resource;

  const String spdx_id;

  const String text;

  License() = default;

  explicit License(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LICENSE_HPP_
