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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HEADER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HEADER_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/license.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- FileHeader -------------------------------------------------------------
 *
 *  <xsd:complexType name="FileHeader">
 *    <xsd:sequence>
 *      <xsd:element name="License" type="License" minOccurs="0"/>
 *      <xsd:element name="Properties" type="Properties" minOccurs="0"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="revMajor" type="UnsignedShort" use="required"/>
 *    <xsd:attribute name="revMinor" type="UnsignedShort" use="required"/>
 *    <xsd:attribute name="date" type="DateTime" use="required"/>
 *    <xsd:attribute name="description" type="String" use="required"/>
 *    <xsd:attribute name="author" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct FileHeader
{
  const License license;

  const Properties properties;

  const UnsignedShort revMajor;

  const UnsignedShort revMinor;

  const String date;

  const String description;

  const String author;

  explicit FileHeader(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FILE_HEADER_HPP_
