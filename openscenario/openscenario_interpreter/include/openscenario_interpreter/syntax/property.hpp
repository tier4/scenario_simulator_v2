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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Property ---------------------------------------------------------------
 *
 *  <xsd:complexType name="Property">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Property
{
  const String name;

  const String value;

  /*
     The default construct is used to give the value of an unspecified
     property.

     Generally, this default constructor is called when an unspecified property
     name is specified in operator [] of std::unordered_map that holds the
     Property class.

     The default constructed property has an empty "value".
     The implicit cast operator of the Property class constructs the target
     type by default constructor if the string "value" is empty.

     Keep in mind that the C++ bool type has a value 'false' when it is
     initialized by default construction.
  */
  Property() = default;

  explicit Property(const pugi::xml_node &, Scope &);

  explicit operator bool() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_
