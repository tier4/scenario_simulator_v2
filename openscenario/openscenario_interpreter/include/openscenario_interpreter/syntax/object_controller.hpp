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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_

#include <openscenario_interpreter/object.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ObjectController -------------------------------------------------------
 *
 *  Definition of a controller for a scenario object. Either an inline
 *  definition or a catalog reference to a controller.
 *
 *  <xsd:complexType name="ObjectController">
 *    <xsd:choice>
 *      <xsd:element name="CatalogReference" type="CatalogReference"/>
 *      <xsd:element name="Controller" type="Controller"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ObjectController : public ComplexType
{
  explicit ObjectController();

  explicit ObjectController(const pugi::xml_node &, Scope &);

  auto isAutoware() const & -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_
