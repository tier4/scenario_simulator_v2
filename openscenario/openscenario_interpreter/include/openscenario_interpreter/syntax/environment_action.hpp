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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENVIRONMENT_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENVIRONMENT_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EnvironmentAction (OpenSCENARIO V1.2.0) -----------------------------
 *
 * <xsd:complexType name="EnvironmentAction">
 *   <xsd:choice>
 *     <xsd:element name="Environment" type="Environment" minOccurs="0"/>
 *     <xsd:element name="CatalogReference" type="CatalogReference" minOccurs="0"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct EnvironmentAction : public ComplexType
{
  explicit EnvironmentAction(const pugi::xml_node &, Scope &);

  static auto accomplished() noexcept -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() -> void;

  /*  */ auto start() -> void;
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENVIRONMENT_ACTION_HPP_
