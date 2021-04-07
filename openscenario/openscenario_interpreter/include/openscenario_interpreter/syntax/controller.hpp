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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_

#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Controller -------------------------------------------------------------
 *
 *  Defines a controller type and parameters for the controller.
 *
 *  Used in:
 *    AssignControllerAction,
 *    Catalog
 *    ControllerDistributionEntry,
 *    ObjectController,
 *
 *  <xsd:complexType name="Controller">
 *    <xsd:all>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Properties" type="Properties"/>
 *    </xsd:all>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
using DefaultController = Properties;

struct Controller
{
  /* ---- name -----------------------------------------------------------------
   *
   *  Name of the controller type.
   *
   * ------------------------------------------------------------------------ */
  using Name = String;

  const Name name;

  Scope inner_scope;

  /* ---- ParameterDeclarations ------------------------------------------------
   *
   *  Definition of additional parameters.
   *
   * ------------------------------------------------------------------------ */
  const ParameterDeclarations parameter_declarations;

  /* ---- Properties -----------------------------------------------------------
   *
   *  Describing properties for the controller.
   *
   * ------------------------------------------------------------------------ */
  Properties properties;

  template <typename Node, typename Scope>
  explicit Controller(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    inner_scope(outer_scope),
    parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, inner_scope)),
    properties(readElement<Properties>("Properties", node, inner_scope))
  {
  }

  template <typename... Ts>
  decltype(auto) operator[](Ts &&... xs)
  {
    return properties.operator[](std::forward<decltype(xs)>(xs)...);
  }

  operator openscenario_msgs::msg::DriverModel()
  {
    openscenario_msgs::msg::DriverModel controller;
    {
      controller.see_around = !(*this)["isBlind"];
    }

    return controller;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
