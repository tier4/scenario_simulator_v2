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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_

#include <openscenario_interpreter/syntax/controller.hpp>
#include <utility>

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
  // inline static int ego_count= 0;
  static int ego_count;

  explicit ObjectController()  // ObjectController is optional element.
  : ComplexType(unspecified)
  {
  }

  template <typename Node, typename... Ts>
  explicit ObjectController(const Node & node, Ts &&... xs)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair("CatalogReference", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair("Controller",       [&](auto && node) { return make<Controller>(node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
    if (isEgo()) {
      ego_count++;
    }
  }

  ~ObjectController()
  {
    if (isEgo()) {
      ego_count--;
    }
  }

  bool isEgo() const &
  {
    if (is<Unspecified>()) {
      static auto controller = DefaultController();
      return controller["isEgo"];
    } else {
      return as<Controller>()["isEgo"];
    }
  }

  operator openscenario_msgs::msg::DriverModel() const
  {
    if (is<Unspecified>()) {
      openscenario_msgs::msg::DriverModel controller;
      {
        controller.see_around = !DefaultController()["isBlind"];
      }
      return controller;
    } else {
      return as<Controller>();
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_CONTROLLER_HPP_
