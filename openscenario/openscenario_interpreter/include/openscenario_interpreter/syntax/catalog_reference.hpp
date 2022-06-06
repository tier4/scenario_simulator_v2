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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_REFERENCE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_REFERENCE_HPP_

#include <boost/filesystem.hpp>
#include <openscenario_interpreter/functional/fold.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/catalog.hpp>
#include <openscenario_interpreter/syntax/directory.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- CatalogReference ------------------------------------------------------
 *
 *  <xsd:complexType name="CatalogReference">
 *    <xsd:sequence>
 *      <xsd:element name="ParameterAssignments" type="ParameterAssignments" minOccurs="0"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="catalogName" type="String" use="required"/>
 *    <xsd:attribute name="entryName" type="String" use="required"/>
 *  </xsd:complexType>
 * -------------------------------------------------------------------------- */

struct CatalogReference
{
  static auto make(const pugi::xml_node &, Scope &) -> Object;

  template <typename... Ts>
  static auto make(const pugi::xml_node & node, Scope & scope) -> Object
  {
    auto result = CatalogReference::make(node, scope);

    if (fold_right(std::logical_or<void>(), result.is_also<Ts>()...)) {
      return result;
    } else {
      std::stringstream what;
      what << "Required type of catalog element is one of the following type: ";
      print_to(what, std::array<const char *, sizeof...(Ts)>{typeid(Ts).name()...});
      what << ". But the type of this element is " << result.type().name() << ".";
      throw SyntaxError(what.str());
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_REFERENCE_HPP_
