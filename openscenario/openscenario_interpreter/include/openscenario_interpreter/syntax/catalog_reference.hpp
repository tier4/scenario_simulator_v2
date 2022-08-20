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

#include <openscenario_interpreter/functional/fold.hpp>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/catalog.hpp>
#include <openscenario_interpreter/syntax/directory.hpp>
#include <openscenario_interpreter/syntax/parameter_assignments.hpp>
#include <openscenario_interpreter/utility/print.hpp>

#include <boost/filesystem.hpp>

#include <optional>

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

//template <typename T>
class CatalogReference
{
public:
  CatalogReference(const pugi::xml_node & node, Scope & scope)
  : scope(scope),
    node(node),
    catalog_name(readAttribute<std::string>("catalogName", node, scope)),
    entry_name(readAttribute<std::string>("entryName", node, scope)),
    parameter_assignments(readElement<ParameterAssignments>("ParameterAssignments", node, scope))
  {
//    static_assert(
//      // clang-format off
//      std::is_same<Vehicle    , T>::value or
//      std::is_same<Controller , T>::value or
//      std::is_same<Pedestrian , T>::value or
//      std::is_same<MiscObject , T>::value or
////      std::is_same<Environment, T>::value or
//      std::is_same<Maneuver   , T>::value)
////      std::is_same<Trajectory , T>::value) or
////      std::is_same<Route      , T>::value)
//      // clang-format on
//      , "Catalog element type must be one of supported types"
//    );

    auto catalog_locations = scope.global().catalog_locations;
    if (catalog_locations) {
      for (auto & p : *catalog_locations) {
        auto & catalog_location = p.second;
        auto found_catalog = catalog_location.find(catalog_name);

        if (found_catalog != std::end(catalog_location)) {
          scope_by_catalog = found_catalog->second;
          break;
        }
      }
    }
    std::cout << "Catalog Reference" << std::endl;
  }

  Scope scope;  // anonymous namespace
  const pugi::xml_node node;
  const std::string catalog_name;
  const std::string entry_name;
  ParameterAssignments parameter_assignments;
  std::optional<pugi::xml_node> scope_by_catalog = std::nullopt;

  template <typename T>
  auto make(const pugi::xml_node & node_) -> Object
  {
//    auto scope_ = Scope("", scope);
    return openscenario_interpreter::make<T>(node_, this->scope);
  }
};

}  // namespace syntax

// template<typename T>
auto makeFromCatalogReference(const pugi::xml_node &, Scope &) -> const Object;

}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_REFERENCE_HPP_
