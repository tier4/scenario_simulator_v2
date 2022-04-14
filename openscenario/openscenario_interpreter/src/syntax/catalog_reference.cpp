// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/parameter_assignments.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
#define UNSUPPORTED_CATALOG_REFERENCE_SPECIFIED(ELEMENT)       \
  SyntaxError(                                                 \
    "Given class ", ELEMENT,                                   \
    " is valid OpenSCENARIO element of class CatalogRefenrece" \
    ", but is not supported yet")

template <typename... Ts>
auto choice_by_attribute(const pugi::xml_node & node, const std::string & attribute, Ts &&... xs)
{
  using CalleeT = std::function<Object(const pugi::xml_node &)>;

  const std::unordered_map<std::string, CalleeT> callees{
    {std::forward<decltype(xs)>(xs)...}, sizeof...(Ts)};

  std::vector<std::pair<pugi::xml_node, CalleeT>> specs;

  for (auto && child : node.children()) {
    auto iter = callees.find(child.attribute(attribute.c_str()).as_string());
    if (iter != std::end(callees)) {
      specs.emplace_back(child, iter->second);
    }
  }

  if (specs.empty()) {
    std::stringstream what;
    what << node.name() << " requires one of following " << std::quoted(attribute) << ": ";
    print_keys_to(what, callees);
    what << ". But no element specified";
    throw SyntaxError(what.str());
  }

  if (1 < specs.size()) {
    std::stringstream what;
    what << node.name() << " requires just one of following " << std::quoted(attribute) << ": ";
    print_keys_to(what, callees);
    what << ". But " << specs.size() << " element" << (1 < specs.size() ? "s" : "") << " (";
    print_keys_to(what, specs);
    what << ") specified";
    throw SyntaxError(what.str());
  }

  const auto iter = std::cbegin(specs);
  return iter->second(iter->first);
}

template <typename Derived>
struct CatalogInstance : public Derived
{
  Scope scope;

  ParameterAssignments parameter_assignments;

  CatalogInstance(
    const pugi::xml_node & node,                         //
    Scope & scope,                                       //
    const ParameterAssignments & parameter_assignments)  //
  : Derived(node, scope), scope(scope), parameter_assignments(parameter_assignments)
  {
  }
};

auto CatalogReference::make(const pugi::xml_node & node, Scope & outer_scope) -> Object
{
  auto catalog_name = readAttribute<std::string>("catalogName", node, outer_scope);

  auto entry_name = readAttribute<std::string>("entryName", node, outer_scope);

  auto scope = Scope("", outer_scope);  // anonymous namespace

  auto parameter_assignments =
    readElement<ParameterAssignments>("ParameterAssignments", node, scope);

  auto catalog_locations = scope.global().catalog_locations;

  if (catalog_locations) {
    for (auto & p : *catalog_locations) {
      auto & catalog_location = p.second;
      auto found_catalog = catalog_location.find(catalog_name);

      if (found_catalog != std::end(catalog_location)) {
        using ::openscenario_interpreter::make;

        std::unordered_map<std::string, std::function<Object(const pugi::xml_node &)>> dispatcher{
          // clang-format off
          std::make_pair("Vehicle",     [&](auto && node) { return make<CatalogInstance<Vehicle>>   (node, scope, parameter_assignments); }),
          std::make_pair("Controller",  [&](auto && node) { return make<CatalogInstance<Controller>>(node, scope, parameter_assignments); }),
          std::make_pair("Pedestrian",  [&](auto && node) { return make<CatalogInstance<Pedestrian>>(node, scope, parameter_assignments); }),
          std::make_pair("MiscObject",  [&](auto && node) { return make<CatalogInstance<MiscObject>>(node, scope, parameter_assignments); }),
          std::make_pair("Environment", [ ](auto && node) { throw UNSUPPORTED_CATALOG_REFERENCE_SPECIFIED(node.name()); return unspecified;}),
          std::make_pair("Maneuver",    [&](auto && node) { return make<CatalogInstance<Maneuver>>  (node, scope, parameter_assignments); }),
          std::make_pair("Trajectory",  [ ](auto && node) { throw UNSUPPORTED_CATALOG_REFERENCE_SPECIFIED(node.name()); return unspecified;}),
          std::make_pair("Route",       [ ](auto && node) { throw UNSUPPORTED_CATALOG_REFERENCE_SPECIFIED(node.name()); return unspecified;})
          // clang-format on
        };

        return choice_by_attribute(
          found_catalog->second, "name",
          std::make_pair(entry_name, [&](const pugi::xml_node & node) {
            auto iter = dispatcher.find(node.name());
            if (iter != std::end(dispatcher)) {
              return iter->second(node);
            } else {
              std::stringstream what;
              what << "Catalog element must be one of following elements: ";
              const auto * separator = "[";
              for (auto & each : dispatcher) {
                what << separator << each.first;
                separator = ", ";
              }
              what << "]. But no element specified.";
              throw SyntaxError(what.str());
            }
          }));
      }
    }
  }

  return unspecified;
}

}  // namespace syntax
}  // namespace openscenario_interpreter
