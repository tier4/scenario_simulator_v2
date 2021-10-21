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
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include "openscenario_interpreter/error.hpp"
#include "openscenario_interpreter/object.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
#undef UNSUPPORTED_ELEMENT_SPECIFIED
#define UNSUPPORTED_ELEMENT_SPECIFIED(ELEMENT)                 \
  SyntaxError(                                                 \
    "Given class ", ELEMENT,                                   \
    " is valid OpenSCENARIO element of class CatalogRefenrece" \
    ", but is not supported yet")

namespace
{
template <typename... Ts>
auto choice_by_attribute(const XML & node, const std::string & attribute, Ts &&... xs)
{
  using CalleeT = std::function<Element(const XML &)>;

  const std::unordered_map<std::string, CalleeT> callees{
    {std::forward<decltype(xs)>(xs)...}, sizeof...(Ts)};

  std::vector<std::pair<XML, CalleeT>> specs;

  for (auto && child : node.children()) {
    auto it = callees.find(child.attribute(attribute.c_str()).as_string());
    if (it != callees.end()) {
      specs.emplace_back(child, it->second);
    }
  }

  auto print_keys_to = [&](auto & os, const auto & xs) -> decltype(auto) {
    if (not xs.empty()) {
      for (auto iter = std::begin(xs); iter != std::end(xs); ++iter) {
        os << std::get<0>(*iter);
        switch (std::distance(iter, std::end(xs))) {
          case 1:
            return os;
          case 2:
            os << " and ";
            break;
          default:
            os << ", ";
            break;
        }
      }
    }
    return os;
  };

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
struct ScopeHolder : Derived
{
  Scope scope;

  ScopeHolder(const pugi::xml_node & node, Scope & scope)  //
  : Derived(node, scope), scope(scope)
  {
  }
};

template <typename T, typename... Args>
Element make_element(Args &&... args)
{
  return Element::bind_as<T, ScopeHolder<T>>(std::forward<Args>(args)...);
}
}  // namespace

Element CatalogReference::make(const pugi::xml_node & node, Scope & outer_scope)
{
  auto catalog_name = readAttribute<std::string>("catalogName", node, outer_scope);
  auto entry_name = readAttribute<std::string>("entryName", node, outer_scope);

  auto scope = outer_scope.makeChildScope("");  // anonymous namespace

  auto parameter_assignments =
    readElement<ParameterAssignments>("ParameterAssignments", node, scope);

  auto catalog_locations = scope.global().catalog_locations;
  if (catalog_locations) {
    for (auto & [type, catalog_location] : *catalog_locations) {
      auto found_catalog = catalog_location.find(catalog_name);
      if (found_catalog != catalog_location.end()) {
        // clang-format off
        std::unordered_map<std::string, std::function<Element(const pugi::xml_node &)>> dispatcher = {
          std::make_pair("Vehicle",     [&scope](auto && node) { return make_element<Vehicle>   (node, scope); }),
          std::make_pair("Controller",  [&scope](auto && node) { return make_element<Controller>(node, scope); }),
          std::make_pair("Pedestrian",  [&scope](auto && node) { return make_element<Pedestrian>(node, scope); }),
          std::make_pair("MiscObject",  [&scope](auto && node) { return make_element<MiscObject>(node, scope); }),
          std::make_pair("Environment", [      ](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified;}),
          std::make_pair("Maneuver",    [&scope](auto && node) { return make_element<Maneuver>  (node, scope); }),
          std::make_pair("Trajectory",  [      ](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified;}),
          std::make_pair("Route",       [      ](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified;})
        };
        // clang-format on

        const auto & catalog_xml = found_catalog->second;
        return choice_by_attribute(
          catalog_xml, "name", std::make_pair(entry_name, [&](const pugi::xml_node & node) {
            auto it = dispatcher.find(node.name());
            if (it == dispatcher.end()) {
              std::stringstream what;
              what << "Catalog element must be one of following elements: ";
              for (auto & it : dispatcher) {
                what << it.first << ", ";
              }
              what << ". But no element specified";
              throw SyntaxError(what.str());
            }
            return it->second(node);
          }));
      }
    }
  }

  return Element{};
}

}  // namespace syntax
}  // namespace openscenario_interpreter
