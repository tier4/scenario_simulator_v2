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

#ifndef OPENSCENARIO_INTERPRETER__READER__ELEMENT_HPP_
#define OPENSCENARIO_INTERPRETER__READER__ELEMENT_HPP_

#include <functional>
#include <iterator>
#include <limits>
#include <openscenario_interpreter/iterator/size.hpp>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/type_traits/must_be_default_constructible.hpp>
#include <pugixml.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

/* ---- NOTE -------------------------------------------------------------------
 *
 *  The functions in the reader header only deal with XML parsing and should
 *  not have any prerequisite knowledge about OpenSCENARIO data structures.
 *
 * -------------------------------------------------------------------------- */

namespace openscenario_interpreter
{
inline namespace reader
{
using Cardinality =
  typename std::iterator_traits<typename pugi::xml_node::iterator>::difference_type;

constexpr auto unbounded = std::numeric_limits<Cardinality>::max();

template <Cardinality MinOccurs, Cardinality MaxOccurs, typename F>
auto traverse(const pugi::xml_node & parent, const std::string & name, F && f) -> void
{
  const auto children = parent.children(name.c_str());

  if (const auto size = iterator::size(children)) {
    if (MinOccurs != 0 and size < MinOccurs) {
      throw SyntaxError(
        parent.name(), " requires class ", name, " at least ", MinOccurs, " element",
        (1 < MinOccurs ? "s" : ""), ", but ", size, " element", (1 < size ? "s" : ""),
        " specified");
    } else if (MaxOccurs < size) {
      throw SyntaxError(
        parent.name(), " requires class ", name, " at most ", MaxOccurs, " element",
        (1 < MaxOccurs ? "s" : ""), ", but ", size, " element", (1 < size ? "s" : ""),
        " specified");
    } else {
      for (const auto & child : children) {
        f(child);
      }
    }
  } else if (MinOccurs != 0) {
    throw SyntaxError(
      parent.name(), " requires class ", name, " at least ", MinOccurs, " element",
      (1 < MinOccurs ? "s" : ""), ", but there is no specification");
  }
}

template <typename T, typename Scope>
auto readElement(const std::string & name, const pugi::xml_node & parent, Scope & scope)
{
  if (const auto child = parent.child(name.c_str())) {
    return T(child, scope);
  } else {
    /* ---- NOTE ---------------------------------------------------------------
     *
     *  If the given XML node does not have a child element (T type) with the
     *  specified name, it assumes that the T type is an optional element and
     *  attempts to build a default T type.
     *
     * ---------------------------------------------------------------------- */
    return MustBeDefaultConstructible<T>::makeItOrThrow(SyntaxError(
      parent.name(), " requires class ", name, " as element, but there is no declaration"));
  }
}

template <typename T, typename U, typename Scope>
auto readElement(const std::string & name, const pugi::xml_node & parent, Scope & scope, U && value)
{
  if constexpr (std::is_same<T, typename std::decay<U>::type>::value) {
    // use 'value' as a default element.
    // the default element will be used when current scope has no description about 'name'.
    if (const auto child = parent.child(name.c_str())) {
      return T(child, scope);
    } else {
      return value;
    }
  } else {
    // use "value" as an additional arguments to the constructor
    if (const auto child = parent.child(name.c_str())) {
      return T(child, scope, value);
    } else {
      return MustBeDefaultConstructible<T>::makeItOrThrow(SyntaxError(
        parent.name(), " requires class ", name, " as element, but there is no declaration"));
    }
  }
}

template <typename T, Cardinality MinOccurs, Cardinality MaxOccurs = unbounded, typename... Ts>
auto readElements(const std::string & name, const pugi::xml_node & node, Ts &&... xs)
{
  std::list<T> elements;

  traverse<MinOccurs, MaxOccurs>(node, name, [&](auto && x) {
    elements.emplace_back(std::forward<decltype(x)>(x), std::forward<decltype(xs)>(xs)...);
  });

  return elements;
}

template <typename GroupT, Cardinality MinOccurs, Cardinality MaxOccurs = unbounded, typename... Ts>
auto readGroups(const pugi::xml_node & node, Ts &&... xs)
{
  std::list<GroupT> groups;

  for (auto child = node.begin(); child != node.end(); ++child) {
    try {
      groups.template emplace_back(
        std::forward<decltype(*child)>(*child), std::forward<decltype(xs)>(xs)...);
    } catch (...) {
    }
  }

  if (MinOccurs != 0 and groups.size() < MinOccurs) {
    throw SyntaxError(
      node.name(), " requires Group ", demangle(typeid(GroupT)), " at least ", MinOccurs,
      " element", (1 < MinOccurs ? "s" : ""), ", but ", groups.size(), " element",
      (1 < groups.size() ? "s" : ""), " specified");
  }

  if (MaxOccurs < groups.size()) {
    throw SyntaxError(
      node.name(), " requires Group ", demangle(typeid(GroupT)), " at most ", MaxOccurs, " element",
      (1 < MaxOccurs ? "s" : ""), ", but ", groups.size(), " element",
      (1 < groups.size() ? "s" : ""), " specified");
  }

  return groups;
}

template <typename... Ts>
auto choice(const pugi::xml_node & node, Ts &&... xs) -> decltype(auto)
{
  const std::unordered_map<std::string, std::function<Object(const pugi::xml_node &)>> callees{
    std::forward<decltype(xs)>(xs)...};

  std::unordered_map<std::string, pugi::xml_node> specs{};

  for (const auto & each : callees) {
    if (const auto child = node.child(std::get<0>(each).c_str())) {
      specs.emplace(std::get<0>(each), child);
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
    what << "Class " << node.name() << " requires one of following elements: ";
    print_keys_to(what, callees);
    what << ". But no element specified";
    throw SyntaxError(what.str());
  } else if (1 < specs.size()) {
    std::stringstream what;
    what << "Class " << node.name() << " requires just one of following elements: ";
    print_keys_to(what, callees);
    what << ". But " << specs.size() << " element" << (1 < specs.size() ? "s" : "") << " (";
    print_keys_to(what, specs);
    what << ") specified";
    throw SyntaxError(what.str());
  } else {
    const auto iter = std::cbegin(specs);
    return callees.at(std::get<0>(*iter))(std::get<1>(*iter));
  }
}
}  // namespace reader
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__ELEMENT_HPP_
