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

#ifndef OPENSCENARIO_INTERPRETER__READER__ELEMENT_HPP_
#define OPENSCENARIO_INTERPRETER__READER__ELEMENT_HPP_

#include <functional>
#include <iterator>
#include <limits>
#include <openscenario_interpreter/iterator/size.hpp>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/type_traits/must_be_default_constructible.hpp>
#include <openscenario_interpreter/utility/pugi_extension.hpp>
#include <string>
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
using XML = pugi::xml_node;

using Cardinality =
  typename std::iterator_traits<typename pugi::xml_node::iterator>::difference_type;

constexpr auto unbounded = std::numeric_limits<Cardinality>::max();

template <typename Callee>
void callWithElements(
  const XML & parent, const std::string & name, const Cardinality min_occurs,
  const Cardinality max_occurs, Callee && call_with)
{
  const auto children = parent.children(name.c_str());

  if (const auto size = iterator::size(children)) {
    if (min_occurs != 0 and size < min_occurs) {
      throw SyntaxError(
        parent.name(), " requires class ", name, " at least ", min_occurs, " element",
        (1 < min_occurs ? "s" : ""), ", but ", size, " element", (1 < size ? "s" : ""),
        " specified");
    } else if (max_occurs < size) {
      throw SyntaxError(
        parent.name(), " requires class ", name, " at most ", max_occurs, " element",
        (1 < max_occurs ? "s" : ""), ", but ", size, " element", (1 < size ? "s" : ""),
        " specified");
    } else {
      for (const auto & child : children) {
        call_with(child);
      }
    }
  } else if (min_occurs != 0) {
    throw SyntaxError(
      parent.name(), " requires class ", name, " at least ", min_occurs, " element",
      (1 < min_occurs ? "s" : ""), ", but there is no specification");
  }
}

template <typename Callee>
[[deprecated]] decltype(auto) callWithElement(
  const XML & parent, const std::string & name, Callee && call_with)
{
  return callWithElements(parent, name, 1, 1, std::forward<decltype(call_with)>(call_with));
}

template <typename T, typename... Ts>
auto readElement(const std::string & name, const XML & parent, Ts &&... xs)
{
  if (const auto child = parent.child(name.c_str())) {
    return T(child, std::forward<decltype(xs)>(xs)...);
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

template <typename T, Cardinality MinOccurs, Cardinality MaxOccurs = unbounded, typename... Ts>
auto readElements(const std::string & name, const XML & node, Ts &&... xs)
{
  std::list<T> elements;

  callWithElements(node, name, MinOccurs, MaxOccurs, [&](auto && x) {
    elements.emplace_back(std::forward<decltype(x)>(x), std::forward<decltype(xs)>(xs)...);
  });

  return elements;
}

template <typename... Ts>
decltype(auto) choice(const XML & node, Ts &&... xs)
{
  const std::unordered_map<std::string, std::function<Element(const XML &)>> callees{
    std::forward<decltype(xs)>(xs)...};

  std::unordered_map<std::string, XML> specs{};

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
