// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__READER__ELEMENT_HPP_
#define SCENARIO_RUNNER__READER__ELEMENT_HPP_

#include <scenario_runner/iterator/size.hpp>
#include <scenario_runner/object.hpp>
#include <scenario_runner/type_traits/if_not_default_constructible.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

namespace scenario_runner
{
inline namespace reader
{
constexpr auto unbounded {
  std::numeric_limits<
    typename std::iterator_traits<
      typename pugi::xml_node::iterator
    >::difference_type
  >::max()
};

template<typename T, typename Node, typename ... Ts>
auto readElement(const std::string & name, const Node & parent, Ts && ... xs)
{
  if (const auto child {parent.child(name.c_str())}) {
    return T {child, std::forward<decltype(xs)>(xs)...};
  } else {
    return IfNotDefaultConstructible<T>::error(parent.name(), name);
  }
}

template<typename Node, typename Callee>
void callWithElements(
  const Node & parent,
  const std::string & name,
  typename std::iterator_traits<typename Node::iterator>::difference_type min_occurs,
  typename std::iterator_traits<typename Node::iterator>::difference_type max_occurs,
  Callee && call_with)
{
  const auto children {parent.children(name.c_str())};

  if (const auto size {iterator::size(children)}) {
    if (min_occurs != 0 && size < min_occurs) {
      std::stringstream ss {};
      ss << parent.name() << " requires class " << name <<
        " at least " << min_occurs << " element" << (1 < min_occurs ? "s" : "") <<
        ", but " << size << " element" << (1 < size ? "s" : "") << " specified";
      throw SyntaxError(ss.str());
    } else if (max_occurs < size) {
      std::stringstream ss {};
      ss << parent.name() << " requires class " << name <<
        " at most " << max_occurs << " element" << (1 < max_occurs ? "s" : "") <<
        ", but " << size << " element" << (1 < size ? "s" : "") << " specified";
      throw SyntaxError(ss.str());
    } else {
      for (const auto & child : children) {
        call_with(child);
      }
    }
  } else if (min_occurs != 0) {
    std::stringstream ss {};
    ss << parent.name() << " requires class " << name <<
      " at least " << min_occurs << " element" << (1 < min_occurs ? "s" : "") <<
      ", but there is no specification";
    throw SyntaxError(ss.str());
  }
}

template<typename Node, typename ... Ts>
decltype(auto) choice(const Node & node, Ts && ... xs)
{
  const std::unordered_map<
    std::string,
    std::function<Element(const Node &)>>
  callees
  {
    std::forward<decltype(xs)>(xs)...
  };

  std::unordered_map<std::string, Node> specs {};

  for (const auto & each : callees) {
    if (const auto child {node.child(std::get<0>(each).c_str())}) {
      specs.emplace(std::get<0>(each), child);
    }
  }

  auto print_keys_to = [&](auto & os, const auto & xs) -> decltype(auto)
  {
    if (!xs.empty()) {
      for (auto iter {std::begin(xs)}; iter != std::end(xs); ++iter) {
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
    std::stringstream ss {};

    ss << "Class " << node.name() << " requires one of following elements: ";

    print_keys_to(ss, callees);

    ss << ". But no element specified";

    throw SyntaxError(ss.str());
  } else if (1 < specs.size()) {
    std::stringstream ss {};

    ss << "Class " << node.name() << " requires just one of following elements: ";

    print_keys_to(ss, callees);

    ss << ". But " << specs.size() << " element" << (1 < specs.size() ? "s" : "") << " (";

    print_keys_to(ss, specs);

    ss << ") specified";

    throw SyntaxError(ss.str());
  } else {
    const auto iter {std::cbegin(specs)};
    return callees.at(std::get<0>(*iter))(std::get<1>(*iter));
  }
}

template<typename Callee>
decltype(auto) callWithElement(const pugi::xml_node & parent, const std::string & name,
  Callee && call_with)
{
  return callWithElements(parent, name, 1, 1, std::forward<decltype(call_with)>(call_with));
}

  #define THROW_UNSUPPORTED_ERROR(PARENT) \
  [&](auto && child) \
  { \
    std::stringstream ss {}; \
    ss << "given class \'" << child.name() << "\' (element of class \'" << PARENT.name() << \
      "\') is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError(ss.str()); \
  }
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__READER__ELEMENT_HPP_
