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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/orientation.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/range.hpp>

namespace openscenario_interpreter
{
inline namespace reader
{
template auto readElement(const std::string &, const pugi::xml_node &, Scope &)
  -> syntax::Orientation;
template auto readElement(const std::string &, const pugi::xml_node &, Scope &) -> syntax::Position;
template auto readElement(const std::string &, const pugi::xml_node &, Scope &)
  -> syntax::Properties;
template auto readElement(const std::string &, const pugi::xml_node &, Scope &) -> syntax::Range;

auto choice(
  const pugi::xml_node & node,
  const std::unordered_map<std::string, std::function<Object(const pugi::xml_node &)>> & callees) -> Object
{
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
