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

#ifndef SCENARIO_RUNNER__VALIDATOR__ALL_HPP_
#define SCENARIO_RUNNER__VALIDATOR__ALL_HPP_

#include <scenario_runner/validator/element.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace validator
{
struct All
  : public Elements
{
  template<typename ... Ts>
  explicit All(Ts && ... xs)
  : Elements{std::forward<decltype(xs)>(xs)...}
  {}

  Object validate(const pugi::xml_node & node, Scope & scope)
  {
    for (const auto & each : node.children()) {
      auto iter {find(each.name())};

      if (iter != std::end(*this)) {
        std::get<1>(*iter).emplace_back(each, scope);
      } else {
        std::stringstream ss {};
        ss << "unknown class \'" << each.name() << "\' specified for property of class " <<
          node.name();
        throw SyntaxError {ss.str()};
      }
    }

    for (const auto & each : *this) {
      std::stringstream ss {};
      ss << "class " << node.name() << " requires class " << each.first;
      each.second.validate(ss.str());
    }

    return unspecified;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__VALIDATOR__ALL_HPP_
