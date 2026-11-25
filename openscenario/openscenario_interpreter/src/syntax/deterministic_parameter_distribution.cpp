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
#include <openscenario_interpreter/syntax/deterministic_parameter_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/**
 * Note:
 *   Most Groups are used only once in one scope of OpenSCENARIO.
 *   And they can use `choice` function to load the group from the scenario, despite they don't have `xsd:choice` attribute.
 *   But, DeterministicParameterDistribution group can be used many times in same scope.
 *   That prevents from using `choice` function.
 *   Now, we defined new function `readGroup` instead of `choice`
 */
template <typename... Ts>
auto readGroup(const pugi::xml_node & node, Ts &&... xs)
{
  const std::unordered_map<std::string, std::function<Object(const pugi::xml_node &)>> callees{
    std::forward<decltype(xs)>(xs)...};

  std::unordered_map<std::string, pugi::xml_node> specs{};

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

  if (auto callee = callees.find(node.name()); callee != callees.end()) {
    return callee->second(node);
  } else {
    std::stringstream what;
    what << "Class " << node.name() << " requires one of following elements: ";
    print_keys_to(what, callees);
    what << ". But no element specified";
    std::cout << what.str() << std::endl;
    throw SyntaxError(what.str());
  }
}

DeterministicParameterDistribution::DeterministicParameterDistribution(
  const pugi::xml_node & tree, Scope & scope)
// clang-format off
: Group(
    readGroup(tree,
      std::make_pair("DeterministicMultiParameterDistribution",  [&](auto && node){return make<DeterministicMultiParameterDistribution >(node,scope);}),
      std::make_pair("DeterministicSingleParameterDistribution", [&](auto && node){return make<DeterministicSingleParameterDistribution>(node,scope);})))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
