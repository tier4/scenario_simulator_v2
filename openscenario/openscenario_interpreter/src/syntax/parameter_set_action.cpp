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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/parameter_set_action.hpp>
#include <typeindex>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterSetAction::ParameterSetAction(
  const pugi::xml_node & node, Scope & scope, const String & parameter_ref)
: Scope(scope), parameter_ref(parameter_ref), value(readAttribute<String>("value", node, local()))
{
}

auto ParameterSetAction::accomplished() noexcept -> bool  //
{
  return true;
}

auto ParameterSetAction::run() noexcept -> void {}

auto ParameterSetAction::set(
  const Scope & scope, const String & parameter_ref, const String & value) -> void
{
  static const std::unordered_map<
    std::type_index, std::function<void(const Object &, const String &)>>
    overloads{
      // clang-format off
      { typeid(Boolean),         [](const Object & parameter, const auto & value) { parameter.as<Boolean        >() = boost::lexical_cast<Boolean        >(value); } },
      { typeid(Double),          [](const Object & parameter, const auto & value) { parameter.as<Double         >() = boost::lexical_cast<Double         >(value); } },
      { typeid(Integer),         [](const Object & parameter, const auto & value) { parameter.as<Integer        >() = boost::lexical_cast<Integer        >(value); } },
      { typeid(String),          [](const Object & parameter, const auto & value) { parameter.as<String         >() =                                      value ; } },
      { typeid(UnsignedInteger), [](const Object & parameter, const auto & value) { parameter.as<UnsignedInteger>() = boost::lexical_cast<UnsignedInteger>(value); } },
      { typeid(UnsignedShort),   [](const Object & parameter, const auto & value) { parameter.as<UnsignedShort  >() = boost::lexical_cast<UnsignedShort  >(value); } },
      // clang-format on
    };

  const auto parameter = scope.ref(parameter_ref);

  overloads.at(parameter.type())(parameter, value);
}

auto ParameterSetAction::start() const -> void  //
{
  set(local(), parameter_ref, value);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
