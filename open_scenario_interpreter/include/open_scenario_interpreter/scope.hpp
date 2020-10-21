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

#ifndef OPEN_SCENARIO_INTERPRETER__SCOPE_HPP_
#define OPEN_SCENARIO_INTERPRETER__SCOPE_HPP_

#include <boost/filesystem.hpp>
#include <open_scenario_interpreter/syntax/entity_ref.hpp>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace open_scenario_interpreter
{
struct Scope
{
  std::unordered_map<String, Element> parameters, entities, storyboard_elements;

  std::vector<EntityRef> actors;

  // for substituation syntax '$(dirname)'
  const boost::filesystem::path scenario;

  Scope() = delete;

  explicit Scope(Scope &) = default;
  explicit Scope(const Scope &) = default;

  template<typename ... Ts>
  explicit Scope(const std::string & scenario, Ts && ... xs)
  : scenario(scenario)
  {}
};
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SCOPE_HPP_
