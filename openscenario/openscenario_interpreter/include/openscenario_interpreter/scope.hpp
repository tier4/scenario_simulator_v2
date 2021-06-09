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

#ifndef OPENSCENARIO_INTERPRETER__SCOPE_HPP_
#define OPENSCENARIO_INTERPRETER__SCOPE_HPP_

#include <boost/filesystem.hpp>
#include <limits>
#include <memory>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
struct Scope
{
  std::unordered_map<String, Element> parameters;
  std::unordered_map<String, Element> entities;
  std::unordered_map<String, Element> storyboard_elements;

  using Actor = EntityRef;

  using Actors = std::list<Actor>;

  Actors actors;

  boost::filesystem::path logic_file;
  boost::filesystem::path scene_graph_file;

  // for substitution syntax '$(dirname)'
  const boost::filesystem::path scenario;

  Scope() = delete;

  explicit Scope(Scope &) = default;
  explicit Scope(const Scope &) = default;

  template <typename... Ts>
  explicit Scope(const std::string & scenario, Ts &&... xs) : scenario(scenario)
  {
  }
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
