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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/file_header.hpp>
#include <openscenario_interpreter/syntax/open_scenario_category.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- OpenScenario -----------------------------------------------------------
 *
 *  <xsd:complexType name="OpenScenario">
 *    <xsd:sequence>
 *      <xsd:element name="FileHeader" type="FileHeader"/>
 *      <xsd:group ref="OpenScenarioCategory"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct OpenScenario : public Scope
{
  pugi::xml_document script;

  const FileHeader file_header;

  const OpenScenarioCategory category;

  std::size_t frame;

  explicit OpenScenario(const boost::filesystem::path &);

  auto complete() const -> bool;

  auto evaluate() -> Object;

  auto load(const boost::filesystem::path &) -> const pugi::xml_node &;
};

auto operator<<(nlohmann::json &, const OpenScenario &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_
