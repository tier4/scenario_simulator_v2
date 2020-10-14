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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_

#include <open_scenario_interpreter/syntax/selected_entities.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== EntitySelection ======================================================
 *
 * <xsd:complexType name="EntitySelection">
 *   <xsd:sequence>
 *     <xsd:element name="Members" type="SelectedEntities"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct EntitySelection
{};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_
