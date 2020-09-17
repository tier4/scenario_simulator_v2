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

#ifndef SCENARIO_RUNNER__SYNTAX__ENTITY_REF_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITY_REF_HPP_

#include <scenario_runner/reader/attribute.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== EntityRef ============================================================
 *
 * <xsd:complexType name="EntityRef">
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct EntityRef
  : public String
{
  template<typename ... Ts>
  explicit constexpr EntityRef(Ts && ... xs)
  : String{std::forward<decltype(xs)>(xs)...}
  {}

  template<typename Node, typename Scope>
  explicit EntityRef(const Node & node, Scope & scope)
  : String{readAttribute<String>(node, scope, "entityRef")}
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_REF_HPP_
