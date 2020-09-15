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

#ifndef SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_
#define SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_

#include <scenario_runner/validator/attribute.hpp>
#include <scenario_runner/validator/choice.hpp>
#include <scenario_runner/validator/sequence.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== ObjectController =====================================================
 *
 * <xsd:complexType name="ObjectController">
 *   <xsd:choice>
 *     <xsd:element name="CatalogReference" type="CatalogReference"/>
 *     <xsd:element name="Controller" type="Controller"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct ObjectController
  : public Choice
{
  template<typename ... Ts>
  explicit ObjectController(Ts && ... xs)
  {
    defineElementAsUnsupported("CatalogReference", 0, 1);
    defineElementAsUnimplemented("Controller", 0, 1);

    validate(std::forward<decltype(xs)>(xs)...);
  }

  auto evaluate() const noexcept
  {
    return unspecified;
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__OBJECT_CONTROLLER_HPP_
