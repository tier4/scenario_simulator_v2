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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BY_OBJECT_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BY_OBJECT_TYPE_HPP_

#include <algorithm>
#include <iterator>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <type_traits>
#include <valarray>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   OpenSCENARIO XML 1.3 ByObjectType

   Defines an object type to select entities.

   <xsd:complexType name="ByObjectType">
     <xsd:attribute name="type" type="ObjectType" use="required"/>
   </xsd:complexType>
*/
struct ByObjectType : public Scope
{
  ObjectType type;

  explicit ByObjectType(const pugi::xml_node &, Scope &);

  auto objects() const -> std::set<Entity>;

  template <typename Function>
  auto apply(const Function & function) const
  {
    using Result = std::invoke_result_t<Function, Entity>;
    auto objects = this->objects();
    if constexpr (std::is_same_v<Result, void>) {
      std::for_each(std::begin(objects), std::end(objects), function);
    } else {
      auto results = std::valarray<Result>(objects.size());
      std::transform(std::begin(objects), std::end(objects), std::begin(results), function);
      return results;
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BY_OBJECT_TYPE_HPP_
