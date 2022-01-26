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
#include <openscenario_interpreter/syntax/storyboard_element_state_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
StoryboardElementStateCondition::StoryboardElementStateCondition(
  const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  storyboard_element_ref(readAttribute<String>("storyboardElementRef", node, local())),
  storyboard_element_type(
    readAttribute<StoryboardElementType>("storyboardElementType", node, local())),
  state(readAttribute<StoryboardElementState>("state", node, local())),
  result(StoryboardElementState::standbyState)
{
}

auto StoryboardElementStateCondition::description() const -> String
{
  std::stringstream description;

  description << "The state of StoryboardElement " << std::quoted(storyboard_element_ref)
              << " (= " << result << ") is given state " << state << "?";

  return description.str();
}

auto StoryboardElementStateCondition::evaluate() -> Object
{
  try {
    result = local().ref(storyboard_element_ref).currentState().as<StoryboardElementState>();
    return asBoolean(result == state);
  } catch (const std::out_of_range &) {
    return false_v;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
