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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<StoryboardElementState>::value, "");

static_assert(std::is_trivial<StoryboardElementState>::value, "");

auto operator>>(std::istream & is, StoryboardElementState & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                       \
  if (buffer == #IDENTIFIER) {                        \
    datum.value = StoryboardElementState::IDENTIFIER; \
    return is;                                        \
  }                                                   \
  static_assert(true, "")

  BOILERPLATE(startTransition);
  BOILERPLATE(endTransition);
  BOILERPLATE(stopTransition);
  BOILERPLATE(skipTransition);
  BOILERPLATE(completeState);
  BOILERPLATE(runningState);
  BOILERPLATE(standbyState);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(StoryboardElementState, buffer);
}

auto operator<<(std::ostream & os, const StoryboardElementState & datum) -> std::ostream &
{
#define BOILERPLATE(ID)            \
  case StoryboardElementState::ID: \
    return os << #ID;

  switch (datum) {
    BOILERPLATE(startTransition);
    BOILERPLATE(endTransition);
    BOILERPLATE(stopTransition);
    BOILERPLATE(skipTransition);
    BOILERPLATE(completeState);
    BOILERPLATE(runningState);
    BOILERPLATE(standbyState);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(StoryboardElementState, datum);
  }

#undef BOILERPLATE
}

// clang-format off
const Object start_transition = make<StoryboardElementState>(StoryboardElementState::startTransition);
const Object   end_transition = make<StoryboardElementState>(StoryboardElementState::  endTransition);
const Object  stop_transition = make<StoryboardElementState>(StoryboardElementState:: stopTransition);
const Object  skip_transition = make<StoryboardElementState>(StoryboardElementState:: skipTransition);
const Object    standby_state = make<StoryboardElementState>(StoryboardElementState::   standbyState);
const Object    running_state = make<StoryboardElementState>(StoryboardElementState::   runningState);
const Object   complete_state = make<StoryboardElementState>(StoryboardElementState::  completeState);
// clang-format on
}  // namespace syntax
}  // namespace openscenario_interpreter
