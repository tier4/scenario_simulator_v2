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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/arrow.hpp>
#include <openscenario_interpreter/syntax/color.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto TrafficSignalState::evaluate() const -> Element
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  `state: none` is valid for both Arrow / Color. That is, `state: none`
   *  changes both the arrow signal and the color signal to unlit at once.
   *
   * ------------------------------------------------------------------------ */

  const auto color = boost::lexical_cast<boost::optional<Color>>(state);
  if (color.has_value()) {
    setTrafficSignalColor(id(), color.value());
    return unspecified;
  }

  const auto arrow = boost::lexical_cast<boost::optional<Arrow>>(state);
  if (arrow.has_value()) {
    setTrafficSignalArrow(id(), arrow.value());
    return unspecified;
  }

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Color or Arrow, state);
}

auto TrafficSignalState::id() const -> LaneletId
{
  return boost::lexical_cast<LaneletId>(traffic_signal_id);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
