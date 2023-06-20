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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/longitudinal_action.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
LongitudinalAction::LongitudinalAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair(               "SpeedAction", [&](const auto & node) { return make<SpeedAction>(node, scope); }),
      std::make_pair("LongitudinalDistanceAction", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(        "SpeedProfileAction", [&](const auto & node) { return make<SpeedProfileAction>(node, scope); })))
// clang-format on
{
  {
    // OpenSCENARIO 1.2 Table 11
    auto constraint = [&](auto actor) {
      auto objects = scope.global().entities->objects({actor});
      auto is_vehicle = [&](auto object) {
        return scope.global().entities->ref(object).template is_also<Vehicle>();
      };
      auto is_pedestrian = [&](auto object) {
        return scope.global().entities->ref(object).template is_also<Pedestrian>();
      };
      return std::all_of(std::begin(objects), std::end(objects), is_vehicle) ||
             std::all_of(std::begin(objects), std::end(objects), is_pedestrian);
    };
    if (not std::all_of(std::begin(scope.actors), std::end(scope.actors), constraint)) {
      THROW_SEMANTIC_ERROR(
        "Actors may be either of vehicle type or a pedestrian type;"
        "See OpenSCENARIO 1.2 Table 11 for more details");
    }
  }
}

auto LongitudinalAction::endsImmediately() const -> bool
{
  return apply<bool>([](const auto & action) { return action.endsImmediately(); }, *this);
}

auto LongitudinalAction::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto LongitudinalAction::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
