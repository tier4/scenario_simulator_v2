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

#include <cmath>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <sstream>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto DistanceCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << "'s distance to given position = ";

  print_to(description, results);

  description << " " << rule << " " << value << "?";

  return description.str();
}

#define DISTANCE(...) distance<__VA_ARGS__>(triggering_entity)

#define SWITCH_FREESPACE(FUNCTION, ...) \
  return freespace ? FUNCTION(__VA_ARGS__, true) : FUNCTION(__VA_ARGS__, false)

#define SWITCH_RELATIVE_DISTANCE_TYPE(FUNCTION, ...)                  \
  switch (relative_distance_type) {                                   \
    case RelativeDistanceType::longitudinal:                          \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::longitudinal);      \
      break;                                                          \
    case RelativeDistanceType::lateral:                               \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::lateral);           \
      break;                                                          \
    case RelativeDistanceType::euclidianDistance:                     \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::euclidianDistance); \
      break;                                                          \
  }

#define SWITCH_COORDINATE_SYSTEM(FUNCTION, ...)            \
  switch (coordinate_system) {                             \
    case CoordinateSystem::entity:                         \
      FUNCTION(__VA_ARGS__, CoordinateSystem::entity);     \
      break;                                               \
    case CoordinateSystem::lane:                           \
      FUNCTION(__VA_ARGS__, CoordinateSystem::lane);       \
      break;                                               \
    case CoordinateSystem::road:                           \
      FUNCTION(__VA_ARGS__, CoordinateSystem::road);       \
      break;                                               \
    case CoordinateSystem::trajectory:                     \
      FUNCTION(__VA_ARGS__, CoordinateSystem::trajectory); \
      break;                                               \
  }

#define APPLY(F, ...) F(__VA_ARGS__)

auto DistanceCondition::distance(const EntityRef & triggering_entity) const -> double
{
  APPLY(SWITCH_COORDINATE_SYSTEM, SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_FREESPACE, DISTANCE);
  throw Error(__FILE__, ":", __LINE__);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, false>(
  const EntityRef & triggering_entity) const -> double
{
  const auto pose =
    getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
  return std::hypot(pose.position.x, pose.position.y);
}

auto DistanceCondition::evaluate() -> Element
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(distance(triggering_entity));
    return rule(results.back(), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
