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

auto DistanceCondition::distance(const EntityRef & triggering_entity) const -> double
{
  // clang-format off
  switch (coordinate_system) {
    case CoordinateSystem::entity:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::longitudinal,      true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::longitudinal,      false >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::lateral,           true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::lateral,           false >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::euclidianDistance, true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::euclidianDistance, false >(triggering_entity);
      }
      break;

    case CoordinateSystem::lane:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::longitudinal,      true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::longitudinal,      false >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::lateral,           true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::lateral,           false >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::euclidianDistance, true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::euclidianDistance, false >(triggering_entity);
      }
      break;

    case CoordinateSystem::road:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::longitudinal,      true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::longitudinal,      false >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::lateral,           true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::lateral,           false >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::euclidianDistance, true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::euclidianDistance, false >(triggering_entity);
      }
      break;

    case CoordinateSystem::trajectory:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::longitudinal,      true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::longitudinal,      false >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::lateral,           true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::lateral,           false >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::euclidianDistance, true  >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::euclidianDistance, false >(triggering_entity);
      }
      break;
  }
  // clang-format on

  throw Error(__FILE__, ":", __LINE__);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, false>(const EntityRef & entity_ref) const -> double
{
  const auto pose = getRelativePose(entity_ref, static_cast<geometry_msgs::msg::Pose>(position));
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
