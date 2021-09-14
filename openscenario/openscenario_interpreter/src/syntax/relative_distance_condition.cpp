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

#include <openscenario_interpreter/syntax/relative_distance_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, Between::reference_points>(
  const EntityRef & triggering_entity) -> double
{
  return std::abs(getRelativePose(triggering_entity, entity_ref).position.x);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, Between::reference_points>(
  const EntityRef & triggering_entity) -> double
{
  return std::abs(getRelativePose(triggering_entity, entity_ref).position.y);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance,
  Between::closest_bounding_box_points>(const EntityRef & triggering_entity) -> double
{
  return getBoundingBoxDistance(triggering_entity, entity_ref);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, Between::reference_points>(
  const EntityRef & triggering_entity) -> double
{
  return std::hypot(
    getRelativePose(triggering_entity, entity_ref).position.x,
    getRelativePose(triggering_entity, entity_ref).position.y);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, Between::reference_points>(
  const EntityRef & triggering_entity) -> double
{
  return getLongitudinalDistance(triggering_entity, entity_ref);
}

auto RelativeDistanceCondition::distance(const EntityRef & triggering_entity) -> double
{
  // clang-format off
  switch (coordinate_system) {
    case CoordinateSystem::entity:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::longitudinal,      Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::longitudinal,      Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::lateral,           Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::lateral,           Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::entity,     RelativeDistanceType::euclidianDistance, Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::entity,     RelativeDistanceType::euclidianDistance, Between::reference_points            >(triggering_entity);
      }
      break;

    case CoordinateSystem::lane:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::longitudinal,      Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::longitudinal,      Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::lateral,           Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::lateral,           Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::lane,       RelativeDistanceType::euclidianDistance, Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::lane,       RelativeDistanceType::euclidianDistance, Between::reference_points            >(triggering_entity);
      }
      break;

    case CoordinateSystem::road:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::longitudinal,      Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::longitudinal,      Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::lateral,           Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::lateral,           Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::road,       RelativeDistanceType::euclidianDistance, Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::road,       RelativeDistanceType::euclidianDistance, Between::reference_points            >(triggering_entity);
      }
      break;

    case CoordinateSystem::trajectory:
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:      return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::longitudinal,      Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::longitudinal,      Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::lateral:           return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::lateral,           Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::lateral,           Between::reference_points            >(triggering_entity);
        case RelativeDistanceType::euclidianDistance: return freespace ? distance< CoordinateSystem::trajectory, RelativeDistanceType::euclidianDistance, Between::closest_bounding_box_points >(triggering_entity)
                                                                       : distance< CoordinateSystem::trajectory, RelativeDistanceType::euclidianDistance, Between::reference_points            >(triggering_entity);
      }
      break;
  }
  // clang-format on

  throw Error(__FILE__, ":", __LINE__);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
