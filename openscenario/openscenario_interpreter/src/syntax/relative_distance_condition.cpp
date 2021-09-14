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
auto RelativeDistanceCondition::distance(const EntityRef & triggering_entity) -> double
{
  switch (coordinate_system) {
    //
    case CoordinateSystem::entity:
      //
      switch (relative_distance_type) {
        //
        case RelativeDistanceType::longitudinal:
          if (freespace) {
            return distance<
              CoordinateSystem::entity, RelativeDistanceType::longitudinal,
              Between::closest_bounding_box_points>(triggering_entity);
          } else {
            // return std::abs(getRelativePose(triggering_entity, entity_ref).position.x);
            return distance<
              CoordinateSystem::entity, RelativeDistanceType::longitudinal,
              Between::reference_points>(triggering_entity);
          }

        case RelativeDistanceType::lateral:
          if (freespace) {
            throw SyntaxError(__FILE__, ":", __LINE__);
          } else {
            return std::abs(getRelativePose(triggering_entity, entity_ref).position.y);
          }

        case RelativeDistanceType::cartesianDistance:
        case RelativeDistanceType::euclidianDistance:
          if (freespace) {
            return getBoundingBoxDistance(triggering_entity, entity_ref);
          } else {
            return std::hypot(
              getRelativePose(triggering_entity, entity_ref).position.x,
              getRelativePose(triggering_entity, entity_ref).position.y);
          }

        default:
          throw SyntaxError(__FILE__, ":", __LINE__);
      }

    case CoordinateSystem::lane:
      //
      switch (relative_distance_type) {
        //
        case RelativeDistanceType::longitudinal:
          if (freespace) {
            throw SyntaxError(__FILE__, ":", __LINE__);
          } else {
            return getLongitudinalDistance(triggering_entity, entity_ref);
          }

        case RelativeDistanceType::lateral:
          if (freespace) {
            throw SyntaxError(__FILE__, ":", __LINE__);
          } else {
            throw SyntaxError(__FILE__, ":", __LINE__);
          }

        case RelativeDistanceType::cartesianDistance:
        case RelativeDistanceType::euclidianDistance:
          if (freespace) {
            throw SyntaxError(__FILE__, ":", __LINE__);
          } else {
            throw SyntaxError(__FILE__, ":", __LINE__);
          }

        default:
          throw SyntaxError(__FILE__, ":", __LINE__);
      }

    default:
      throw SyntaxError(__FILE__, ":", __LINE__);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
