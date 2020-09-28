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

#ifndef SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_
#define SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_

#include <scenario_runner/syntax/absolute_target_lane.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== LaneChangeTarget =====================================================
 *
 * <xsd:complexType name="LaneChangeTarget">
 *   <xsd:choice>
 *     <xsd:element name="RelativeTargetLane" type="RelativeTargetLane"/>
 *     <xsd:element name="AbsoluteTargetLane" type="AbsoluteTargetLane"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct LaneChangeTarget
  : public Object
{
  template<typename Node, typename ... Ts>
  explicit LaneChangeTarget(const Node & node, Ts && ... xs)
  {
    callWithElements(node, "RelativeTargetLane", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "AbsoluteTargetLane", 0, 1, [&](auto && node)
      {
        return rebind<AbsoluteTargetLane>(node, std::forward<decltype(xs)>(xs)...);
      });
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__LANE_CHANGE_TARGET_HPP_
