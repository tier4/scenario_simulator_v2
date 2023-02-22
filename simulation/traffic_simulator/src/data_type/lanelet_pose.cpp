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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_pose
{
CanonicalizedLaneletPose::CanonicalizedLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & maybe_non_canonicalized_lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
: lanelet_pose_(canonicalize(maybe_non_canonicalized_lanelet_pose, hdmap_utils))
{
}

auto CanonicalizedLaneletPose::canonicalize(
  const traffic_simulator_msgs::msg::LaneletPose & may_non_canonicalized_lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
  -> traffic_simulator_msgs::msg::LaneletPose
{
  if (
    const auto canonicalized =
      hdmap_utils->canonicalizeLaneletPose(may_non_canonicalized_lanelet_pose)) {
    return canonicalized.get();
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", may_non_canonicalized_lanelet_pose.lanelet_id,
      ",s=", may_non_canonicalized_lanelet_pose.s,
      ",offset=", may_non_canonicalized_lanelet_pose.offset,
      ",rpy.x=", may_non_canonicalized_lanelet_pose.rpy.x,
      ",rpy.y=", may_non_canonicalized_lanelet_pose.rpy.y,
      ",rpy.z=", may_non_canonicalized_lanelet_pose.rpy.z,
      ") is invalid, please check lanelet length and connection.");
  }
}
}  // namespace lanelet_pose
}  // namespace traffic_simulator
