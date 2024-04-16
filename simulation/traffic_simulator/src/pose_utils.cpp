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

#include <traffic_simulator/pose_utils.hpp>

namespace traffic_simulator
{
auto PoseUtils::canonicalize(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose
{
  return CanonicalizedLaneletPose(lanelet_pose, hdmap_utils_ptr);
}

auto PoseUtils::toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto PoseUtils::toMapPose(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> geometry_msgs::msg::Pose
{
  return hdmap_utils_ptr->toMapPose(lanelet_pose).pose;
}

auto PoseUtils::toLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (const auto pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk)) {
    return PoseUtils::canonicalize(pose.value(), hdmap_utils_ptr);
  }
  return std::nullopt;
}
}  // namespace traffic_simulator
