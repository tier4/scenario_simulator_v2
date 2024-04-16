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

#ifndef TRAFFIC_SIMULATOR__POSE_HPP_
#define TRAFFIC_SIMULATOR__POSE_HPP_

#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>

namespace traffic_simulator
{
class PoseUtils
{
  using CanonicalizedLaneletPose = lanelet_pose::CanonicalizedLaneletPose;

public:
  static auto canonicalize(
    const LaneletPose & lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose;

  static auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose;

  static auto toMapPose(
    const LaneletPose & lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> geometry_msgs::msg::Pose;

  static auto toLaneletPose(
    const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
    -> std::optional<CanonicalizedLaneletPose>;
};

}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__POSE_HPP_
