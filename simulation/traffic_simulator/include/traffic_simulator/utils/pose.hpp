// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__POSE_HPP_

#include <traffic_simulator/data_type/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace pose
{
using CanonicalizedLaneletPose = lanelet_pose::CanonicalizedLaneletPose;

// Constructors
auto getQuietNaNPose() -> geometry_msgs::msg::Pose;

auto getQuietNaNLaneletPose() -> traffic_simulator::LaneletPose;

// Conversions
auto canonicalize(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose;

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> const geometry_msgs::msg::Pose;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>;

// Relative msg::Pose
auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto getRelativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto getRelativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto getBoundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<geometry_msgs::msg::Pose>;

// Relative LaneletPose
auto makeNativeRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose;

auto makeNativeBoundingBoxRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose;
}  // namespace pose
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__POSE_HPP_
