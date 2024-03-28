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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__LANELET_POSE_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__LANELET_POSE_HPP_

#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

namespace traffic_simulator
{
using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;

inline namespace lanelet_pose
{
class CanonicalizedLaneletPose
{
public:
  explicit CanonicalizedLaneletPose(
    const LaneletPose & maybe_non_canonicalized_lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);
  explicit CanonicalizedLaneletPose(
    const LaneletPose & maybe_non_canonicalized_lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const lanelet::Ids & route_lanelets);
  explicit operator LaneletPose() const noexcept { return lanelet_pose_; }
  explicit operator geometry_msgs::msg::Pose() const noexcept { return map_pose_; }
  bool hasAlternativeLaneletPose() const { return lanelet_poses_.size() > 1; }
  auto getAlternativeLaneletPoseBaseOnShortestRouteFrom(
    LaneletPose from, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    bool allow_lane_change = false) const -> std::optional<LaneletPose>;

/**
   Note: The comparison operator for the CanonicalizedLaneletPose type compares
   the s values after making sure that the lanelet_id is the same. Offset and
   rpy values are not taken into account.
*/
#define DEFINE_COMPARISON_OPERATOR(OPERATOR)                                                    \
  bool operator OPERATOR(const CanonicalizedLaneletPose & rhs) const                            \
  {                                                                                             \
    if (                                                                                        \
      static_cast<LaneletPose>(*this).lanelet_id == static_cast<LaneletPose>(rhs).lanelet_id && \
      static_cast<LaneletPose>(*this).s OPERATOR static_cast<LaneletPose>(rhs).s) {             \
      return true;                                                                              \
    }                                                                                           \
    return false;                                                                               \
  }

  DEFINE_COMPARISON_OPERATOR(<=)
  DEFINE_COMPARISON_OPERATOR(<)
  DEFINE_COMPARISON_OPERATOR(>=)
  DEFINE_COMPARISON_OPERATOR(>)
#undef DEFINE_COMPARISON_OPERATOR

private:
  auto canonicalize(
    const LaneletPose & may_non_canonicalized_lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> LaneletPose;
  auto canonicalize(
    const LaneletPose & may_non_canonicalized_lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    const lanelet::Ids & route_lanelets) -> LaneletPose;
  const LaneletPose lanelet_pose_;
  const std::vector<LaneletPose> lanelet_poses_;
  const geometry_msgs::msg::Pose map_pose_;
};
}  // namespace lanelet_pose

bool isSameLaneletId(const CanonicalizedLaneletPose &, const CanonicalizedLaneletPose &);
bool isSameLaneletId(const CanonicalizedLaneletPose &, const lanelet::Id lanelet_id);

}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__LANELET_POSE_HPP_
