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
    const LaneletPose & maybe_non_canonicalized_lanelet_pose, const lanelet::Ids & route_lanelets,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils);
  CanonicalizedLaneletPose(const CanonicalizedLaneletPose & other);
  CanonicalizedLaneletPose(CanonicalizedLaneletPose && other) noexcept;
  CanonicalizedLaneletPose & operator=(const CanonicalizedLaneletPose & obj);
  explicit operator LaneletPose() const noexcept { return lanelet_pose_; }
  explicit operator geometry_msgs::msg::Pose() const noexcept { return map_pose_; }
  auto getLaneletPose() const -> const LaneletPose & { return lanelet_pose_; }
  auto hasAlternativeLaneletPose() const -> bool { return lanelet_poses_.size() > 1; }
  auto getAlternativeLaneletPoseBaseOnShortestRouteFrom(
    LaneletPose from, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
    bool allow_lane_change = false) const -> std::optional<LaneletPose>;
  static auto setConsiderPoseByRoadSlope(bool consider_pose_by_road_slope) -> void
  {
    consider_pose_by_road_slope_ = consider_pose_by_road_slope;
  }
  static auto getConsiderPoseByRoadSlope() -> bool { return consider_pose_by_road_slope_; }

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
  auto adjustOrientationAndOzPosition(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils)
    -> void;
  auto canonicalize(
    const LaneletPose & may_non_canonicalized_lanelet_pose,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> LaneletPose;
  auto canonicalize(
    const LaneletPose & may_non_canonicalized_lanelet_pose, const lanelet::Ids & route_lanelets,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils) -> LaneletPose;
  LaneletPose lanelet_pose_;
  std::vector<LaneletPose> lanelet_poses_;
  geometry_msgs::msg::Pose map_pose_;
  inline static bool consider_pose_by_road_slope_{false};
};
}  // namespace lanelet_pose

auto isSameLaneletId(const CanonicalizedLaneletPose &, const CanonicalizedLaneletPose &) -> bool;
auto isSameLaneletId(const CanonicalizedLaneletPose &, const lanelet::Id lanelet_id) -> bool;
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__LANELET_POSE_HPP_
