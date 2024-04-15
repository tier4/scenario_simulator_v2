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

#ifndef TRAFFIC_SIMULATOR__DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__DISTANCE_HPP_

#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>

namespace traffic_simulator
{
class DistanceUtils
{
  using CanonicalizedLaneletPose = lanelet_pose::CanonicalizedLaneletPose;
  using CanonicalizedEntityStatus = entity_status::CanonicalizedEntityStatus;

public:
  // Lateral
  static auto getLateralDistance(
    const lanelet_pose::CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
    bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
    -> std::optional<double>;

  // static auto getLateralDistance(
  //   const CanonicalizedLaneletPose & from, const std::string & to, bool allow_lane_change)
  //   -> std::optional<double>;

  // static auto getLateralDistance(
  //   const std::string & from, const CanonicalizedLaneletPose & to, bool allow_lane_change)
  //   -> std::optional<double>;

  // static auto getLateralDistance(
  //   const std::string & from, const std::string & to, bool allow_lane_change)
  //   -> std::optional<double>;

  static auto getLateralDistance(
    const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
    double matching_distance, bool allow_lane_change,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

  // static auto getLateralDistance(
  //   const CanonicalizedLaneletPose & from, const std::string & to, double matching_distance,
  //   bool allow_lane_change) -> std::optional<double>;

  // static auto getLateralDistance(
  //   const std::string & from, const CanonicalizedLaneletPose & to, double matching_distance,
  //   bool allow_lane_change) -> std::optional<double>;

  // static auto getLateralDistance(
  //   const std::string & from, const std::string & to, double matching_distance,
  //   bool allow_lane_change) -> std::optional<double>;
  // Longitudinal
  static auto getLongitudinalDistance(
    const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
    bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

  // static auto getLongitudinalDistance(
  //   const CanonicalizedLaneletPose & from, const std::string & to, bool include_adjacent_lanelet,
  //   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

  // static auto getLongitudinalDistance(
  //   const std::string & from, const CanonicalizedLaneletPose & to, bool include_adjacent_lanelet,
  //   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

  // static auto getLongitudinalDistance(
  //   const std::string & from, const std::string & to, bool include_adjacent_lanelet,
  //   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

  static auto makeNativeRelativeLanePosition(
    const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
    bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
    -> traffic_simulator::LaneletPose;

  // BoundingBox
  static auto getBoundingBoxDistance(const std::string & from, const std::string & to)
    -> std::optional<double>;

  static auto getBoundingBoxLaneLateralDistance(
    const CanonicalizedLaneletPose & from,
    const traffic_simulator_msgs::msg::BoundingBox & from_bbox, const CanonicalizedLaneletPose & to,
    const traffic_simulator_msgs::msg::BoundingBox & to_bbox, bool allow_lane_change,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

  // static auto getBoundingBoxLaneLateralDistance(
  //   const std::string & from, const CanonicalizedLaneletPose & to, bool allow_lane_change)
  //   -> std::optional<double>;

  // static auto getBoundingBoxLaneLateralDistance(
  //   const std::string & from, const std::string & to, bool allow_lane_change)
  //   -> std::optional<double>;

  static auto getBoundingBoxLaneLongitudinalDistance(
    const CanonicalizedLaneletPose & from,
    const traffic_simulator_msgs::msg::BoundingBox & from_bbox, const CanonicalizedLaneletPose & to,
    const traffic_simulator_msgs::msg::BoundingBox & to_bbox, bool include_adjacent_lanelet,
    bool include_opposite_direction, bool allow_lane_change,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

  // static auto getBoundingBoxLaneLongitudinalDistance(
  //   const std::string & from, const CanonicalizedLaneletPose & to, bool include_adjacent_lanelet,
  //   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

  // static auto getBoundingBoxLaneLongitudinalDistance(
  //   const std::string & from, const std::string & to, bool include_adjacent_lanelet,
  //   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>;

  static auto getBoundingBoxRelativePose(
    const geometry_msgs::msg::Pose & from,
    const traffic_simulator_msgs::msg::BoundingBox & from_bbox, const geometry_msgs::msg::Pose & to,
    const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
    -> std::optional<geometry_msgs::msg::Pose>;

  // static auto getBoundingBoxRelativePose(
  //   const std::string & from, const geometry_msgs::msg::Pose & to)
  //   -> std::optional<geometry_msgs::msg::Pose>;

  // static auto getBoundingBoxRelativePose(const std::string & from, const std::string & to)
  //   -> std::optional<geometry_msgs::msg::Pose>;

  // Bounds
  // static auto getDistanceToLaneBound() -> double;

  static auto getDistanceToLaneBound(
    const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  static auto getDistanceToLaneBound(
    const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  // static auto getDistanceToLeftLaneBound() -> double;

  static auto getDistanceToLeftLaneBound(
    const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  static auto getDistanceToLeftLaneBound(
    const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  // static auto getDistanceToRightLaneBound() -> double;

  static auto getDistanceToRightLaneBound(
    const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  static auto getDistanceToRightLaneBound(
    const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

  // Others

  // static auto getDistanceToCrosswalk(
  //   const std::string & name, const lanelet::Id target_crosswalk_id) -> std::optional<double>;

  // static auto getDistanceToStopLine(const std::string & name, const lanelet::Id
  // target_stop_line_id)
  //   -> std::optional<double>;

  // RelativePose
  static auto getRelativePose(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
    -> geometry_msgs::msg::Pose;

  // auto EntityManager::getRelativePose(
  //   const geometry_msgs::msg::Pose & from, const std::string & to) ->
  //   geometry_msgs::msg::Pose

  // static auto getRelativePose(const std::string & from, const geometry_msgs::msg::Pose & to)
  //   -> geometry_msgs::msg::Pose;

  // auto EntityManager::getRelativePose(const std::string & from, const std::string & to)
  //   -> geometry_msgs::msg::Pose

  static auto getRelativePose(
    const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
    -> geometry_msgs::msg::Pose;

  static auto getRelativePose(
    const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
    -> geometry_msgs::msg::Pose;

  // auto EntityManager::getRelativePose(
  //   const std::string & from, const CanonicalizedLaneletPose & to) ->
  //   geometry_msgs::msg::Pose

  // auto EntityManager::getRelativePose(
  //   const CanonicalizedLaneletPose & from, const std::string & to) ->
  //   geometry_msgs::msg::Pose

  static auto get2DPolygon(const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
    -> std::vector<geometry_msgs::msg::Point>;
};

}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__DISTANCE_HPP_
