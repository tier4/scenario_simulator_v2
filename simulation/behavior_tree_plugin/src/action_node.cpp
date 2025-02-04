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

#include <algorithm>
#include <behavior_tree_plugin/action_node.hpp>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior
{
BT::PortsList operator+(const BT::PortsList & ports_1, const BT::PortsList & ports_2)
{
  BT::PortsList ports = ports_1;
  ports.insert(ports_2.begin(), ports_2.end());
  return ports;
}

ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

auto ActionNode::executeTick() -> BT::NodeStatus { return BT::ActionNodeBase::executeTick(); }

auto ActionNode::getBlackBoardValues() -> void
{
  if (!getInput<traffic_simulator::behavior::Request>("request", request)) {
    THROW_SIMULATION_ERROR("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time)) {
    THROW_SIMULATION_ERROR("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time)) {
    THROW_SIMULATION_ERROR("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    THROW_SIMULATION_ERROR("failed to get input hdmap_utils in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightsBase>>(
        "traffic_lights", traffic_lights)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_lights in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>(
        "canonicalized_entity_status", canonicalized_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input canonicalized_entity_status in ActionNode");
  }

  if (!getInput<std::optional<double>>("target_speed", target_speed)) {
    target_speed = std::nullopt;
  }

  if (!getInput<double>(
        "matching_distance_for_lanelet_pose_calculation",
        default_matching_distance_for_lanelet_pose_calculation)) {
    THROW_SIMULATION_ERROR(
      "failed to get input matching_distance_for_lanelet_pose_calculation in ActionNode");
  }

  if (!getInput<EntityStatusDict>("other_entity_status", other_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<lanelet::Ids>("route_lanelets", route_lanelets)) {
    THROW_SIMULATION_ERROR("failed to get input route_lanelets in ActionNode");
  }
}

auto ActionNode::getOtherEntitiesCanonicalizedLaneletPoses() const
  -> std::vector<traffic_simulator::CanonicalizedLaneletPose>
{
  std::vector<traffic_simulator::CanonicalizedLaneletPose> other_canonicalized_lanelet_poses;
  for (const auto & [name, status] : other_entity_status) {
    if (auto const & canonicalized_lanelet_pose = status.getCanonicalizedLaneletPose()) {
      other_canonicalized_lanelet_poses.push_back(canonicalized_lanelet_pose.value());
    }
  }
  return other_canonicalized_lanelet_poses;
}

auto ActionNode::getHorizon() const -> double
{
  return std::clamp(canonicalized_entity_status->getTwist().linear.x * 5.0, 20.0, 50.0);
}

auto ActionNode::stopEntity() const -> void
{
  canonicalized_entity_status->setTwist(geometry_msgs::msg::Twist());
  canonicalized_entity_status->setAccel(geometry_msgs::msg::Accel());
  canonicalized_entity_status->setLinearJerk(0);
}

auto ActionNode::setCanonicalizedEntityStatus(const traffic_simulator::EntityStatus & entity_status)
  -> void
{
  canonicalized_entity_status->set(
    entity_status, default_matching_distance_for_lanelet_pose_calculation);
}

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status->getCanonicalizedLaneletPose()) {
    if (const auto other_canonicalized_lanelet_poses = getOtherEntitiesCanonicalizedLaneletPoses();
        !other_canonicalized_lanelet_poses.empty()) {
      traffic_simulator::distance::distanceToYieldStop(
        canonicalized_lanelet_pose.value(), following_lanelets, other_canonicalized_lanelet_poses);
    }
  }
  return std::nullopt;
}

auto ActionNode::getRightOfWayEntities(const lanelet::Ids & following_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  auto is_the_same_right_of_way =
    [&](const std::int64_t & lanelet_id, const std::int64_t & following_lanelet) {
      const auto right_of_way_lanelet_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet_id);
      const auto the_same_right_of_way_it = std::find(
        right_of_way_lanelet_ids.begin(), right_of_way_lanelet_ids.end(), following_lanelet);
      return the_same_right_of_way_it != std::end(right_of_way_lanelet_ids);
    };

  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids_list = hdmap_utils->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & [name, status] : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const lanelet::Id & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          status.isInLanelet() && traffic_simulator::isSameLaneletId(status, lanelet_id) &&
          not is_the_same_right_of_way(lanelet_id, following_lanelet)) {
          ret.emplace_back(status);
        }
      }
    }
  }
  return ret;
}

auto ActionNode::getRightOfWayEntities() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  if (!canonicalized_entity_status->isInLanelet()) {
    return {};
  }
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids =
    hdmap_utils->getRightOfWayLaneletIds(canonicalized_entity_status->getLaneletId());
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & [name, status] : other_entity_status) {
    for (const lanelet::Id & lanelet_id : lanelet_ids) {
      if (status.isInLanelet() && traffic_simulator::isSameLaneletId(status, lanelet_id)) {
        ret.emplace_back(status);
      }
    }
  }
  return ret;
}

auto ActionNode::getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (const auto traffic_light_ids = hdmap_utils->getTrafficLightIdsOnPath(route_lanelets);
      !traffic_light_ids.empty()) {
    std::set<double> collision_points = {};
    for (const auto traffic_light_id : traffic_light_ids) {
      if (traffic_lights->isRequiredStopTrafficLightState(traffic_light_id)) {
        if (
          const auto collision_point =
            traffic_simulator::distance::distanceToTrafficLightStopLine(spline, traffic_light_id)) {
          collision_points.insert(collision_point.value());
        }
      }
    }
    if (!collision_points.empty()) {
      return *collision_points.begin();
    }
  }
  return std::nullopt;
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (const auto entity_name = getFrontEntityName(spline)) {
    return getDistanceToTargetEntity(spline, getEntityStatus(entity_name.value()));
  } else {
    return std::nullopt;
  }
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & [name, status] : other_entity_status) {
    const auto distance = getDistanceToTargetEntity(spline, status);
    const auto quat = math::geometry::getRotation(
      canonicalized_entity_status->getMapPose().orientation,
      other_entity_status.at(name).getMapPose().orientation);
    /**
     * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a candidate of front entity.
     */
    if (
      std::fabs(math::geometry::convertQuaternionToEulerAngle(quat).z) <=
      boost::math::constants::half_pi<double>()) {
      if (distance && distance.value() < 40) {
        entities.emplace_back(name);
        distances.emplace_back(distance.value());
      }
    }
  }
  if (entities.size() != distances.size()) {
    THROW_SIMULATION_ERROR("size of entities and distances vector does not match.");
  }
  if (distances.empty()) {
    return std::nullopt;
  }
  std::vector<double>::iterator iter = std::min_element(distances.begin(), distances.end());
  size_t index = std::distance(distances.begin(), iter);
  return entities[index];
}

auto ActionNode::getDistanceToTargetEntityOnCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status) const -> std::optional<double>
{
  if (status.isInLanelet()) {
    return spline.getCollisionPointIn2D(
      hdmap_utils->getLaneletPolygon(status.getLaneletId()), false);
  }
  return std::nullopt;
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> const traffic_simulator::CanonicalizedEntityStatus &
{
  if (auto it = other_entity_status.find(target_name); it != other_entity_status.end()) {
    return it->second;
  } else {
    THROW_SEMANTIC_ERROR("other entity : ", target_name, " does not exist.");
  }
}

/**
 * @note getDistanceToTargetEntity working schematics
 *
 * 1. Check if route to target entity from reference entity exists, if not try to transform pose to other
 *    routable lanelet, within matching distance (findRoutableAlternativeLaneletPoseFrom).
 * 2. Calculate longitudinal distance between entities bounding boxes -> bounding_box_distance.
 * 3. Calculate longitudinal distance between entities poses -> position_distance.
 * 4. Calculate target entity bounding box distance to reference entity spline (minimal distance from all corners)
 *    -> target_to_spline_distance.
 * 5. If target_to_spline_distance is less than half width of reference entity target entity is conflicting.
 * 6. Check corner case where target entity width is bigger than width of entity and target entity
 *    is exactly on the spline -> spline.getCollisionPointIn2D
 * 7. If target entity is conflicting return bounding_box_distance enlarged by half of the entity
 *    length.
 */
auto ActionNode::getDistanceToTargetEntity(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status) const -> std::optional<double>
{
  if (
    !status.isInLanelet() || !canonicalized_entity_status->isInLanelet() ||
    !isOtherEntityAtConsideredAltitude(status)) {
    return std::nullopt;
  }
  /**
      * boundingBoxLaneLongitudinalDistance requires routing_configuration,
      * 'allow_lane_change = true' is needed to check distances to entities on neighbour lanelets
      */
  traffic_simulator::RoutingConfiguration routing_configuration;
  routing_configuration.allow_lane_change = true;
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};
  constexpr bool search_backward{false};

  const auto & target_bounding_box = status.getBoundingBox();

  if (const auto & target_lanelet_pose =
        traffic_simulator::pose::findRoutableAlternativeLaneletPoseFrom(
          canonicalized_entity_status->getLaneletId(), status.getCanonicalizedLaneletPose().value(),
          target_bounding_box);
      target_lanelet_pose) {
    const auto & from_lanelet_pose = canonicalized_entity_status->getCanonicalizedLaneletPose();
    const auto & from_bounding_box = canonicalized_entity_status->getBoundingBox();
    if (const auto bounding_box_distance =
          traffic_simulator::distance::boundingBoxLaneLongitudinalDistance(
            *from_lanelet_pose, from_bounding_box, *target_lanelet_pose, target_bounding_box,
            include_adjacent_lanelet, include_opposite_direction, routing_configuration);
        !bounding_box_distance || bounding_box_distance.value() < 0.0) {
      return std::nullopt;
    } else if (const auto position_distance = traffic_simulator::distance::longitudinalDistance(
                 *from_lanelet_pose, *target_lanelet_pose, include_adjacent_lanelet,
                 include_opposite_direction, routing_configuration);
               !position_distance) {
      return std::nullopt;
    } else {
      const auto target_bounding_box_distance =
        bounding_box_distance.value() + from_bounding_box.dimensions.x / 2.0;

      /// @note if the distance of the target entity to the spline is smaller than the width of the reference entity
      if (const auto target_to_spline_distance = traffic_simulator::distance::distanceToSpline(
            static_cast<geometry_msgs::msg::Pose>(*target_lanelet_pose), target_bounding_box,
            spline, position_distance.value());
          target_to_spline_distance <= from_bounding_box.dimensions.y / 2.0) {
        return target_bounding_box_distance;
      }
      /// @note if the distance of the target entity to the spline cannot be calculated because a collision occurs
      else if (const auto target_polygon = math::geometry::transformPoints(
                 status.getMapPose(), math::geometry::getPointsFromBbox(target_bounding_box));
               spline.getCollisionPointIn2D(target_polygon, search_backward)) {
        return target_bounding_box_distance;
      }
    }
  }
  return std::nullopt;
}

auto ActionNode::isOtherEntityAtConsideredAltitude(
  const traffic_simulator::CanonicalizedEntityStatus & entity_status) const -> bool
{
  if (canonicalized_entity_status->isInLanelet() && entity_status.isInLanelet()) {
    return traffic_simulator::pose::isAltitudeMatching(
      canonicalized_entity_status->getCanonicalizedLaneletPose().value(),
      entity_status.getCanonicalizedLaneletPose().value());
  } else {
    return false;
  }
}

auto ActionNode::getDistanceToConflictingEntity(
  const lanelet::Ids & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto crosswalk_entity_status = getConflictingEntityStatusOnCrossWalk(route_lanelets);
  auto lane_entity_status = getConflictingEntityStatusOnLane(route_lanelets);
  std::set<double> distances;
  for (const auto & status : crosswalk_entity_status) {
    const auto s = getDistanceToTargetEntityOnCrosswalk(spline, status);
    if (s) {
      distances.insert(s.value());
    }
  }
  for (const auto & status : lane_entity_status) {
    if (const auto distance_to_entity = getDistanceToTargetEntity(spline, status)) {
      distances.insert(distance_to_entity.value());
    }
  }
  if (distances.empty()) {
    return std::nullopt;
  }
  return *distances.begin();
}

auto ActionNode::getConflictingEntityStatusOnCrossWalk(const lanelet::Ids & route_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(route_lanelets);
  for (const auto & [name, status] : other_entity_status) {
    if (
      status.isInLanelet() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(), status.getLaneletId()) >= 1) {
      conflicting_entity_status.emplace_back(status);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::getConflictingEntityStatusOnLane(const lanelet::Ids & route_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(route_lanelets);
  for (const auto & [name, status] : other_entity_status) {
    if (
      status.isInLanelet() &&
      std::count(conflicting_lanes.begin(), conflicting_lanes.end(), status.getLaneletId()) >= 1) {
      conflicting_entity_status.emplace_back(status);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::foundConflictingEntity(const lanelet::Ids & following_lanelets) const -> bool
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(following_lanelets);
  for (const auto & [name, status] : other_entity_status) {
    if (
      status.isInLanelet() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(), status.getLaneletId()) >= 1) {
      return true;
    }
    if (
      status.isInLanelet() &&
      std::count(conflicting_lanes.begin(), conflicting_lanes.end(), status.getLaneletId()) >= 1) {
      return true;
    }
  }
  return false;
}

auto ActionNode::calculateUpdatedEntityStatus(
  const double local_target_speed,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());

  const double linear_jerk_new = std::get<2>(dynamics);
  const geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  const geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status->getCanonicalizedLaneletPose()) {
    const auto distance =
      (twist_new.linear.x + canonicalized_entity_status->getTwist().linear.x) / 2.0 * step_time;
    auto entity_status_updated =
      static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    entity_status_updated.action_status.linear_jerk = linear_jerk_new;
    /// @todo it will be moved to route::moveAlongLaneletPose(...)
    entity_status_updated.lanelet_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
      static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value()),
      route_lanelets, distance);
    entity_status_updated.lanelet_pose_valid = true;
    entity_status_updated.pose =
      traffic_simulator::pose::toMapPose(entity_status_updated.lanelet_pose);
    return entity_status_updated;
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot move along lanelet - entity ", std::quoted(canonicalized_entity_status->getName()),
      " has invalid lanelet pose.");
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  const double local_target_speed,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  using math::geometry::operator*;
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());
  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  geometry_msgs::msg::Pose pose_new;
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = twist_new.angular.z * step_time;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    math::geometry::convertEulerAngleToQuaternion(angular_trans_vec);
  pose_new.orientation = canonicalized_entity_status->getMapPose().orientation * angular_trans_quat;
  Eigen::Vector3d trans_vec;
  trans_vec(0) = twist_new.linear.x * step_time;
  trans_vec(1) = twist_new.linear.y * step_time;
  trans_vec(2) = 0;
  Eigen::Matrix3d rotation_mat = math::geometry::getRotationMatrix(pose_new.orientation);
  trans_vec = rotation_mat * trans_vec;
  pose_new.position.x = trans_vec(0) + canonicalized_entity_status->getMapPose().position.x;
  pose_new.position.y = trans_vec(1) + canonicalized_entity_status->getMapPose().position.y;
  pose_new.position.z = trans_vec(2) + canonicalized_entity_status->getMapPose().position.z;
  auto entity_status_updated =
    static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.lanelet_pose = traffic_simulator::LaneletPose();
  entity_status_updated.lanelet_pose_valid = false;
  entity_status_updated.pose = pose_new;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.action_status.linear_jerk = linear_jerk_new;
  return entity_status_updated;
}

auto ActionNode::calculateStopDistance(
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const -> double
{
  return traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
           step_time, canonicalized_entity_status->getName())
    .getRunningDistance(
      0, constraints, canonicalized_entity_status->getTwist(),
      canonicalized_entity_status->getAccel(), canonicalized_entity_status->getLinearJerk());
}
}  // namespace entity_behavior
