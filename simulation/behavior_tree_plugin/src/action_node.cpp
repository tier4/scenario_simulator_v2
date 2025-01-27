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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/operator.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
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

auto ActionNode::getOtherEntityStatus(lanelet::Id lanelet_id) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      traffic_simulator::isSameLaneletId(status.second, lanelet_id)) {
      ret.emplace_back(status.second);
    }
  }
  return ret;
}

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  std::set<double> distances;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (!other_status.empty() && canonicalized_entity_status->laneMatchingSucceed()) {
        const auto lanelet_pose = canonicalized_entity_status->getLaneletPose();
        const auto distance_forward = hdmap_utils->getLongitudinalDistance(
          lanelet_pose, traffic_simulator::helper::constructLaneletPose(lanelet, 0));
        const auto distance_backward = hdmap_utils->getLongitudinalDistance(
          traffic_simulator::helper::constructLaneletPose(lanelet, 0), lanelet_pose);
        if (distance_forward) {
          distances.insert(distance_forward.value());
        } else if (distance_backward) {
          distances.insert(-distance_backward.value());
        }
      }
    }
    if (distances.size() != 0) {
      return *distances.begin();
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
  for (const auto & status : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const lanelet::Id & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          status.second.laneMatchingSucceed() &&
          traffic_simulator::isSameLaneletId(status.second, lanelet_id) &&
          not is_the_same_right_of_way(lanelet_id, following_lanelet)) {
          ret.emplace_back(status.second);
        }
      }
    }
  }
  return ret;
}

auto ActionNode::getRightOfWayEntities() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    return {};
  }
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids =
    hdmap_utils->getRightOfWayLaneletIds(canonicalized_entity_status->getLaneletId());
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & status : other_entity_status) {
    for (const lanelet::Id & lanelet_id : lanelet_ids) {
      if (
        status.second.laneMatchingSucceed() &&
        traffic_simulator::isSameLaneletId(status.second, lanelet_id)) {
        ret.emplace_back(status.second);
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
            hdmap_utils->getDistanceToTrafficLightStopLine(spline, traffic_light_id)) {
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

auto ActionNode::getDistanceToStopLine(
  const lanelet::Ids & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>
{
  return hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints);
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto name = getFrontEntityName(spline);
  if (!name) {
    return std::nullopt;
  }
  return getDistanceToTargetEntityPolygon(spline, name.value());
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & each : other_entity_status) {
    const auto distance = getDistanceToTargetEntityPolygon(spline, each.first);
    const auto quat = math::geometry::getRotation(
      canonicalized_entity_status->getMapPose().orientation,
      other_entity_status.at(each.first).getMapPose().orientation);
    /**
     * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a candidate of front entity.
     */
    if (
      std::fabs(math::geometry::convertQuaternionToEulerAngle(quat).z) <=
      boost::math::constants::half_pi<double>()) {
      if (distance && distance.value() < 40) {
        entities.emplace_back(each.first);
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
  if (status.laneMatchingSucceed()) {
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

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline, const std::string target_name,
  double width_extension_right, double width_extension_left, double length_extension_front,
  double length_extension_rear) const -> std::optional<double>
{
  return getDistanceToTargetEntityPolygon(
    spline, getEntityStatus(target_name), width_extension_right, width_extension_left,
    length_extension_front, length_extension_rear);
}

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear) const
  -> std::optional<double>
{
  if (isOtherEntityAtConsideredAltitude(status)) {
    const auto polygon = math::geometry::transformPoints(
      status.getMapPose(), math::geometry::getPointsFromBbox(
                             status.getBoundingBox(), width_extension_right, width_extension_left,
                             length_extension_front, length_extension_rear));
    return spline.getCollisionPointIn2D(polygon, false);
  } else {
    return std::nullopt;
  }
}

auto ActionNode::isOtherEntityAtConsideredAltitude(
  const traffic_simulator::CanonicalizedEntityStatus & entity_status) const -> bool
{
  if (canonicalized_entity_status->laneMatchingSucceed() && entity_status.laneMatchingSucceed()) {
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
    const auto s = getDistanceToTargetEntityPolygon(spline, status, 0.0, 0.0, 0.0, 1.0);
    if (s) {
      distances.insert(s.value());
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
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.getLaneletId()) >= 1) {
      conflicting_entity_status.emplace_back(status.second);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::getConflictingEntityStatusOnLane(const lanelet::Ids & route_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(), status.second.getLaneletId()) >= 1) {
      conflicting_entity_status.emplace_back(status.second);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::foundConflictingEntity(const lanelet::Ids & following_lanelets) const -> bool
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.getLaneletId()) >= 1) {
      return true;
    }
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(), status.second.getLaneletId()) >= 1) {
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
  using math::geometry::operator+;
  using math::geometry::operator+=;

  constexpr bool desired_velocity_is_global{false};

  const auto include_crosswalk = [](const auto & entity_type) {
    return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
           (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
  }(canonicalized_entity_status->getType());

  const auto matching_distance = default_matching_distance_for_lanelet_pose_calculation;

  const auto buildUpdatedPose =
    [&include_crosswalk, &matching_distance](
      const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status,
      const geometry_msgs::msg::Twist & desired_twist, const double time_step) {
      geometry_msgs::msg::Pose updated_pose;

      // Apply yaw change (delta rotation) in radians: yaw_angular_speed (rad/s) * time_step (s)
      geometry_msgs::msg::Vector3 delta_rotation;
      delta_rotation = desired_twist.angular * time_step;
      const auto delta_quaternion = math::geometry::convertEulerAngleToQuaternion(delta_rotation);
      updated_pose.orientation = status->getMapPose().orientation * delta_quaternion;

      // Apply position change
      /// @todo first determine global desired_velocity, calculate position change using it
      /// then pass the same global desired_velocity to adjustPoseForLaneletTransition()
      const Eigen::Matrix3d rotation_matrix =
        math::geometry::getRotationMatrix(updated_pose.orientation);
      const auto translation = Eigen::Vector3d(
        desired_twist.linear.x * time_step, desired_twist.linear.y * time_step,
        desired_twist.linear.z * time_step);
      const Eigen::Vector3d delta_position = rotation_matrix * translation;
      updated_pose.position = status->getMapPose().position + delta_position;

      // If it is the transition between lanelets: overwrite position to improve precision
      if (const auto canonicalized_lanelet_pose = status->getCanonicalizedLaneletPose()) {
        const auto estimated_next_canonicalized_lanelet_pose =
          traffic_simulator::pose::toCanonicalizedLaneletPose(
            updated_pose, status->getBoundingBox(), include_crosswalk, matching_distance);
        if (estimated_next_canonicalized_lanelet_pose) {
          const auto next_lanelet_id = static_cast<traffic_simulator::LaneletPose>(
                                         estimated_next_canonicalized_lanelet_pose.value())
                                         .lanelet_id;
          if (  // Handle lanelet transition
            const auto updated_position =
              traffic_simulator::pose::updatePositionForLaneletTransition(
                canonicalized_lanelet_pose.value(), next_lanelet_id, desired_twist.linear,
                desired_velocity_is_global, time_step)) {
            updated_pose.position = updated_position.value();
          }
        }
      }
      return updated_pose;
    };

  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());
  const auto linear_jerk_new = std::get<2>(dynamics);
  const auto & accel_new = std::get<1>(dynamics);
  const auto & twist_new = std::get<0>(dynamics);
  const auto pose_new = buildUpdatedPose(canonicalized_entity_status, twist_new, step_time);

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
