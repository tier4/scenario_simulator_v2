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

auto ActionNode::tick() -> BT::NodeStatus
{
  getBlackBoardValues();
  if (!checkPreconditions()) {
    return BT::NodeStatus::FAILURE;
  }
  return doAction();
}

auto ActionNode::getBlackBoardValues() -> void
{
  if (!getInput<traffic_simulator::behavior::Request>("request", request_)) {
    THROW_SIMULATION_ERROR("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time_)) {
    THROW_SIMULATION_ERROR("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time_)) {
    THROW_SIMULATION_ERROR("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils_)) {
    THROW_SIMULATION_ERROR("failed to get input hdmap_utils in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightsBase>>(
        "traffic_lights", traffic_lights_)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_lights in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>(
        "canonicalized_entity_status", canonicalized_entity_status_)) {
    THROW_SIMULATION_ERROR("failed to get input canonicalized_entity_status in ActionNode");
  }

  if (!getInput<std::optional<double>>("target_speed", target_speed_)) {
    target_speed_ = std::nullopt;
  }

  if (!getInput<double>(
        "matching_distance_for_lanelet_pose_calculation",
        default_matching_distance_for_lanelet_pose_calculation_)) {
    THROW_SIMULATION_ERROR(
      "failed to get input matching_distance_for_lanelet_pose_calculation in ActionNode");
  }

  if (!getInput<EntityStatusDict>("other_entity_status", other_entity_status_)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status_ in ActionNode");
  }
  if (!getInput<lanelet::Ids>("route_lanelets", route_lanelets_)) {
    THROW_SIMULATION_ERROR("failed to get input route_lanelets_ in ActionNode");
  }
  if (!getInput<std::shared_ptr<EuclideanDistancesMap>>(
        "euclidean_distances_map", euclidean_distances_map_)) {
    euclidean_distances_map_ = std::make_shared<EuclideanDistancesMap>();
  }
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter_)) {
    behavior_parameter_ = traffic_simulator_msgs::msg::BehaviorParameter();
  }
}

auto ActionNode::getHorizon() const -> double
{
  return std::clamp(canonicalized_entity_status_->getTwist().linear.x * 5.0, 20.0, 50.0);
}

auto ActionNode::stopEntity() const -> void
{
  canonicalized_entity_status_->setTwist(geometry_msgs::msg::Twist());
  canonicalized_entity_status_->setAccel(geometry_msgs::msg::Accel());
  canonicalized_entity_status_->setLinearJerk(0);
}

auto ActionNode::setCanonicalizedEntityStatus(const traffic_simulator::EntityStatus & entity_status)
  -> void
{
  canonicalized_entity_status_->set(
    entity_status, default_matching_distance_for_lanelet_pose_calculation_);
}

auto ActionNode::getOtherEntityStatus(lanelet::Id lanelet_id) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  for (const auto & [name, status] : other_entity_status_) {
    if (status.isInLanelet() && traffic_simulator::isSameLaneletId(status, lanelet_id)) {
      ret.emplace_back(status);
    }
  }
  return ret;
}

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  std::set<double> distances;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils_->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (!other_status.empty() && canonicalized_entity_status_->isInLanelet()) {
        const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
        const auto distance_forward = hdmap_utils_->getLongitudinalDistance(
          lanelet_pose, traffic_simulator::helper::constructLaneletPose(lanelet, 0));
        const auto distance_backward = hdmap_utils_->getLongitudinalDistance(
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
      const auto right_of_way_lanelet_ids = hdmap_utils_->getRightOfWayLaneletIds(lanelet_id);
      const auto the_same_right_of_way_it = std::find(
        right_of_way_lanelet_ids.begin(), right_of_way_lanelet_ids.end(), following_lanelet);
      return the_same_right_of_way_it != std::end(right_of_way_lanelet_ids);
    };

  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids_list = hdmap_utils_->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & [name, status] : other_entity_status_) {
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
  if (!canonicalized_entity_status_->isInLanelet()) {
    return {};
  }
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids =
    hdmap_utils_->getRightOfWayLaneletIds(canonicalized_entity_status_->getLaneletId());
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & [name, status] : other_entity_status_) {
    for (const lanelet::Id & lanelet_id : lanelet_ids) {
      if (status.isInLanelet() && traffic_simulator::isSameLaneletId(status, lanelet_id)) {
        ret.emplace_back(status);
      }
    }
  }
  return ret;
}

auto ActionNode::getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets_,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (const auto traffic_light_ids = hdmap_utils_->getTrafficLightIdsOnPath(route_lanelets_);
      !traffic_light_ids.empty()) {
    std::set<double> collision_points = {};
    for (const auto traffic_light_id : traffic_light_ids) {
      if (traffic_lights_->isRequiredStopTrafficLightState(traffic_light_id)) {
        if (
          const auto collision_point =
            hdmap_utils_->getDistanceToTrafficLightStopLine(spline, traffic_light_id)) {
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
    std::cout << "getDistanceToFrontEntity: " << entity_name.value() << std::endl;

    return getDistanceToTargetEntity(spline, getEntityStatus(entity_name.value()));
  } else {
    std::cout << "getDistanceToFrontEntity: "
              << "NULL" << std::endl;

    return std::nullopt;
  }
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  if (euclidean_distances_map_ != nullptr) {
    std::map<double, std::string> local_euclidean_distances_map_;
    const double stop_distance = calculateStopDistance(behavior_parameter_.dynamic_constraints);
    const double horizon = spline.getLength() > stop_distance ? spline.getLength() : stop_distance;
    for (const auto & [name_pair, euclidean_distance] : *euclidean_distances_map_) {
      /**
       * @note Euclidean distance is here used as a "rough" distance to filter only NPCs which possibly are in range of current horizon. Because euclidean distance is the shortest possible distance comparing it with horizon will never omit NPCs for which actual lane distance is in range of horizon.
       */
      if (euclidean_distance < horizon) {
        if (name_pair.first == canonicalized_entity_status_->getName()) {
          local_euclidean_distances_map_.emplace(euclidean_distance, name_pair.second);
        } else if (name_pair.second == canonicalized_entity_status_->getName()) {
          local_euclidean_distances_map_.emplace(euclidean_distance, name_pair.first);
        }
      }
    }

    for (const auto & [euclidean_distance, name] : local_euclidean_distances_map_) {
      const auto self_pos = canonicalized_entity_status_->getMapPose().position;
      const auto other_pos = other_entity_status_.at(name).getMapPose().position;
      const auto dx = other_pos.x - self_pos.x;
      const auto dy = other_pos.y - self_pos.y;
      const auto self_yaw = math::geometry::convertQuaternionToEulerAngle(
                              canonicalized_entity_status_->getMapPose().orientation)
                              .z;
      const auto vec_yaw = std::atan2(dy, dx);
      const auto yaw_diff = std::atan2(std::sin(vec_yaw - self_yaw), std::cos(vec_yaw - self_yaw));
      if (std::fabs(yaw_diff) <= boost::math::constants::half_pi<double>()) {
        return name;
      }
    }
  }
  return std::nullopt;
}

auto ActionNode::getDistanceToTargetEntityOnCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status) const -> std::optional<double>
{
  if (status.isInLanelet()) {
    return spline.getCollisionPointIn2D(
      hdmap_utils_->getLaneletPolygon(status.getLaneletId()), false);
  }
  return std::nullopt;
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> const traffic_simulator::CanonicalizedEntityStatus &
{
  if (auto it = other_entity_status_.find(target_name); it != other_entity_status_.end()) {
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
 * 3. Calculate longitudinal distance between entities poses -> longitudinal_distance.
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
    !status.isInLanelet() || !canonicalized_entity_status_->isInLanelet() ||
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
          canonicalized_entity_status_->getLaneletId(),
          status.getCanonicalizedLaneletPose().value(), target_bounding_box);
      target_lanelet_pose) {
    const auto & from_lanelet_pose = canonicalized_entity_status_->getCanonicalizedLaneletPose();
    const auto & from_bounding_box = canonicalized_entity_status_->getBoundingBox();
    const auto bounding_box_map_points = math::geometry::transformPoints(
      static_cast<geometry_msgs::msg::Pose>(*target_lanelet_pose),
      math::geometry::getPointsFromBbox(target_bounding_box));
    const auto bounding_box_diagonal_length =
      math::geometry::getDistance(bounding_box_map_points[0], bounding_box_map_points[2]);
    if (const auto longitudinal_distance = traffic_simulator::distance::longitudinalDistance(
          *from_lanelet_pose, *target_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, routing_configuration, hdmap_utils_);
        !longitudinal_distance) {
      return std::nullopt;
    } else if (const auto bounding_box_distance =
                 traffic_simulator::distance::boundingBoxLaneLongitudinalDistance(
                   longitudinal_distance, from_bounding_box, target_bounding_box);
               !bounding_box_distance || bounding_box_distance.value() < 0.0) {
      return std::nullopt;
    } else {
      /// @todo rotation of NPC is not taken into account, same as in boundingBoxLaneLongitudinalDistance
      /// this should be considered to be changed in separate task in the future
      const auto target_bounding_box_distance =
        bounding_box_distance.value() + from_bounding_box.dimensions.x / 2.0;

      /// @note if the distance of the target entity to the spline is smaller than the width of the reference entity
      if (const auto target_to_spline_distance = traffic_simulator::distance::distanceToSpline(
            static_cast<geometry_msgs::msg::Pose>(*target_lanelet_pose), target_bounding_box,
            spline, longitudinal_distance.value());
          target_to_spline_distance <= from_bounding_box.dimensions.y / 2.0) {
        return target_bounding_box_distance;
      }
      /// @note if the distance of the target entity to the spline cannot be calculated because a collision occurs
      else if (const auto target_polygon = math::geometry::transformPoints(
                 status.getMapPose(), math::geometry::getPointsFromBbox(target_bounding_box));
               spline.getCollisionPointIn2D(
                 target_polygon, search_backward,
                 std::make_pair(
                   bounding_box_distance.value(),
                   target_bounding_box_distance + bounding_box_diagonal_length))) {
        return target_bounding_box_distance;
      }
    }
  }
  return std::nullopt;
}

auto ActionNode::isOtherEntityAtConsideredAltitude(
  const traffic_simulator::CanonicalizedEntityStatus & entity_status) const -> bool
{
  if (canonicalized_entity_status_->isInLanelet() && entity_status.isInLanelet()) {
    return traffic_simulator::pose::isAltitudeMatching(
      canonicalized_entity_status_->getCanonicalizedLaneletPose().value(),
      entity_status.getCanonicalizedLaneletPose().value());
  } else {
    return false;
  }
}

auto ActionNode::getDistanceToConflictingEntity(
  const lanelet::Ids & route_lanelets_,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto crosswalk_entity_status = getConflictingEntityStatusOnCrossWalk(route_lanelets_);
  auto lane_entity_status = getConflictingEntityStatusOnLane(route_lanelets_);
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

auto ActionNode::getConflictingEntityStatusOnCrossWalk(const lanelet::Ids & route_lanelets_) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_crosswalks = hdmap_utils_->getConflictingCrosswalkIds(route_lanelets_);
  for (const auto & [name, status] : other_entity_status_) {
    if (
      status.isInLanelet() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(), status.getLaneletId()) >= 1) {
      conflicting_entity_status.emplace_back(status);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::getConflictingEntityStatusOnLane(const lanelet::Ids & route_lanelets_) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils_->getConflictingLaneIds(route_lanelets_);
  for (const auto & [name, status] : other_entity_status_) {
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
  auto conflicting_crosswalks = hdmap_utils_->getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = hdmap_utils_->getConflictingLaneIds(following_lanelets);
  for (const auto & [name, status] : other_entity_status_) {
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
  const double local_target_speed_,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time_, canonicalized_entity_status_->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed_, constraints, canonicalized_entity_status_->getTwist(),
    canonicalized_entity_status_->getAccel());

  const double linear_jerk_new = std::get<2>(dynamics);
  const geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  const geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status_->getCanonicalizedLaneletPose()) {
    const auto distance =
      (twist_new.linear.x + canonicalized_entity_status_->getTwist().linear.x) / 2.0 * step_time_;
    auto entity_status_updated =
      static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status_);
    entity_status_updated.time = current_time_ + step_time_;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    entity_status_updated.action_status.linear_jerk = linear_jerk_new;
    /// @todo it will be moved to route::moveAlongLaneletPose(...)
    entity_status_updated.lanelet_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
      static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value()),
      route_lanelets_, distance);
    entity_status_updated.lanelet_pose_valid = true;
    entity_status_updated.pose =
      traffic_simulator::pose::toMapPose(entity_status_updated.lanelet_pose);
    return entity_status_updated;
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot move along lanelet - entity ", std::quoted(canonicalized_entity_status_->getName()),
      " has invalid lanelet pose.");
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  const double local_target_speed_,
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
  }(canonicalized_entity_status_->getType());

  const auto matching_distance = default_matching_distance_for_lanelet_pose_calculation_;

  const auto build_updated_pose =
    [&include_crosswalk, &matching_distance](
      const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status,
      const geometry_msgs::msg::Twist & desired_twist, const double time_step) {
      geometry_msgs::msg::Pose updated_pose;

      /// @note Apply yaw change (delta rotation) in radians: yaw_angular_speed (rad/s) * time_step (s)
      geometry_msgs::msg::Vector3 delta_rotation;
      delta_rotation = desired_twist.angular * time_step;
      const auto delta_quaternion = math::geometry::convertEulerAngleToQuaternion(delta_rotation);
      updated_pose.orientation = status->getMapPose().orientation * delta_quaternion;

      /// @note Apply position change
      /// @todo first determine global desired_velocity, calculate position change using it
      /// then pass the same global desired_velocity to updatePositionForLaneletTransition()
      const Eigen::Matrix3d rotation_matrix =
        math::geometry::getRotationMatrix(updated_pose.orientation);
      const auto translation = Eigen::Vector3d(
        desired_twist.linear.x * time_step, desired_twist.linear.y * time_step,
        desired_twist.linear.z * time_step);
      const Eigen::Vector3d delta_position = rotation_matrix * translation;
      updated_pose.position = status->getMapPose().position + delta_position;

      /// @note If it is the transition between lanelets: overwrite position to improve precision
      if (const auto canonicalized_lanelet_pose = status->getCanonicalizedLaneletPose()) {
        const auto estimated_next_canonicalized_lanelet_pose =
          traffic_simulator::pose::toCanonicalizedLaneletPose(
            updated_pose, status->getBoundingBox(), include_crosswalk, matching_distance);
        if (estimated_next_canonicalized_lanelet_pose) {
          const auto next_lanelet_id = static_cast<traffic_simulator::LaneletPose>(
                                         estimated_next_canonicalized_lanelet_pose.value())
                                         .lanelet_id;
          if (  /// @note Handle lanelet transition
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
      step_time_, canonicalized_entity_status_->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed_, constraints, canonicalized_entity_status_->getTwist(),
    canonicalized_entity_status_->getAccel());
  const auto linear_jerk_new = std::get<2>(dynamics);
  const auto & accel_new = std::get<1>(dynamics);
  const auto & twist_new = std::get<0>(dynamics);
  const auto pose_new = build_updated_pose(canonicalized_entity_status_, twist_new, step_time_);

  auto entity_status_updated =
    static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status_);
  entity_status_updated.time = current_time_ + step_time_;
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
           step_time_, canonicalized_entity_status_->getName())
    .getRunningDistance(
      0, constraints, canonicalized_entity_status_->getTwist(),
      canonicalized_entity_status_->getAccel(), canonicalized_entity_status_->getLinearJerk());
}
}  // namespace entity_behavior
