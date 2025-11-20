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
#include <cmath>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/operator.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
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
  if (!getInput<std::optional<double>>(
        "lateral_collision_threshold", lateral_collision_threshold_)) {
    lateral_collision_threshold_ = std::nullopt;  // default: not set
  }
}

auto ActionNode::getOtherEntitiesCanonicalizedLaneletPoses() const
  -> std::vector<traffic_simulator::CanonicalizedLaneletPose>
{
  std::vector<traffic_simulator::CanonicalizedLaneletPose> other_canonicalized_lanelet_poses;
  for (const auto & [name, status] : other_entity_status_) {
    if (auto const & canonicalized_lanelet_pose = status.getCanonicalizedLaneletPose()) {
      other_canonicalized_lanelet_poses.push_back(canonicalized_lanelet_pose.value());
    }
  }
  return other_canonicalized_lanelet_poses;
}

auto ActionNode::getOtherEntitiesCanonicalizedEntityStatuses() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> other_status;
  other_status.reserve(other_entity_status_.size());
  for (const auto & [entity_name, entity_status] : other_entity_status_) {
    other_status.push_back(entity_status);
  }
  return other_status;
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

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status_->getCanonicalizedLaneletPose()) {
    if (const auto other_canonicalized_lanelet_poses = getOtherEntitiesCanonicalizedLaneletPoses();
        !other_canonicalized_lanelet_poses.empty()) {
      return traffic_simulator::distance::distanceToYieldStop(
        canonicalized_lanelet_pose.value(), following_lanelets, other_canonicalized_lanelet_poses);
    }
  }
  return std::nullopt;
}

/// @todo it will be moved to traffic_simulator::route::isNeedToRightOfWay(...)
auto ActionNode::isNeedToRightOfWay(const lanelet::Ids & following_lanelets) const -> bool
{
  auto isTheSameRightOfWay =
    [&](const std::int64_t & lanelet_id, const std::int64_t & following_lanelet) {
      const auto right_of_way_lanelet_ids =
        traffic_simulator::lanelet_wrapper::lanelet_map::rightOfWayLaneletIds(lanelet_id);
      const auto the_same_right_of_way_it = std::find(
        right_of_way_lanelet_ids.begin(), right_of_way_lanelet_ids.end(), following_lanelet);
      return the_same_right_of_way_it != std::end(right_of_way_lanelet_ids);
    };

  const auto lanelet_ids_list =
    traffic_simulator::lanelet_wrapper::lanelet_map::rightOfWayLaneletIds(following_lanelets);
  for (const auto & pose : getOtherEntitiesCanonicalizedLaneletPoses()) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const lanelet::Id lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          isSameLaneletId(pose, lanelet_id) &&
          not isTheSameRightOfWay(lanelet_id, following_lanelet)) {
          return true;
        }
      }
    }
  }
  return false;
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (not canonicalized_entity_status_->isInLanelet()) {
    return std::nullopt;
  }
  if (const auto front_entity_name = getFrontEntityName(spline)) {
    const auto & front_entity_status = getEntityStatus(front_entity_name.value());
    if (
      const auto & front_entity_canonicalized_lanelet_pose =
        front_entity_status.getCanonicalizedLaneletPose()) {
      return traffic_simulator::distance::splineDistanceToBoundingBox(
        spline, canonicalized_entity_status_->getCanonicalizedLaneletPose().value(),
        canonicalized_entity_status_->getBoundingBox(),
        front_entity_canonicalized_lanelet_pose.value(), front_entity_status.getBoundingBox(),
        lateral_collision_threshold_.value_or(-1.0));
    }
  }
  return std::nullopt;
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  if (not canonicalized_entity_status_->isInLanelet()) {
    return std::nullopt;
  }
  /**
   * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a
   * candidate of front entity.
   */
  constexpr double front_entity_angle_threshold{boost::math::constants::half_pi<double>()};

  if (euclidean_distances_map_ != nullptr) {
    std::map<double, std::string> local_euclidean_distances_map;
    const double stop_distance = calculateStopDistance(behavior_parameter_.dynamic_constraints);
    const double horizon = spline.getLength() > stop_distance ? spline.getLength() : stop_distance;
    for (const auto & [name_pair, euclidean_distance] : *euclidean_distances_map_) {
      /**
       * @note Euclidean distance is here used as a "rough" distance to filter only NPCs which possibly are in range of current horizon. Because euclidean distance is the shortest possible distance comparing it with horizon will never omit NPCs for which actual lane distance is in range of horizon.
       */
      if (euclidean_distance < horizon) {
        if (name_pair.first == canonicalized_entity_status_->getName()) {
          local_euclidean_distances_map.emplace(euclidean_distance, name_pair.second);
        } else if (name_pair.second == canonicalized_entity_status_->getName()) {
          local_euclidean_distances_map.emplace(euclidean_distance, name_pair.first);
        }
      }
    }

    const auto self_pos = canonicalized_entity_status_->getMapPose().position;
    const auto self_yaw = math::geometry::convertQuaternionToEulerAngle(
                            canonicalized_entity_status_->getMapPose().orientation)
                            .z;
    // Iterate by increasing euclidean distance and compute the
    // actual spline-based distance using getDistanceToTargetEntity.
    // Select the entity that yields the minimal valid distance.
    std::optional<std::string> best_name;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & [euclidean_distance, name] : local_euclidean_distances_map) {
      const auto & other_status = other_entity_status_.at(name);
      if (
        const auto & other_canonicalized_lanelet_pose =
          other_status.getCanonicalizedLaneletPose()) {
        const auto other_pos = other_status.getMapPose().position;
        const auto dx = other_pos.x - self_pos.x;
        const auto dy = other_pos.y - self_pos.y;
        const auto vec_yaw = std::atan2(dy, dx);
        const auto yaw_diff =
          std::atan2(std::sin(vec_yaw - self_yaw), std::cos(vec_yaw - self_yaw));
        if (std::fabs(yaw_diff) <= front_entity_angle_threshold) {
          if (const auto distance_opt = traffic_simulator::distance::splineDistanceToBoundingBox(
                spline, canonicalized_entity_status_->getCanonicalizedLaneletPose().value(),
                canonicalized_entity_status_->getBoundingBox(),
                other_canonicalized_lanelet_pose.value(), other_status.getBoundingBox());
              distance_opt.has_value()) {
            const auto dist = distance_opt.value();
            if (dist < best_distance) {
              best_distance = dist;
              best_name = name;
            }
          }
        }
      }
    }
    return best_name;
  }
  return std::nullopt;
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> const traffic_simulator::CanonicalizedEntityStatus &
{
  if (auto it = other_entity_status_.find(target_name); it != other_entity_status_.end()) {
    return it->second;
  } else {
    THROW_SEMANTIC_ERROR("Other entity ", std::quoted(target_name), " does not exist.");
  }
}

namespace
{
namespace bg = boost::geometry;
using BoostPolygon = math::geometry::boost_polygon;

struct QuadrilateralData
{
  std::vector<BoostPolygon> polygons;
  std::vector<double> longitudinal_starts;
};

auto buildQuadrilateralData(
  const math::geometry::CatmullRomSpline & spline, const double width,
  const std::size_t num_segments) -> QuadrilateralData
{
  QuadrilateralData data;
  if (num_segments == 0) {
    return data;
  }

  const double total_length = spline.getLength();
  const double step_size =
    num_segments > 0 ? total_length / static_cast<double>(num_segments) : 0.0;

  // Helper that computes the left/right boundary points from the spline center at distance s.
  const auto compute_bound_point = [&](const double s, const double direction) {
    geometry_msgs::msg::Vector3 normal_vector = spline.getNormalVector(s);
    const double theta = std::atan2(normal_vector.y, normal_vector.x);
    geometry_msgs::msg::Point center_point = spline.getPoint(s);
    geometry_msgs::msg::Point bound_point;
    bound_point.x = center_point.x + direction * 0.5 * width * std::cos(theta);
    bound_point.y = center_point.y + direction * 0.5 * width * std::sin(theta);
    bound_point.z = center_point.z;
    return bound_point;
  };

  // Precompute left/right boundary coordinates for every segment endpoint.
  std::vector<geometry_msgs::msg::Point> left_bounds;
  std::vector<geometry_msgs::msg::Point> right_bounds;
  left_bounds.reserve(num_segments + 1);
  right_bounds.reserve(num_segments + 1);
  for (std::size_t i = 0; i <= num_segments; ++i) {
    const double s = step_size * static_cast<double>(i);
    right_bounds.emplace_back(compute_bound_point(s, 1.0));
    left_bounds.emplace_back(compute_bound_point(s, -1.0));
  }

  data.polygons.reserve(num_segments);
  data.longitudinal_starts.reserve(num_segments);
  for (std::size_t i = 0; i < num_segments; ++i) {
    data.longitudinal_starts.emplace_back(step_size * static_cast<double>(i));

    BoostPolygon polygon;
    auto & outer = polygon.outer();
    outer.reserve(5);
    // Add a closing point because Boost.Geometry requires the first and last points to match.
    outer.emplace_back(right_bounds[i].x, right_bounds[i].y);
    outer.emplace_back(left_bounds[i].x, left_bounds[i].y);
    outer.emplace_back(left_bounds[i + 1].x, left_bounds[i + 1].y);
    outer.emplace_back(right_bounds[i + 1].x, right_bounds[i + 1].y);
    outer.push_back(outer.front());
    bg::correct(polygon);
    data.polygons.emplace_back(std::move(polygon));
  }
  return data;
}

auto intersectsTrajectory(
  const QuadrilateralData & trajectory_polygons, const BoostPolygon & target_polygon)
  -> std::optional<double>
{
  const auto num = trajectory_polygons.polygons.size();
  for (std::size_t i = 0; i < num; ++i) {
    if (bg::intersects(trajectory_polygons.polygons.at(i), target_polygon)) {
      return trajectory_polygons.longitudinal_starts.at(i);
    }
  }
  return std::nullopt;
}

auto detectEntityCollisions(
  const QuadrilateralData & data,
  const std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> &
    other_entity_status,
  const std::string & entity_name) -> std::optional<std::pair<std::string, double>>
{
  if (data.polygons.empty()) {
    return std::nullopt;
  }

  std::optional<std::pair<std::string, double>> nearest_collision;
  double nearest_range = std::numeric_limits<double>::infinity();

  for (const auto & [name, status] : other_entity_status) {
    if (entity_name == name) {
      continue;
    }
    // Generate a 2D polygon from the entity pose and bounding box.
    auto polygon = math::geometry::toPolygon2D(status.getMapPose(), status.getBoundingBox());
    if (polygon.outer().size() < 3) {
      continue;
    }

    // Determine whether it intersects the candidate trajectory and accumulate the result.
    if (const auto range = intersectsTrajectory(data, polygon)) {
      const double start = range.value();
      if (start < nearest_range) {
        nearest_range = start;
        nearest_collision = std::make_pair(name, start);
      }
    }
  }

  return nearest_collision;
}

}  // namespace

auto ActionNode::getFrontEntityNameAndDistanceByTrajectory(
  const std::vector<geometry_msgs::msg::Point> & waypoints, const double width,
  const std::size_t num_segments) const -> std::optional<std::pair<std::string, double>>
{
  if (waypoints.size() < 2) {
    return std::nullopt;
  }
  const math::geometry::CatmullRomSpline spline(waypoints);
  const auto quadrilateral_data =
    buildQuadrilateralData(spline, std::max(0.0, width), num_segments);
  if (
    const auto collision = detectEntityCollisions(
      quadrilateral_data, other_entity_status_, canonicalized_entity_status_->getName())) {
    return collision;
  }
  return std::nullopt;
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
