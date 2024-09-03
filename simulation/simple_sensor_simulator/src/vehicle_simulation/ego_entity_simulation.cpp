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

#include <concealer/autoware_universe.hpp>
#include <filesystem>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace vehicle_simulation
{
/// @todo find some shared space for this function
template <typename T>
static auto getParameter(const std::string & name, T value = {})
{
  rclcpp::Node node{"get_parameter", "simulation"};

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

EgoEntitySimulation::EgoEntitySimulation(
  const traffic_simulator_msgs::msg::EntityStatus & initial_status,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters, double step_time,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
  const rclcpp::Parameter & use_sim_time, const bool consider_acceleration_by_road_slope)
: autoware(std::make_unique<concealer::AutowareUniverse>()),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters)),
  status_(initial_status, std::nullopt),
  consider_acceleration_by_road_slope_(consider_acceleration_by_road_slope),
  hdmap_utils_ptr_(hdmap_utils),
  vehicle_parameters(parameters)
{
  setStatus(initial_status);
  initial_pose_ = status_.getMapPose();
  autoware->set_parameter(use_sim_time);
}

auto toString(const VehicleModelType datum) -> std::string
{
#define BOILERPLATE(IDENTIFIER)      \
  case VehicleModelType::IDENTIFIER: \
    return #IDENTIFIER

  switch (datum) {
    BOILERPLATE(DELAY_STEER_ACC);
    BOILERPLATE(DELAY_STEER_ACC_GEARED);
    BOILERPLATE(DELAY_STEER_MAP_ACC_GEARED);
    BOILERPLATE(DELAY_STEER_VEL);
    BOILERPLATE(IDEAL_STEER_ACC);
    BOILERPLATE(IDEAL_STEER_ACC_GEARED);
    BOILERPLATE(IDEAL_STEER_VEL);
  }

#undef BOILERPLATE

  THROW_SIMULATION_ERROR("Unsupported vehicle model type, failed to convert to string");
}

auto EgoEntitySimulation::getVehicleModelType() -> VehicleModelType
{
  const auto vehicle_model_type =
    getParameter<std::string>("vehicle_model_type", "IDEAL_STEER_VEL");

  static const std::unordered_map<std::string, VehicleModelType> table{
    {"DELAY_STEER_ACC", VehicleModelType::DELAY_STEER_ACC},
    {"DELAY_STEER_ACC_GEARED", VehicleModelType::DELAY_STEER_ACC_GEARED},
    {"DELAY_STEER_MAP_ACC_GEARED", VehicleModelType::DELAY_STEER_MAP_ACC_GEARED},
    {"DELAY_STEER_VEL", VehicleModelType::DELAY_STEER_VEL},
    {"IDEAL_STEER_ACC", VehicleModelType::IDEAL_STEER_ACC},
    {"IDEAL_STEER_ACC_GEARED", VehicleModelType::IDEAL_STEER_ACC_GEARED},
    {"IDEAL_STEER_VEL", VehicleModelType::IDEAL_STEER_VEL},
  };

  const auto iter = table.find(vehicle_model_type);

  if (iter != std::end(table)) {
    return iter->second;
  } else {
    THROW_SEMANTIC_ERROR("Unsupported vehicle_model_type ", vehicle_model_type, " specified");
  }
}

auto EgoEntitySimulation::makeSimulationModel(
  const VehicleModelType vehicle_model_type, const double step_time,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
  -> const std::shared_ptr<SimModelInterface>
{
  auto node = rclcpp::Node("get_parameter", "simulation");

  auto get_parameter = [&](const std::string & name, auto value = {}) {
    node.declare_parameter<decltype(value)>(name, value);
    node.get_parameter<decltype(value)>(name, value);
    return value;
  };
  // clang-format off
  const auto acc_time_constant          = get_parameter("acc_time_constant",           0.1);
  const auto acc_time_delay             = get_parameter("acc_time_delay",              0.1);
  const auto acceleration_map_path      = get_parameter("acceleration_map_path",       std::string(""));
  const auto debug_acc_scaling_factor   = get_parameter("debug_acc_scaling_factor",    1.0);
  const auto debug_steer_scaling_factor = get_parameter("debug_steer_scaling_factor",  1.0);
  const auto steer_lim                  = get_parameter("steer_lim",                   parameters.axles.front_axle.max_steering);  // 1.0
  const auto steer_dead_band            = get_parameter("steer_dead_band",             0.0);
  const auto steer_rate_lim             = get_parameter("steer_rate_lim",              5.0);
  const auto steer_time_constant        = get_parameter("steer_time_constant",         0.27);
  const auto steer_time_delay           = get_parameter("steer_time_delay",            0.24);
  const auto vel_lim                    = get_parameter("vel_lim",                     parameters.performance.max_speed);  // 50.0
  const auto vel_rate_lim               = get_parameter("vel_rate_lim",                parameters.performance.max_acceleration);  // 7.0
  const auto vel_time_constant          = get_parameter("vel_time_constant",           0.1);  /// @note 0.5 is default value on simple_planning_simulator
  const auto vel_time_delay             = get_parameter("vel_time_delay",              0.1);  /// @note 0.25 is default value on simple_planning_simulator
  const auto wheel_base                 = get_parameter("wheel_base",                  parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x);
  // clang-format on

  switch (vehicle_model_type) {
    case VehicleModelType::DELAY_STEER_ACC:
      return std::make_shared<SimModelDelaySteerAcc>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
        debug_acc_scaling_factor, debug_steer_scaling_factor);

    case VehicleModelType::DELAY_STEER_ACC_GEARED:
      return std::make_shared<SimModelDelaySteerAccGeared>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
        debug_acc_scaling_factor, debug_steer_scaling_factor);

    case VehicleModelType::DELAY_STEER_MAP_ACC_GEARED:
      if (!std::filesystem::exists(acceleration_map_path)) {
        throw std::runtime_error(
          "`acceleration_map_path` parameter is necessary for `DELAY_STEER_MAP_ACC_GEARED` "
          "simulator model, but " +
          acceleration_map_path +
          " does not exist. Please confirm that the parameter is set correctly.");
      }
      return std::make_shared<SimModelDelaySteerMapAccGeared>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant, acceleration_map_path);
    case VehicleModelType::DELAY_STEER_VEL:
      return std::make_shared<SimModelDelaySteerVel>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, vel_time_delay,
        vel_time_constant, steer_time_delay, steer_time_constant, steer_dead_band);

    case VehicleModelType::IDEAL_STEER_ACC:
      return std::make_shared<SimModelIdealSteerAcc>(wheel_base);

    case VehicleModelType::IDEAL_STEER_ACC_GEARED:
      return std::make_shared<SimModelIdealSteerAccGeared>(wheel_base);

    case VehicleModelType::IDEAL_STEER_VEL:
      return std::make_shared<SimModelIdealSteerVel>(wheel_base);

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported vehicle_model_type ", toString(vehicle_model_type), " specified");
  }
}

auto EgoEntitySimulation::setAutowareStatus() -> void
{
  autoware->set([this]() {
    geometry_msgs::msg::Accel message;
    message.linear.x = vehicle_model_ptr_->getAx();
    return message;
  }());

  autoware->set(status_.getMapPose());

  autoware->set(getCurrentTwist());
}

void EgoEntitySimulation::requestSpeedChange(double value)
{
  Eigen::VectorXd v(vehicle_model_ptr_->getDimX());

  switch (vehicle_model_type_) {
    case VehicleModelType::DELAY_STEER_ACC:
    case VehicleModelType::DELAY_STEER_ACC_GEARED:
    case VehicleModelType::DELAY_STEER_MAP_ACC_GEARED:
      v << 0, 0, 0, value, 0, 0;
      break;

    case VehicleModelType::IDEAL_STEER_ACC:
    case VehicleModelType::IDEAL_STEER_ACC_GEARED:
      v << 0, 0, 0, value;
      break;

    case VehicleModelType::IDEAL_STEER_VEL:
      v << 0, 0, 0;
      break;

    case VehicleModelType::DELAY_STEER_VEL:
      v << 0, 0, 0, value, 0;
      break;

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported simulation model ", toString(vehicle_model_type_), " specified");
  }

  vehicle_model_ptr_->setState(v);
}

auto EgoEntitySimulation::overwrite(
  const traffic_simulator_msgs::msg::EntityStatus & status, const double current_time,
  const double step_time, const bool is_npc_logic_started) -> void
{
  using math::geometry::convertQuaternionToEulerAngle;
  using math::geometry::getRotationMatrix;

  autoware->rethrow();

  /*
     SimModelInterface only supports 2D, therefore the position in Oz is
     considered unchangeable and stored in an additional variable
     world_relative_position_ that is used in calculations.
  */
  world_relative_position_ = getRotationMatrix(initial_pose_.orientation).transpose() *
                             Eigen::Vector3d(
                               status.pose.position.x - initial_pose_.position.x,
                               status.pose.position.y - initial_pose_.position.y,
                               status.pose.position.z - initial_pose_.position.z);

  if (is_npc_logic_started) {
    const auto yaw = [&]() {
      const auto q = Eigen::Quaterniond(
        getRotationMatrix(initial_pose_.orientation).transpose() *
        getRotationMatrix(status.pose.orientation));
      geometry_msgs::msg::Quaternion relative_orientation;
      relative_orientation.x = q.x();
      relative_orientation.y = q.y();
      relative_orientation.z = q.z();
      relative_orientation.w = q.w();
      return convertQuaternionToEulerAngle(relative_orientation).z -
             (previous_linear_velocity_ ? *previous_angular_velocity_ : 0) * step_time;
    }();

    switch (auto state = Eigen::VectorXd(vehicle_model_ptr_->getDimX()); vehicle_model_type_) {
      case VehicleModelType::DELAY_STEER_ACC:
      case VehicleModelType::DELAY_STEER_ACC_GEARED:
      case VehicleModelType::DELAY_STEER_MAP_ACC_GEARED:
        state(5) = status.action_status.accel.linear.x;
        [[fallthrough]];

      case VehicleModelType::DELAY_STEER_VEL:
        state(4) = 0;
        [[fallthrough]];

      case VehicleModelType::IDEAL_STEER_ACC:
      case VehicleModelType::IDEAL_STEER_ACC_GEARED:
        state(3) = status.action_status.twist.linear.x;
        [[fallthrough]];

      case VehicleModelType::IDEAL_STEER_VEL:
        state(0) = world_relative_position_.x();
        state(1) = world_relative_position_.y();
        state(2) = yaw;
        vehicle_model_ptr_->setState(state);
        break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported simulation model ", toString(vehicle_model_type_), " specified");
    }
  }
  updateStatus(current_time, step_time);
  updatePreviousValues();
}

void EgoEntitySimulation::update(
  const double current_time, const double step_time, const bool is_npc_logic_started)
{
  using math::geometry::getRotationMatrix;

  autoware->rethrow();

  /*
     SimModelInterface only supports 2D, therefore the position in Oz is
     considered unchangeable and stored in an additional variable
     world_relative_position_ that is used in calculations.
  */
  world_relative_position_ = getRotationMatrix(initial_pose_.orientation).transpose() *
                             Eigen::Vector3d(
                               status_.getMapPose().position.x - initial_pose_.position.x,
                               status_.getMapPose().position.y - initial_pose_.position.y,
                               status_.getMapPose().position.z - initial_pose_.position.z);

  if (is_npc_logic_started) {
    auto input = Eigen::VectorXd(vehicle_model_ptr_->getDimU());

    auto acceleration_by_slope = [this]() {
      if (consider_acceleration_by_road_slope_) {
        // calculate longitudinal acceleration by slope
        constexpr double gravity_acceleration = -9.81;
        const double ego_pitch_angle = calculateEgoPitch();
        const double slope_angle = -ego_pitch_angle;
        return gravity_acceleration * std::sin(slope_angle);
      } else {
        return 0.0;
      }
    }();

    switch (vehicle_model_type_) {
      case VehicleModelType::DELAY_STEER_ACC:
      case VehicleModelType::DELAY_STEER_ACC_GEARED:
      case VehicleModelType::DELAY_STEER_MAP_ACC_GEARED:
      case VehicleModelType::IDEAL_STEER_ACC:
      case VehicleModelType::IDEAL_STEER_ACC_GEARED:
        input(0) = autoware->getGearSign() * (autoware->getAcceleration() + acceleration_by_slope);
        input(1) = autoware->getSteeringAngle();
        break;

      case VehicleModelType::DELAY_STEER_VEL:
      case VehicleModelType::IDEAL_STEER_VEL:
        input(0) = autoware->getVelocity();
        input(1) = autoware->getSteeringAngle();
        break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported vehicle_model_type ", toString(vehicle_model_type_), "specified");
    }

    vehicle_model_ptr_->setGear(autoware->getGearCommand().command);
    vehicle_model_ptr_->setInput(input);
    vehicle_model_ptr_->update(step_time);
  }
  // only the position in the Oz axis is left unchanged, the rest is taken from SimModelInterface
  world_relative_position_.x() = vehicle_model_ptr_->getX();
  world_relative_position_.y() = vehicle_model_ptr_->getY();
  updateStatus(current_time, step_time);
  updatePreviousValues();
}

auto EgoEntitySimulation::calculateEgoPitch() const -> double
{
  // calculate prev/next point of lanelet centerline nearest to ego pose.
  if (!status_.laneMatchingSucceed()) {
    return 0.0;
  }

  /// @note Copied from motion_util::findNearestSegmentIndex
  auto centerline_points = hdmap_utils_ptr_->getCenterPoints(status_.getLaneletId());
  auto find_nearest_segment_index =
    [](const std::vector<geometry_msgs::msg::Point> & points, const Eigen::Vector3d & point) {
      assert(not points.empty());

      double min_dist = std::numeric_limits<double>::max();
      size_t min_idx = 0;

      for (size_t i = 0; i < points.size(); ++i) {
        const auto dist =
          [](const geometry_msgs::msg::Point & point1, const Eigen::Vector3d & point2) {
            const auto dx = point1.x - point2.x();
            const auto dy = point1.y - point2.y();
            return dx * dx + dy * dy;
          }(points.at(i), point);

        if (dist < min_dist) {
          min_dist = dist;
          min_idx = i;
        }
      }
      return min_idx;
    };

  const size_t ego_seg_idx =
    find_nearest_segment_index(centerline_points, world_relative_position_);

  const auto & prev_point = centerline_points.at(ego_seg_idx);
  const auto & next_point = centerline_points.at(ego_seg_idx + 1);

  /// @note Calculate ego yaw angle on lanelet coordinates
  const double lanelet_yaw = std::atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
  const double ego_yaw_against_lanelet = vehicle_model_ptr_->getYaw() - lanelet_yaw;

  /// @note calculate ego pitch angle considering ego yaw.
  const double diff_z = next_point.z - prev_point.z;
  const double diff_xy = std::hypot(next_point.x - prev_point.x, next_point.y - prev_point.y) /
                         std::cos(ego_yaw_against_lanelet);
  const bool reverse_sign = std::cos(ego_yaw_against_lanelet) < 0.0;
  const double ego_pitch_angle =
    reverse_sign ? -std::atan2(-diff_z, -diff_xy) : -std::atan2(diff_z, diff_xy);
  return ego_pitch_angle;
}

auto EgoEntitySimulation::getCurrentTwist() const -> geometry_msgs::msg::Twist
{
  geometry_msgs::msg::Twist current_twist;
  current_twist.linear.x = vehicle_model_ptr_->getVx();
  current_twist.angular.z = vehicle_model_ptr_->getWz();
  return current_twist;
}

auto EgoEntitySimulation::getCurrentPose(const double pitch_angle = 0.) const
  -> geometry_msgs::msg::Pose
{
  using math::geometry::operator*;
  const auto relative_position =
    math::geometry::getRotationMatrix(initial_pose_.orientation) * world_relative_position_;
  const auto relative_orientation = math::geometry::convertEulerAngleToQuaternion(
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(0)
      .y(pitch_angle)
      .z(vehicle_model_ptr_->getYaw()));

  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                .x(initial_pose_.position.x + relative_position(0))
                .y(initial_pose_.position.y + relative_position(1))
                .z(initial_pose_.position.z + relative_position(2)))
    .orientation(initial_pose_.orientation * relative_orientation);
}

auto EgoEntitySimulation::getCurrentAccel(const double step_time) const -> geometry_msgs::msg::Accel
{
  geometry_msgs::msg::Accel accel;
  if (previous_angular_velocity_) {
    accel.linear.x = vehicle_model_ptr_->getAx();
    accel.angular.z =
      (vehicle_model_ptr_->getWz() - previous_angular_velocity_.value()) / step_time;
  }
  return accel;
}

auto EgoEntitySimulation::getLinearJerk(double step_time) -> double
{
  // FIXME: This seems to be an acceleration, not jerk
  if (previous_linear_velocity_) {
    return (vehicle_model_ptr_->getVx() - previous_linear_velocity_.value()) / step_time;
  } else {
    return 0;
  }
}

auto EgoEntitySimulation::updatePreviousValues() -> void
{
  previous_linear_velocity_ = vehicle_model_ptr_->getVx();
  previous_angular_velocity_ = vehicle_model_ptr_->getWz();
}

auto EgoEntitySimulation::getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus
{
  return static_cast<traffic_simulator_msgs::msg::EntityStatus>(status_);
}

auto EgoEntitySimulation::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
  -> void
{
  /// @note The lanelet matching algorithm should be equivalent to the one used in
  /// EgoEntity::setStatus
  const auto unique_route_lanelets =
    traffic_simulator::helper::getUniqueValues(autoware->getRouteLanelets());
  const auto matching_distance = std::max(
                                   vehicle_parameters.axles.front_axle.track_width,
                                   vehicle_parameters.axles.rear_axle.track_width) *
                                   0.5 +
                                 1.0;
  /// @note Ego uses the unique_route_lanelets get from Autoware, instead of the current lanelet_id
  /// value from EntityStatus, therefore canonicalization has to be done in advance,
  /// not inside CanonicalizedEntityStatus
  const auto canonicalized_lanelet_pose = traffic_simulator::pose::toCanonicalizedLaneletPose(
    status.pose, status.bounding_box, unique_route_lanelets, false, matching_distance,
    hdmap_utils_ptr_);
  status_.set(traffic_simulator::CanonicalizedEntityStatus(status, canonicalized_lanelet_pose));
  setAutowareStatus();
}

auto EgoEntitySimulation::updateStatus(const double current_time, const double step_time) -> void
{
  auto status = static_cast<traffic_simulator_msgs::msg::EntityStatus>(status_);
  status.time = std::isnan(current_time) ? 0 : current_time;
  status.pose = getCurrentPose();
  status.action_status.twist = getCurrentTwist();
  status.action_status.accel = getCurrentAccel(step_time);
  status.action_status.linear_jerk = getLinearJerk(step_time);
  setStatus(status);
}
}  // namespace vehicle_simulation
