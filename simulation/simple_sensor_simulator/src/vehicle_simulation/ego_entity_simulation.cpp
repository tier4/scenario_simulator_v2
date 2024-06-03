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
#include <simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp>
#include <traffic_simulator/helper/helper.hpp>

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
  const traffic_simulator_msgs::msg::VehicleParameters & parameters, double step_time,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils,
  const rclcpp::Parameter & use_sim_time, const bool consider_acceleration_by_road_slope,
  const bool consider_pose_by_road_slope)
: autoware(std::make_unique<concealer::AutowareUniverse>()),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters)),
  hdmap_utils_ptr_(hdmap_utils),
  vehicle_parameters(parameters),
  consider_acceleration_by_road_slope_(consider_acceleration_by_road_slope),
  consider_pose_by_road_slope_(consider_pose_by_road_slope)
{
  autoware->set_parameter(use_sim_time);
  vehicle_model_state.emplace(*vehicle_model_ptr_, 0.0, std::nullopt);
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
  autoware->set(vehicle_model_state->getAccel());
  autoware->set(status_.pose);
  autoware->set(vehicle_model_state->getTwist());
}

void EgoEntitySimulation::requestSpeedChange(double value)
{
  Eigen::VectorXd v(vehicle_model_state->getOutputDimension());

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

void EgoEntitySimulation::overwrite(
  const traffic_simulator_msgs::msg::EntityStatus & status, double current_scenario_time,
  double step_time, bool npc_logic_started)
{
  autoware->rethrow();

  if (npc_logic_started) {
    using quaternion_operation::convertQuaternionToEulerAngle;
    using quaternion_operation::getRotationMatrix;

    switch (auto state = Eigen::VectorXd(vehicle_model_state->getOutputDimension());
            vehicle_model_type_) {
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
        state(0) = vehicle_model_state->getRelativePosition().x();
        state(1) = vehicle_model_state->getRelativePosition().y();
        state(2) = vehicle_model_state->getRelativeYaw(initial_pose_, status.pose, step_time);
        vehicle_model_ptr_->setState(state);
        break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported simulation model ", toString(vehicle_model_type_), " specified");
    }

    const auto world_relative_position = getRotationMatrix(initial_pose_.orientation).transpose() *
                                         Eigen::Vector3d(
                                           status.pose.position.x - initial_pose_.position.x,
                                           status.pose.position.y - initial_pose_.position.y,
                                           status.pose.position.z - initial_pose_.position.z);
    // world_relative_position is passed as the position is overwritten
    vehicle_model_state.emplace(*vehicle_model_ptr_, world_relative_position, vehicle_model_state);
  }
  updateStatus(current_scenario_time, step_time);
}

void EgoEntitySimulation::update(
  double current_scenario_time, double step_time, bool npc_logic_started)
{
  autoware->rethrow();

  if (npc_logic_started) {
    using quaternion_operation::getRotationMatrix;

    auto input = Eigen::VectorXd(vehicle_model_state->getInputDimension());

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

    const auto world_relative_position = getRotationMatrix(initial_pose_.orientation).transpose() *
                                         Eigen::Vector3d(
                                           status_.pose.position.x - initial_pose_.position.x,
                                           status_.pose.position.y - initial_pose_.position.y,
                                           status_.pose.position.z - initial_pose_.position.z);
    /*
       SimModelInterface only supports 2D, therefore the position in Oz is
       considered unchangeable so world_relative_position.z() is passed
    */
    vehicle_model_state.emplace(
      *vehicle_model_ptr_, world_relative_position.z(), vehicle_model_state);
  }
  updateStatus(current_scenario_time, step_time);
}

auto EgoEntitySimulation::getMatchedLaneletPoseFromEntityStatus(
  const traffic_simulator_msgs::msg::EntityStatus & status, const double entity_width) const
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  /// @note The lanelet matching algorithm should be equivalent to the one used in
  /// EgoEntity::setMapPose
  const auto unique_route_lanelets =
    traffic_simulator::helper::getUniqueValues(autoware->getRouteLanelets());
  const auto matching_distance = std::max(
                                   vehicle_parameters.axles.front_axle.track_width,
                                   vehicle_parameters.axles.rear_axle.track_width) *
                                   0.5 +
                                 1.0;

  std::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;

  if (unique_route_lanelets.empty()) {
    lanelet_pose =
      hdmap_utils_ptr_->toLaneletPose(status.pose, status.bounding_box, false, matching_distance);
  } else {
    lanelet_pose =
      hdmap_utils_ptr_->toLaneletPose(status.pose, unique_route_lanelets, matching_distance);
    if (!lanelet_pose) {
      lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(status.pose, status.bounding_box, false, matching_distance);
    }
  }
  return lanelet_pose;
}

auto EgoEntitySimulation::calculateEgoPitch() const -> double
{
  // calculate prev/next point of lanelet centerline nearest to ego pose.
  auto ego_lanelet = getMatchedLaneletPoseFromEntityStatus(
    status_, std::max(
               vehicle_parameters.axles.front_axle.track_width,
               vehicle_parameters.axles.rear_axle.track_width));
  if (not ego_lanelet) {
    return 0.0;
  }

  auto centerline_points = hdmap_utils_ptr_->getCenterPoints(ego_lanelet.value().lanelet_id);

  /// @note Copied from motion_util::findNearestSegmentIndex
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
    find_nearest_segment_index(centerline_points, vehicle_model_state->getRelativePosition());

  const auto & prev_point = centerline_points.at(ego_seg_idx);
  const auto & next_point = centerline_points.at(ego_seg_idx + 1);

  /// @note Calculate ego yaw angle on lanelet coordinates
  const double lanelet_yaw = std::atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
  const double ego_yaw_against_lanelet = vehicle_model_state->getYaw() - lanelet_yaw;

  /// @note calculate ego pitch angle considering ego yaw.
  const double diff_z = next_point.z - prev_point.z;
  const double diff_xy = std::hypot(next_point.x - prev_point.x, next_point.y - prev_point.y) /
                         std::cos(ego_yaw_against_lanelet);
  const bool reverse_sign = std::cos(ego_yaw_against_lanelet) < 0.0;
  const double ego_pitch_angle =
    reverse_sign ? -std::atan2(-diff_z, -diff_xy) : -std::atan2(diff_z, diff_xy);
  return ego_pitch_angle;
}

auto EgoEntitySimulation::getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus &
{
  return status_;
}

auto EgoEntitySimulation::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
  -> void
{
  status_ = status;
  setAutowareStatus();
}

auto EgoEntitySimulation::setInitialStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
  -> void
{
  setStatus(status);
  initial_pose_ = status_.pose;
}

auto EgoEntitySimulation::updateStatus(double current_scenario_time, double step_time) -> void
{
  traffic_simulator_msgs::msg::EntityStatus status;
  status.name = status_.name;
  status.time = std::isnan(current_scenario_time) ? 0 : current_scenario_time;
  status.type = status_.type;
  status.bounding_box = status_.bounding_box;
  status.pose = vehicle_model_state->getPose(initial_pose_);
  status.action_status.twist = vehicle_model_state->getTwist();
  status.action_status.accel = vehicle_model_state->getAccel(step_time);
  status.action_status.linear_jerk = vehicle_model_state->getLinearJerk(step_time);

  fillLaneletDataAndSnapZToLanelet(status);
  setStatus(status);
}

auto EgoEntitySimulation::fillLaneletDataAndSnapZToLanelet(
  traffic_simulator_msgs::msg::EntityStatus & status) -> void
{
  if (
    auto lanelet_pose = getMatchedLaneletPoseFromEntityStatus(
      status, std::max(
                vehicle_parameters.axles.front_axle.track_width,
                vehicle_parameters.axles.rear_axle.track_width))) {
    math::geometry::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(lanelet_pose->lanelet_id));
    if (const auto s_value = spline.getSValue(status.pose)) {
      status.pose.position.z = spline.getPoint(s_value.value()).z;
      if (consider_pose_by_road_slope_) {
        const auto rpy = quaternion_operation::convertQuaternionToEulerAngle(
          spline.getPose(s_value.value(), true).orientation);
        const auto original_rpy =
          quaternion_operation::convertQuaternionToEulerAngle(status.pose.orientation);
        status.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
          geometry_msgs::build<geometry_msgs::msg::Vector3>()
            .x(original_rpy.x)
            .y(rpy.y)
            .z(original_rpy.z));
        lanelet_pose->rpy =
          quaternion_operation::convertQuaternionToEulerAngle(quaternion_operation::getRotation(
            spline.getPose(s_value.value(), true).orientation, status.pose.orientation));
      }
    }
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.lanelet_pose_valid = false;
  }
}
}  // namespace vehicle_simulation
