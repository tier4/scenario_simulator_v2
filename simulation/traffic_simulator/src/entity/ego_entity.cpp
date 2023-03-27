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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/lexical_cast.hpp>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
auto toString(const VehicleModelType datum) -> std::string
{
#define BOILERPLATE(IDENTIFIER)      \
  case VehicleModelType::IDENTIFIER: \
    return #IDENTIFIER

  switch (datum) {
    BOILERPLATE(DELAY_STEER_ACC);
    BOILERPLATE(DELAY_STEER_ACC_GEARED);
    BOILERPLATE(DELAY_STEER_VEL);
    BOILERPLATE(IDEAL_STEER_ACC);
    BOILERPLATE(IDEAL_STEER_ACC_GEARED);
    BOILERPLATE(IDEAL_STEER_VEL);
  }

#undef BOILERPLATE

  THROW_SIMULATION_ERROR("Unsupported vehicle model type, failed to convert to string");
}

auto EgoEntity::getVehicleModelType() -> VehicleModelType
{
  const auto architecture_type = getParameter<std::string>("architecture_type", "awf/universe");

  const auto vehicle_model_type =
    getParameter<std::string>("vehicle_model_type", "IDEAL_STEER_VEL");

  static const std::unordered_map<std::string, VehicleModelType> table{
    {"DELAY_STEER_ACC", VehicleModelType::DELAY_STEER_ACC},
    {"DELAY_STEER_ACC_GEARED", VehicleModelType::DELAY_STEER_ACC_GEARED},
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

auto EgoEntity::makeSimulationModel(
  const VehicleModelType vehicle_model_type, const double step_time,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
  -> const std::shared_ptr<SimModelInterface>
{
  // clang-format off
  const auto acc_time_constant   = getParameter<double>("acc_time_constant",     0.1);
  const auto acc_time_delay      = getParameter<double>("acc_time_delay",        0.1);
  const auto steer_lim           = getParameter<double>("steer_lim",            parameters.axles.front_axle.max_steering);  // 1.0
  const auto steer_rate_lim      = getParameter<double>("steer_rate_lim",        5.0);
  const auto steer_time_constant = getParameter<double>("steer_time_constant",   0.27);
  const auto steer_time_delay    = getParameter<double>("steer_time_delay",      0.24);
  const auto vel_lim             = getParameter<double>("vel_lim",              parameters.performance.max_speed);  // 50.0
  const auto vel_rate_lim        = getParameter<double>("vel_rate_lim",         parameters.performance.max_acceleration);  // 7.0
  const auto vel_time_constant   = getParameter<double>("vel_time_constant",     0.1);
  const auto vel_time_delay      = getParameter<double>("vel_time_delay",        0.1);
  const auto wheel_base          = getParameter<double>("wheel_base",           parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x);
  // clang-format on

  switch (vehicle_model_type) {
    case VehicleModelType::DELAY_STEER_ACC:
      return std::make_shared<SimModelDelaySteerAcc>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant);

    case VehicleModelType::DELAY_STEER_ACC_GEARED:
      return std::make_shared<SimModelDelaySteerAccGeared>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant);

    case VehicleModelType::DELAY_STEER_VEL:
      return std::make_shared<SimModelDelaySteerVel>(
        vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheel_base, step_time, vel_time_delay,
        vel_time_constant, steer_time_delay, steer_time_constant);

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

auto EgoEntity::makeAutoware(const Configuration & configuration)
  -> std::unique_ptr<concealer::Autoware>
{
  if (const auto architecture_type = getParameter<std::string>("architecture_type", "awf/universe");
      architecture_type == "awf/universe") {
    std::string rviz_config = getParameter<std::string>("rviz_config", "");
    return getParameter<bool>("launch_autoware", true)
             ? std::make_unique<concealer::AutowareUniverse>(
                 getParameter<std::string>("autoware_launch_package"),
                 getParameter<std::string>("autoware_launch_file"),
                 "map_path:=" + configuration.map_path.string(),
                 "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                 "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                 "sensor_model:=" + getParameter<std::string>("sensor_model"),
                 "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                 "rviz_config:=" + ((rviz_config == "")
                                      ? configuration.rviz_config_path.string()
                                      : Configuration::Pathname(rviz_config).string()),
                 "scenario_simulation:=true", "perception/enable_traffic_light:=false")
             : std::make_unique<concealer::AutowareUniverse>();
  } else {
    throw common::SemanticError(
      "Unexpected architecture_type ", std::quoted(architecture_type), " was given.");
  }
}

EgoEntity::EgoEntity(
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const Configuration & configuration, const double step_time)
: VehicleEntity(name, entity_status, parameters),
  autoware(makeAutoware(configuration)),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters))
{
}

auto EgoEntity::asAutoware() const -> concealer::Autoware &
{
  assert(autoware);
  return *autoware;
}

auto EgoEntity::getCurrentAction() const -> std::string
{
  const auto state = autoware->getAutowareStateName();
  return state.empty() ? "Launching" : state;
}

auto EgoEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  traffic_simulator_msgs::msg::BehaviorParameter parameter;
  /**
   * @brief TODO, Input values get from autoware.
   */
  parameter.see_around = true;
  parameter.dynamic_constraints.max_acceleration = 0;
  parameter.dynamic_constraints.max_deceleration = 0;
  return parameter;
}

auto EgoEntity::getEntityStatus(const double time, const double step_time) const
  -> const traffic_simulator_msgs::msg::EntityStatus
{
  traffic_simulator_msgs::msg::EntityStatus status;
  {
    status.time = time;
    status.type = getStatus().type;
    status.bounding_box = getStatus().bounding_box;
    status.pose = getCurrentPose();
    status.action_status.twist = getCurrentTwist();
    status.action_status.accel = [&]() {
      geometry_msgs::msg::Accel accel;
      if (previous_angular_velocity_) {
        accel.linear.x = vehicle_model_ptr_->getAx();
        accel.angular.z =
          (vehicle_model_ptr_->getWz() - previous_angular_velocity_.get()) / step_time;
      }
      return accel;
    }();

    const auto unique_route_lanelets =
      traffic_simulator::helper::getUniqueValues(getRouteLanelets());

    boost::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;

    if (unique_route_lanelets.empty()) {
      lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(status.pose, getStatus().bounding_box, false, 1.0);
    } else {
      lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose, unique_route_lanelets, 1.0);
      if (!lanelet_pose) {
        lanelet_pose =
          hdmap_utils_ptr_->toLaneletPose(status.pose, getStatus().bounding_box, false, 1.0);
      }
    }

    if (lanelet_pose) {
      math::geometry::CatmullRomSpline spline(
        hdmap_utils_ptr_->getCenterPoints(lanelet_pose->lanelet_id));
      if (const auto s_value = spline.getSValue(status.pose)) {
        status.pose.position.z = spline.getPoint(s_value.get()).z;
      }
    }

    status.lanelet_pose_valid = static_cast<bool>(lanelet_pose);
    if (status.lanelet_pose_valid) {
      status.lanelet_pose = lanelet_pose.get();
    }
  }

  return status;
}

auto EgoEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "EgoEntity";
  return result;
}

auto EgoEntity::getObstacle() -> boost::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return boost::none;
}

auto EgoEntity::getRouteLanelets() const -> std::vector<std::int64_t>
{
  std::vector<std::int64_t> ids{};

  if (const auto universe = dynamic_cast<concealer::AutowareUniverse *>(autoware.get()); universe) {
    for (const auto & point : universe->getPathWithLaneId().points) {
      std::copy(point.lane_ids.begin(), point.lane_ids.end(), std::back_inserter(ids));
    }
  }

  return ids;
}

auto EgoEntity::getCurrentPose() const -> geometry_msgs::msg::Pose
{
  Eigen::VectorXd relative_position(3);
  relative_position(0) = vehicle_model_ptr_->getX();
  relative_position(1) = vehicle_model_ptr_->getY();
  relative_position(2) = 0.0;
  relative_position =
    quaternion_operation::getRotationMatrix(initial_pose_->orientation) * relative_position;

  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = initial_pose_.get().position.x + relative_position(0);
  current_pose.position.y = initial_pose_.get().position.y + relative_position(1);
  current_pose.position.z = initial_pose_.get().position.z + relative_position(2);
  current_pose.orientation = [this]() {
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0;
    rpy.y = 0;
    rpy.z = vehicle_model_ptr_->getYaw();
    return initial_pose_.get().orientation *
           quaternion_operation::convertEulerAngleToQuaternion(rpy);
  }();

  return current_pose;
}

auto EgoEntity::getCurrentTwist() const -> geometry_msgs::msg::Twist
{
  geometry_msgs::msg::Twist current_twist;
  current_twist.linear.x = vehicle_model_ptr_->getVx();
  current_twist.angular.z = vehicle_model_ptr_->getWz();
  return current_twist;
}

auto EgoEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return autoware->getWaypoints();
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  autoware->rethrow();

  EntityBase::onUpdate(current_time, step_time);

  if (npc_logic_started_) {
    Eigen::VectorXd input(vehicle_model_ptr_->getDimU());

    switch (vehicle_model_type_) {
      case VehicleModelType::DELAY_STEER_ACC:
      case VehicleModelType::IDEAL_STEER_ACC:
        input << autoware->getGearSign() * autoware->getAcceleration(),
          autoware->getSteeringAngle();
        break;

      case VehicleModelType::DELAY_STEER_ACC_GEARED:
      case VehicleModelType::IDEAL_STEER_ACC_GEARED:
        input << autoware->getGearSign() * autoware->getAcceleration(),
          autoware->getSteeringAngle();
        break;

      case VehicleModelType::DELAY_STEER_VEL:
      case VehicleModelType::IDEAL_STEER_VEL:
        input << autoware->getVelocity(), autoware->getSteeringAngle();
        break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported vehicle_model_type ", toString(vehicle_model_type_), "specified");
    }

    vehicle_model_ptr_->setGear(autoware->getGearCommand().command);
    vehicle_model_ptr_->setInput(input);
    vehicle_model_ptr_->update(step_time);
  }

  auto entity_status = getEntityStatus(current_time + step_time, step_time);
  if (previous_linear_velocity_) {
    entity_status.action_status.linear_jerk =
      (vehicle_model_ptr_->getVx() - previous_linear_velocity_.get()) / step_time;
  } else {
    entity_status.action_status.linear_jerk = 0;
  }
  setStatus(entity_status);
  updateStandStillDuration(step_time);
  updateTraveledDistance(step_time);

  previous_linear_velocity_ = vehicle_model_ptr_->getVx();
  previous_angular_velocity_ = vehicle_model_ptr_->getWz();

  EntityBase::onPostUpdate(current_time, step_time);
}

void EgoEntity::requestAcquirePosition(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  requestAssignRoute({lanelet_pose});
}

void EgoEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  requestAssignRoute({map_pose});
}

void EgoEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  std::vector<geometry_msgs::msg::Pose> route;

  for (const auto & waypoint : waypoints) {
    route.push_back((*hdmap_utils_ptr_).toMapPose(waypoint).pose);
  }

  requestAssignRoute(route);
}

void EgoEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<geometry_msgs::msg::PoseStamped> route;

  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    {
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose = waypoint;
    }

    route.push_back(pose_stamped);
  }

  if (not autoware->initialized()) {
    autoware->initialize(getStatus().pose);
    autoware->plan(route);
    // NOTE: engage() will be executed at simulation-time 0.
  } else {
    autoware->plan(route);
    autoware->engage();
  }
}

void EgoEntity::requestLaneChange(const std::int64_t)
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestSpeedChange(
  const double target_speed, const speed_change::Transition, const speed_change::Constraint,
  const bool) -> void
{
  requestSpeedChange(target_speed, false);
}

auto EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
  const speed_change::Constraint, const bool) -> void
{
  THROW_SEMANTIC_ERROR(
    "The traffic_simulator's request to set speed to the Ego type entity is for initialization "
    "purposes only.");
}

auto EgoEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  THROW_SEMANTIC_ERROR("getDefaultDynamicConstraints function does not support EgoEntity");
}

auto EgoEntity::setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) -> void
{
}

auto EgoEntity::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void
{
  VehicleEntity::setStatus(status);

  const auto current_pose = getStatus().pose;

  if (autoware->initialized()) {
    autoware->set([this]() {
      geometry_msgs::msg::Accel message;
      message.linear.x = vehicle_model_ptr_->getAx();
      return message;
    }());

    autoware->set(current_pose);

    autoware->set(getCurrentTwist());
  }

  if (not initial_pose_) {
    initial_pose_ = current_pose;
  }
}

void EgoEntity::requestSpeedChange(double value, bool)
{
  Eigen::VectorXd v(vehicle_model_ptr_->getDimX());

  switch (vehicle_model_type_) {
    case VehicleModelType::DELAY_STEER_ACC:
    case VehicleModelType::DELAY_STEER_ACC_GEARED:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0, 0;
      break;

    case VehicleModelType::IDEAL_STEER_ACC:
    case VehicleModelType::IDEAL_STEER_ACC_GEARED:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value);
      break;

    case VehicleModelType::IDEAL_STEER_VEL:
      v << 0, 0, 0;
      break;

    case VehicleModelType::DELAY_STEER_VEL:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0;
      break;

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported simulation model ", toString(vehicle_model_type_), " specified");
  }

  vehicle_model_ptr_->setState(v);
}

void EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & /*target_speed*/, bool /*continuous*/)
{
}

auto EgoEntity::setVelocityLimit(double value) -> void  //
{
  autoware->setVelocityLimit(value);
}
}  // namespace entity
}  // namespace traffic_simulator
