// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
    BOILERPLATE(LEGACY_DELAY_STEER);
    BOILERPLATE(LEGACY_DELAY_STEER_ACC);
    BOILERPLATE(LEGACY_IDEAL_ACCEL);
    BOILERPLATE(LEGACY_IDEAL_STEER);
  }

#undef BOILERPLATE

  THROW_SIMULATION_ERROR("Unsupported vehicle model type, failed to convert to string");
}

auto getVehicleModelType()
{
  const auto architecture_type = getParameter<std::string>("architecture_type", "tier4/proposal");

  const auto vehicle_model_type =
    architecture_type == "awf/auto"
      ? "IDEAL_STEER"
      : getParameter<std::string>("vehicle_model_type", "IDEAL_STEER");

  static const std::unordered_map<std::string, VehicleModelType> table{
    {"DELAY_STEER_ACC", VehicleModelType::DELAY_STEER_ACC},
    {"DELAY_STEER_ACC_GEARED", VehicleModelType::DELAY_STEER_ACC_GEARED},
    {"DELAY_STEER_VEL", VehicleModelType::DELAY_STEER_VEL},
    {"IDEAL_STEER_ACC", VehicleModelType::IDEAL_STEER_ACC},
    {"IDEAL_STEER_ACC_GEARED", VehicleModelType::IDEAL_STEER_ACC_GEARED},
    {"IDEAL_STEER_VEL", VehicleModelType::IDEAL_STEER_VEL},
  };

  static const std::unordered_map<std::string, VehicleModelType> legacy_table{
    {"DELAY_STEER", VehicleModelType::LEGACY_DELAY_STEER},
    {"DELAY_STEER_ACC", VehicleModelType::LEGACY_DELAY_STEER_ACC},
    {"IDEAL_ACCEL", VehicleModelType::LEGACY_IDEAL_ACCEL},
    {"IDEAL_STEER", VehicleModelType::LEGACY_IDEAL_STEER},
  };

  const auto iter =
    (architecture_type == "tier4/proposal" or architecture_type == "awf/auto" ? legacy_table
                                                                              : table)
      .find(vehicle_model_type);

  if (iter != std::end(table)) {
    return iter->second;
  } else {
    THROW_SEMANTIC_ERROR("Unsupported vehicle_model_type ", vehicle_model_type, " specified");
  }
}

auto makeSimulationModel(
  const VehicleModelType vehicle_model_type,
  const double step_time,  //
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
  -> const std::shared_ptr<SimModelInterface>
{
  // clang-format off
  const auto acc_time_constant    = getParameter<double>("acc_time_constant",     0.1);
  const auto acc_time_delay       = getParameter<double>("acc_time_delay",        0.1);
  const auto accel_rate           = getParameter<double>("accel_rate",           10.0);
  const auto deadzone_delta_steer = getParameter<double>("deadzone_delta_steer",  0.0);
  const auto steer_lim            = getParameter<double>("steer_lim",            parameters.axles.front_axle.max_steering);  // 1.0
  const auto steer_rate_lim       = getParameter<double>("steer_rate_lim",        5.0);
  const auto steer_time_constant  = getParameter<double>("steer_time_constant",   0.27);
  const auto steer_time_delay     = getParameter<double>("steer_time_delay",      0.24);
  const auto vel_lim              = getParameter<double>("vel_lim",              parameters.performance.max_speed);  // 50.0
  const auto vel_rate_lim         = getParameter<double>("vel_rate_lim",         parameters.performance.max_acceleration);  // 7.0
  const auto vel_time_constant    = getParameter<double>("vel_time_constant",     0.1);
  const auto vel_time_delay       = getParameter<double>("vel_time_delay",        0.1);
  const auto wheel_base           = getParameter<double>("wheel_base",           parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x);
  // clang-format on

  switch (vehicle_model_type) {
    case VehicleModelType::IDEAL_STEER_VEL:
    case VehicleModelType::LEGACY_IDEAL_STEER:
      return std::make_shared<SimModelIdealSteerVel>(wheel_base);

    case VehicleModelType::IDEAL_STEER_ACC:
    case VehicleModelType::LEGACY_IDEAL_ACCEL:
      return std::make_shared<SimModelIdealSteerAcc>(wheel_base);

    case VehicleModelType::IDEAL_STEER_ACC_GEARED:
      return std::make_shared<SimModelIdealSteerAccGeared>(wheel_base);

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

    case VehicleModelType::LEGACY_DELAY_STEER:
      return std::make_shared<SimModelTimeDelaySteer>(
        vel_lim, steer_lim, accel_rate, steer_rate_lim, wheel_base, step_time, vel_time_delay,
        vel_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);

    case VehicleModelType::LEGACY_DELAY_STEER_ACC:
      return std::make_shared<SimModelTimeDelaySteerAccel>(
        vel_lim, steer_lim, accel_rate, steer_rate_lim, wheel_base, step_time, acc_time_delay,
        acc_time_constant, steer_time_delay, steer_time_constant, deadzone_delta_steer);

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported vehicle_model_type ", toString(vehicle_model_type), " specified");
  }
}

auto makeAutoware(const Configuration & configuration) -> std::unique_ptr<concealer::Autoware>
{
  const auto architecture_type = getParameter<std::string>("architecture_type", "tier4/proposal");

  if (architecture_type == "tier4/proposal") {
    return getParameter<bool>("launch_autoware", true)
             ? std::make_unique<concealer::AutowareArchitectureProposal>(
                 getParameter<std::string>("autoware_launch_package"),
                 getParameter<std::string>("autoware_launch_file"),
                 "map_path:=" + configuration.map_path.string(),
                 "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                 "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                 "sensor_model:=" + getParameter<std::string>("sensor_model"),
                 "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                 "rviz_config:=" + configuration.rviz_config_path.string(),
                 "scenario_simulation:=true")
             : std::make_unique<concealer::AutowareArchitectureProposal>();
  } else if (architecture_type == "awf/universe") {
    return getParameter<bool>("launch_autoware", true)
             ? std::make_unique<concealer::AutowareUniverse>(
                 getParameter<std::string>("autoware_launch_package"),
                 getParameter<std::string>("autoware_launch_file"),
                 "map_path:=" + configuration.map_path.string(),
                 "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                 "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                 "sensor_model:=" + getParameter<std::string>("sensor_model"),
                 "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                 "rviz_config:=" + configuration.rviz_config_path.string(),
                 "scenario_simulation:=true")
             : std::make_unique<concealer::AutowareUniverse>();
  } else if (architecture_type == "awf/auto") {
    return getParameter<bool>("launch_autoware", true)
             ? std::make_unique<concealer::AutowareAuto>(
                 getParameter<std::string>("autoware_launch_package"),
                 getParameter<std::string>("autoware_launch_file"),
                 "map_path:=" + configuration.map_path.string(),
                 "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                 "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                 "sensor_model:=" + getParameter<std::string>("sensor_model"),
                 "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                 "rviz_config:=" + configuration.rviz_config_path.string(),
                 "scenario_simulation:=true")
             : std::make_unique<concealer::AutowareAuto>();
  } else {
    throw std::invalid_argument("Invalid architecture_type = " + architecture_type);
  }
}

EgoEntity::EgoEntity(
  const std::string & name,             //
  const Configuration & configuration,  //
  const double step_time,               //
  const traffic_simulator_msgs::msg::VehicleParameters & parameters)
: VehicleEntity(name, parameters),
  autoware(makeAutoware(configuration)),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters))
{
  entity_type_.type = traffic_simulator_msgs::msg::EntityType::EGO;
}

void EgoEntity::engage() { autoware->engage(); }

auto EgoEntity::getVehicleCommand() -> const autoware_vehicle_msgs::msg::VehicleCommand
{
  return autoware->getVehicleCommand();
}

auto EgoEntity::getCurrentAction() const -> const std::string
{
  const auto state = autoware->getAutowareStateMessage();
  return state.empty() ? "Launching" : state;
}

auto EgoEntity::getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel
{
  traffic_simulator_msgs::msg::DriverModel model;
  /**
   * @brief TODO, Input values get from autoware.
   */
  model.see_around = true;
  model.acceleration = 0;
  model.deceleration = 0;
  return model;
}

auto EgoEntity::getEntityStatus(const double time, const double step_time) const
  -> const traffic_simulator_msgs::msg::EntityStatus
{
  geometry_msgs::msg::Vector3 rpy;
  {
    rpy.x = 0;
    rpy.y = 0;
    rpy.z = vehicle_model_ptr_->getYaw();
  }

  geometry_msgs::msg::Pose pose;
  {
    pose.position.x = vehicle_model_ptr_->getX();
    pose.position.y = vehicle_model_ptr_->getY();
    pose.position.z = 0.0;
    pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
  }

  geometry_msgs::msg::Twist twist;
  {
    twist.linear.x = vehicle_model_ptr_->getVx();
    twist.angular.z = vehicle_model_ptr_->getWz();
  }

  geometry_msgs::msg::Accel accel;
  {
    if (previous_angular_velocity_ && previous_linear_velocity_) {
      accel.linear.x = (twist.linear.x - previous_linear_velocity_.get()) / step_time;
      accel.angular.z = (twist.angular.z - previous_angular_velocity_.get()) / step_time;
    }
  }

  Eigen::VectorXd v(3);
  {
    v(0) = pose.position.x;
    v(1) = pose.position.y;
    v(2) = pose.position.z;

    v = quaternion_operation::getRotationMatrix((*initial_pose_).orientation) * v;
  }

  traffic_simulator_msgs::msg::EntityStatus status;
  {
    status.time = time;
    status.type.type = entity_type_.type;
    status.bounding_box = getBoundingBox();
    status.action_status.twist = twist;
    status.action_status.accel = accel;
    status.pose.position.x = v(0) + initial_pose_.get().position.x;
    status.pose.position.y = v(1) + initial_pose_.get().position.y;
    status.pose.position.z = v(2) + initial_pose_.get().position.z;

    const auto closest_lanelet_id = hdmap_utils_ptr_->getClosestLaneletId(status.pose);
    if (!closest_lanelet_id) {
      THROW_SEMANTIC_ERROR("failed to find the closest lane, lane is too far away.");
    }

    traffic_simulator::math::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(closest_lanelet_id.get()));
    if (const auto s_value = spline.getSValue(status.pose)) {
      status.pose.position.z = spline.getPoint(s_value.get()).z;
    }

    status.pose.orientation = initial_pose_.get().orientation * pose.orientation;

    const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose, getBoundingBox(), false);

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

auto EgoEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return autoware->getWaypoints();
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (current_time < 0) {
    updateEntityStatusTimestamp(current_time);

    geometry_msgs::msg::Pose current_pose;
    {
      geometry_msgs::msg::Vector3 rpy;
      {
        rpy.x = 0;
        rpy.y = 0;
        rpy.z = vehicle_model_ptr_->getYaw();
      }

      current_pose.position.x = (*vehicle_model_ptr_).getX() + initial_pose_.get().position.x;
      current_pose.position.y = (*vehicle_model_ptr_).getY() + initial_pose_.get().position.y;
      current_pose.position.z = /*                          */ initial_pose_.get().position.z;

      current_pose.orientation =
        initial_pose_.get().orientation * quaternion_operation::convertEulerAngleToQuaternion(rpy);
    }

    autoware->set(current_pose);

    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    autoware->set(current_twist);
  } else {
    Eigen::VectorXd input(vehicle_model_ptr_->getDimU());

    switch (vehicle_model_type_) {
      case VehicleModelType::DELAY_STEER_ACC:
      case VehicleModelType::IDEAL_STEER_ACC:
      case VehicleModelType::LEGACY_IDEAL_ACCEL:
        input << autoware->getAcceleration(), autoware->getSteeringAngle();
        break;

      case VehicleModelType::DELAY_STEER_ACC_GEARED:
      case VehicleModelType::IDEAL_STEER_ACC_GEARED:
        input << autoware->getGearSign() * autoware->getAcceleration(),
          autoware->getSteeringAngle();
        break;

      case VehicleModelType::DELAY_STEER_VEL:
      case VehicleModelType::IDEAL_STEER_VEL:
      case VehicleModelType::LEGACY_IDEAL_STEER:
        input << autoware->getVelocity(), autoware->getSteeringAngle();
        break;

      case VehicleModelType::LEGACY_DELAY_STEER:
        input << autoware->getVelocity(), autoware->getSteeringAngle();
        break;

      case VehicleModelType::LEGACY_DELAY_STEER_ACC:
        input << autoware->getAcceleration(), autoware->getSteeringAngle(), autoware->getGearSign();
        break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported vehicle_model_type ", toString(vehicle_model_type_), "specified");
    }

    vehicle_model_ptr_->setInput(input);
    vehicle_model_ptr_->update(step_time);

    setStatus(getEntityStatus(current_time + step_time, step_time));
    updateStandStillDuration(step_time);

    if (previous_linear_velocity_) {
      linear_jerk_ = (vehicle_model_ptr_->getVx() - previous_linear_velocity_.get()) / step_time;
    } else {
      linear_jerk_ = 0;
    }

    previous_linear_velocity_ = vehicle_model_ptr_->getVx();
    previous_angular_velocity_ = vehicle_model_ptr_->getWz();
  }
}

auto EgoEntity::ready() const -> bool { return autoware->ready(); }

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

auto EgoEntity::setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) -> void  //
{
}

bool EgoEntity::setStatus(const traffic_simulator_msgs::msg::EntityStatus & status)
{
  const bool success = VehicleEntity::setStatus(status);  // NOTE: setStatus always succeeds.

  const auto current_pose = getStatus().pose;

  if (autoware->initialized()) {
    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    autoware->set(current_pose);
    autoware->set(current_twist);
  }

  if (not initial_pose_) {
    initial_pose_ = current_pose;
  }

  return success;
}

void EgoEntity::setTargetSpeed(double value, bool)
{
  Eigen::VectorXd v(vehicle_model_ptr_->getDimX());

  switch (vehicle_model_type_) {
    case VehicleModelType::DELAY_STEER_ACC:
    case VehicleModelType::DELAY_STEER_ACC_GEARED:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0, 0;
      break;

    case VehicleModelType::IDEAL_STEER_ACC:
    case VehicleModelType::IDEAL_STEER_ACC_GEARED:
    case VehicleModelType::LEGACY_IDEAL_ACCEL:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value);
      break;

    case VehicleModelType::IDEAL_STEER_VEL:
    case VehicleModelType::LEGACY_IDEAL_STEER:
      v << 0, 0, 0;
      break;

    case VehicleModelType::DELAY_STEER_VEL:
    case VehicleModelType::LEGACY_DELAY_STEER:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0;
      break;

    case VehicleModelType::LEGACY_DELAY_STEER_ACC:
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0, 0;
      break;

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported simulation model ", toString(vehicle_model_type_), " specified");
  }

  vehicle_model_ptr_->setState(v);
}

void EgoEntity::setTargetSpeed(const RelativeTargetSpeed & /*target_speed*/, bool /*continuous*/) {}

auto EgoEntity::setUpperBoundSpeed(double value) -> void  //
{
  autoware->setUpperBoundSpeed(value);
}
}  // namespace entity
}  // namespace traffic_simulator
