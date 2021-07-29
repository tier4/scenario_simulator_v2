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

#include <concealer/autoware_auto.hpp>
#include <concealer/autoware_architecture_proposal.hpp>
#include <functional>
#include <memory>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <string>
#include <system_error>
#include <thread>
#include <traffic_simulator/entity/ego_entity.hpp>
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
    BOILERPLATE(CONST_ACCEL_TWIST);
    BOILERPLATE(DELAY_FORKLIFT_RLS);
    BOILERPLATE(DELAY_STEER);
    BOILERPLATE(DELAY_STEER_ACC);
    BOILERPLATE(DELAY_TWIST);
    BOILERPLATE(IDEAL_ACCEL);
    BOILERPLATE(IDEAL_FORKLIFT_RLS);
    BOILERPLATE(IDEAL_STEER);
    BOILERPLATE(IDEAL_TWIST);
  }

#undef BOILERPLATE

  THROW_SIMULATION_ERROR("Unsupported vehicle model type, failed to convert to string");
}

auto getVehicleModelType()
{
  auto autoware_type = getParameter<std::string>("autoware_type", std::string(""));
  std::string vehicle_model_type;

  if (autoware_type == "proposal") {
    vehicle_model_type = getParameter<std::string>("vehicle_model_type", "IDEAL_STEER");
  } else if (autoware_type == "auto") {
    // hard-coded for now
    // it would require changes in https://github.com/tier4/lexus_description.iv.universe
    vehicle_model_type = "IDEAL_STEER";
  } else {
    THROW_SEMANTIC_ERROR("Unsupported autoware_type ", autoware_type);
  }

  if (vehicle_model_type == "IDEAL_STEER") {
    return VehicleModelType::IDEAL_STEER;
  } else if (vehicle_model_type == "DELAY_STEER") {
    return VehicleModelType::DELAY_STEER;
  } else if (vehicle_model_type == "DELAY_STEER_ACC") {
    return VehicleModelType::DELAY_STEER_ACC;
  } else {
    THROW_SEMANTIC_ERROR("Unsupported vehicle_model_type ", vehicle_model_type, "specified");
  }
}

auto makeSimulationModel(
  const VehicleModelType vehicle_model_type,
  const double step_time,  //
  const openscenario_msgs::msg::VehicleParameters & parameters)
  -> const std::shared_ptr<SimModelInterface>
{
  switch (vehicle_model_type) {
    case VehicleModelType::IDEAL_STEER:
      return std::make_shared<SimModelIdealSteer>(getParameter<double>(
        "wheel_base",
        parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x));

    case VehicleModelType::DELAY_STEER:
      return std::make_shared<SimModelTimeDelaySteer>(
        getParameter<double>("vel_lim", 50.0),     // parameters.performance.max_speed,
        getParameter<double>("steer_lim", 1.0),    // parameters.axles.front_axle.max_steering,
        getParameter<double>("accel_rate", 10.0),  // parameters.performance.max_acceleration,
        getParameter<double>("steer_rate_lim", 5.0),
        getParameter<double>(
          "wheel_base",
          parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x),  //
        step_time,                                                                          //
        getParameter<double>("vel_time_delay", 0.1),                                        //
        getParameter<double>("vel_time_constant", 0.1),                                     //
        getParameter<double>("steer_time_delay", 0.3),                                      //
        getParameter<double>("steer_time_constant", 0.3),                                   //
        getParameter<double>("deadzone_delta_steer", 0.0));

    case VehicleModelType::DELAY_STEER_ACC:
      return std::make_shared<SimModelTimeDelaySteerAccel>(
        getParameter<double>("vel_lim", 50.0),     // parameters.performance.max_speed,
        getParameter<double>("steer_lim", 1.0),    // parameters.axles.front_axle.max_steering,
        getParameter<double>("accel_rate", 10.0),  // parameters.performance.max_acceleration,
        getParameter<double>("steer_rate_lim", 5.0),
        getParameter<double>(
          "wheel_base",
          parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x),  //
        step_time,                                                                          //
        getParameter<double>("acc_time_delay", 0.1),                                        //
        getParameter<double>("acc_time_constant", 0.1),                                     //
        getParameter<double>("steer_time_delay", 0.3),                                      //
        getParameter<double>("steer_time_constant", 0.3),                                   //
        getParameter<double>("deadzone_delta_steer", 0.0));

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported vehicle_model_type ", toString(vehicle_model_type), " specified");
  }
}

EgoEntity::EgoEntity(
  const std::string & name,             //
  const Configuration & configuration,  //
  const double step_time,               //
  const openscenario_msgs::msg::VehicleParameters & parameters)
: VehicleEntity(name, parameters),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters))
{
  auto autoware_type = getParameter<std::string>("autoware_type", std::string(""));

  std::cout << "EgoEntity::autoware_type = " << autoware_type << std::endl;

  if (autoware_type == "proposal") {
    autoware = std::make_unique<concealer::AutowareArchitectureProposal>(getParameter<std::string>("autoware_launch_package"),
                                                                         getParameter<std::string>("autoware_launch_file"),
                                                                         "map_path:=" + configuration.map_path.string(),
                                                                         "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                                                                         "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                                                                         "sensor_model:=" + getParameter<std::string>("sensor_model"),
                                                                         "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                                                                         "rviz_config:=" + configuration.rviz_config_path.string(),
                                                                         "scenario_simulation:=true");
  } else if (autoware_type == "auto") {
    autoware = std::make_unique<concealer::AutowareAuto>(getParameter<std::string>("autoware_launch_package"),
                                                         getParameter<std::string>("autoware_launch_file"),
                                                         "map_path:=" + configuration.map_path.string(),
                                                         "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                                                         "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                                                         "sensor_model:=" + getParameter<std::string>("sensor_model"),
                                                         "vehicle_model:=" + getParameter<std::string>("vehicle_model"),
                                                         "rviz_config:=" + configuration.rviz_config_path.string(),
                                                         "scenario_simulation:=true");

  } else {
    throw std::invalid_argument("Invalid autoware_type = " + autoware_type);
  }

  entity_type_.type = openscenario_msgs::msg::EntityType::EGO;
}

void EgoEntity::engage() { autoware->engage(); }

auto EgoEntity::getVehicleCommand() -> const autoware_vehicle_msgs::msg::VehicleCommand
{
  return autoware->getVehicleCommand();
}

auto EgoEntity::getCurrentAction() const -> const std::string
{
  std::stringstream message;

  const auto state{autoware->getAutowareStateMessage()};

  message << (state.empty() ? "Starting" : state)  //
          << "_(t_=_"                              //
          << std::fixed                            //
          << std::setprecision(2)                  //
          << (status_ ? status_->time : 0)         //
          << ")";

  return message.str();
}

auto EgoEntity::getEntityStatus(const double time, const double step_time) const
  -> const openscenario_msgs::msg::EntityStatus
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

  openscenario_msgs::msg::EntityStatus status;
  {
    status.time = time;
    status.type.type = entity_type_.type;
    status.bounding_box = getBoundingBox();
    status.action_status.twist = twist;
    status.action_status.accel = accel;
    status.pose.position.x = v(0) + initial_pose_.get().position.x;
    status.pose.position.y = v(1) + initial_pose_.get().position.y;
    status.pose.position.z = v(2) + initial_pose_.get().position.z;

    const auto closest_lanelet_id = hdmap_utils_ptr_->getClosetLaneletId(status.pose);
    if (!closest_lanelet_id) {
      THROW_SEMANTIC_ERROR("failed to find the closest lane, lane is too far away.");
    }

    traffic_simulator::math::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(closest_lanelet_id.get()));
    if (const auto s_value = spline.getSValue(status.pose.position)) {
      status.pose.position.z = spline.getPoint(s_value.get()).z;
    }

    status.pose.orientation = initial_pose_.get().orientation * pose.orientation;

    const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(status.pose);

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

auto EgoEntity::getObstacle() -> boost::optional<openscenario_msgs::msg::Obstacle>
{
  return boost::none;
}

auto EgoEntity::getWaypoints() -> const openscenario_msgs::msg::WaypointsArray
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
    switch (vehicle_model_type_) {
      case VehicleModelType::IDEAL_STEER:
      case VehicleModelType::DELAY_STEER: {
        Eigen::VectorXd input(2);

        input << autoware->getVelocity(), autoware->getSteeringAngle();

        (*vehicle_model_ptr_).setInput(input);
      } break;

      case VehicleModelType::DELAY_STEER_ACC: {
        Eigen::VectorXd input(3);

        input << autoware->getVelocity(), autoware->getSteeringAngle(), autoware->getGearSign();

              (*vehicle_model_ptr_).setInput(input);
      } break;

      default:
        THROW_SEMANTIC_ERROR(
          "Unsupported vehicle_model_type ", toString(vehicle_model_type_), "specified");
    }

    (*vehicle_model_ptr_).update(step_time);

    setStatus(getEntityStatus(current_time + step_time, step_time));

    if (previous_linear_velocity_) {
      linear_jerk_ = (vehicle_model_ptr_->getVx() - previous_linear_velocity_.get()) / step_time;
    } else {
      linear_jerk_ = 0;
    }

    previous_linear_velocity_ = vehicle_model_ptr_->getVx();
    previous_angular_velocity_ = vehicle_model_ptr_->getWz();
  }
}

auto EgoEntity::ready() const -> bool
{
  // NOTE: Autoware::ready() will notify you with an exception if Autoware has detected any anomalies at the time of the call.
  return autoware->ready();
}

void EgoEntity::requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose)
{
  requestAssignRoute({lanelet_pose});
}

void EgoEntity::requestAssignRoute(
  const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints)
{
  std::vector<geometry_msgs::msg::PoseStamped> route;

  for (const auto & waypoint : waypoints) {
    route.push_back((*hdmap_utils_ptr_).toMapPose(waypoint));
  }

  assert(0 < route.size());

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

bool EgoEntity::setStatus(const openscenario_msgs::msg::EntityStatus & status)
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

  if (!initial_pose_) {
    initial_pose_ = current_pose;
  }

  return success;
}

void EgoEntity::setTargetSpeed(double value, bool)
{
  switch (vehicle_model_type_) {
    case VehicleModelType::IDEAL_STEER: {
      Eigen::VectorXd v(3);
      v << 0, 0, 0;

      (*vehicle_model_ptr_).setState(v);
    } break;

    case VehicleModelType::DELAY_STEER: {
      Eigen::VectorXd v(5);
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0;

      (*vehicle_model_ptr_).setState(v);
    } break;

    case VehicleModelType::DELAY_STEER_ACC: {
      Eigen::VectorXd v(6);
      v << 0, 0, 0, autoware->restrictTargetSpeed(value), 0, 0;

      (*vehicle_model_ptr_).setState(v);
    } break;

    default:
      THROW_SEMANTIC_ERROR(
        "Unsupported simulation model ",
        getParameter<std::string>("vehicle_model_type", "IDEAL_STEER"), "specified");
  }
}
}  // namespace entity
}  // namespace traffic_simulator
