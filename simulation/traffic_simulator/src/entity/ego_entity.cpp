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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <concealer/autoware_def.hpp>
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

#define DEBUG_VALUE(...) \
  std::cout << "\x1b[32m" #__VA_ARGS__ " = " << (__VA_ARGS__) << "\x1b[0m" << std::endl

#define DEBUG_LINE() \
  std::cout << "\x1b[32m" << __FILE__ << ":" << __LINE__ << "\x1b[0m" << std::endl

template <typename T>
auto getParameter(const std::string & name, T value)
{
  rclcpp::Node node{"get_parameter", "simulation"};

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

namespace traffic_simulator
{
namespace entity
{
std::unordered_map<std::string, concealer::Autoware> EgoEntity::ego_entities{};

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
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  const auto vehicle_model_type = getParameter<std::string>("vehicle_model_type", "IDEAL_STEER");
#endif
#ifdef AUTOWARE_AUTO
  // hard-coded for now
  // it would require changes in https://github.com/tier4/lexus_description.iv.universe
  // please, let us know if we should do that
  const auto vehicle_model_type = "IDEAL_STEER";
#endif

  DEBUG_VALUE(vehicle_model_type);

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
  -> std::shared_ptr<SimModelInterface>
{
  DEBUG_VALUE(getParameter<double>("vel_lim", 50.0));
  DEBUG_VALUE(getParameter<double>("steer_lim", 1.0));
  DEBUG_VALUE(getParameter<double>("accel_rate", 10.0));
  DEBUG_VALUE(getParameter<double>("steer_rate_lim", 5.0));
  DEBUG_VALUE(getParameter<double>(
    "wheel_base", parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x));
  DEBUG_VALUE(step_time);
  DEBUG_VALUE(getParameter<double>("vel_time_delay", 0.25));
  DEBUG_VALUE(getParameter<double>("vel_time_constant", 0.5));
  DEBUG_VALUE(getParameter<double>("acc_time_delay", 0.1));
  DEBUG_VALUE(getParameter<double>("acc_time_constant", 0.1));
  DEBUG_VALUE(getParameter<double>("steer_time_delay", 0.3));
  DEBUG_VALUE(getParameter<double>("steer_time_constant", 0.3));
  DEBUG_VALUE(getParameter<double>("deadzone_delta_steer", 0.0));

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
        "Unsupported vehicle_model_type ", toString(vehicle_model_type), "specified");
  }
}

EgoEntity::EgoEntity(
  const std::string & name,  //
  const boost::filesystem::path & lanelet2_map_osm,
  const double step_time,  //
  const openscenario_msgs::msg::VehicleParameters & parameters)
: VehicleEntity(name, parameters),
  vehicle_model_type_(getVehicleModelType()),
  vehicle_model_ptr_(makeSimulationModel(vehicle_model_type_, step_time, parameters))
{
  entity_type_.type = openscenario_msgs::msg::EntityType::EGO;

  ego_entities.emplace(
    std::piecewise_construct, std::forward_as_tuple(name),
    std::forward_as_tuple(
      getParameter("autoware_launch_package", std::string("")),
      getParameter("autoware_launch_file", std::string("")),
      "map_path:=" + lanelet2_map_osm.parent_path().string(),
      "lanelet2_map_file:=" + lanelet2_map_osm.filename().string(),
      "sensor_model:=" + getParameter("sensor_model", std::string("")),
      "vehicle_model:=" + getParameter("vehicle_model", std::string("")),
      "rviz_config:=" + ament_index_cpp::get_package_share_directory("scenario_test_runner") +
        "/config/scenario_simulator_v2.rviz",
      "scenario_simulation:=true"));
}

EgoEntity::~EgoEntity() { ego_entities.erase(name); }

void EgoEntity::engage()
{
  // while (not ready()) {  // guard
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }

  ego_entities.at(name).engage();
}

const autoware_vehicle_msgs::msg::VehicleCommand EgoEntity::getVehicleCommand()
{
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  return ego_entities.at(name).getVehicleCommand();
#endif
#ifdef AUTOWARE_AUTO
  // gathering information and converting it to autoware_vehicle_msgs::msg::VehicleCommand
  autoware_vehicle_msgs::msg::VehicleCommand vehicle_command;

  auto vehicle_control_command = ego_entities.at(name).getVehicleControlCommand();

  vehicle_command.header.stamp = vehicle_control_command.stamp;

  vehicle_command.control.steering_angle = vehicle_control_command.front_wheel_angle_rad;
  vehicle_command.control.velocity = vehicle_control_command.velocity_mps;
  vehicle_command.control.acceleration = vehicle_control_command.long_accel_mps2;

  auto vehicle_state_command = ego_entities.at(name).getVehicleStateCommand();

  // handle gear enum remapping
  switch (vehicle_state_command.gear) {
    case autoware_auto_msgs::msg::VehicleStateReport::GEAR_DRIVE:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::DRIVE;
      break;
    case autoware_auto_msgs::msg::VehicleStateReport::GEAR_REVERSE:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::REVERSE;
      break;
    case autoware_auto_msgs::msg::VehicleStateReport::GEAR_PARK:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::PARKING;
      break;
    case autoware_auto_msgs::msg::VehicleStateReport::GEAR_LOW:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::LOW;
      break;
    case autoware_auto_msgs::msg::VehicleStateReport::GEAR_NEUTRAL:
      vehicle_command.shift.data = autoware_vehicle_msgs::msg::Shift::NEUTRAL;
      break;
  }

  // these fields are hard-coded because they are not present in AutowareAuto
  vehicle_command.header.frame_id = "";
  vehicle_command.control.steering_angle_velocity = 0.0;
  vehicle_command.emergency = 0;

  return vehicle_command;
#endif
}

auto EgoEntity::getCurrentAction() const -> const std::string
{
  std::stringstream message;
  {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
    // *** getAutowareStatus - FundamentalAPI dependency ***
    const auto state{ego_entities.at(name).getAutowareStatus().autoware_state};

    message << (state.empty() ? "Starting" : state)  //
            << "_(t_=_"                              //
            << std::fixed                            //
            << std::setprecision(2)                  //
            << (status_ ? status_->time : 0)         //
            << ")";
#endif
#ifdef AUTOWARE_AUTO
    // TODO: implement Autoware.Auto equivalent when state monitoring is there
#endif
  }

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
  openscenario_msgs::msg::WaypointsArray waypoints;

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
  // Trajectory returned by getTrajectory() is a different message between AAP and AA
  for (const auto & point : ego_entities.at(name).getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }
#endif
#ifdef AUTOWARE_AUTO
  for (const auto & point : ego_entities.at(name).getTrajectory().points) {
    geometry_msgs::msg::Point waypoint;
    waypoint.x = point.x;
    waypoint.y = point.y;
    waypoint.z = 0;
    waypoints.waypoints.push_back(waypoint);
  }
#endif

  return waypoints;
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

    ego_entities.at(name).set(current_pose);

    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    ego_entities.at(name).set(current_twist);
  } else {
    switch (vehicle_model_type_) {
      case VehicleModelType::IDEAL_STEER:
      case VehicleModelType::DELAY_STEER: {
        Eigen::VectorXd input(2);

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
        input <<  //
          ego_entities.at(name).getVehicleCommand().control.velocity,
          ego_entities.at(name).getVehicleCommand().control.steering_angle;
#endif
#ifdef AUTOWARE_AUTO
        input <<  //
          ego_entities.at(name).getVehicleControlCommand().velocity_mps,
          ego_entities.at(name).getVehicleControlCommand().front_wheel_angle_rad;
#endif

        (*vehicle_model_ptr_).setInput(input);
      } break;

      case VehicleModelType::DELAY_STEER_ACC: {
        Eigen::VectorXd input(3);

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
        using autoware_vehicle_msgs::msg::Shift;

        input <<  //
          ego_entities.at(name).getVehicleCommand().control.acceleration,
          ego_entities.at(name).getVehicleCommand().control.steering_angle,
          ego_entities.at(name).getVehicleCommand().shift.data == Shift::REVERSE ? -1.0 : 1.0;
#endif
#ifdef AUTOWARE_AUTO
        input <<  //
          ego_entities.at(name).getVehicleControlCommand().velocity_mps,
          ego_entities.at(name).getVehicleControlCommand().front_wheel_angle_rad,
          ego_entities.at(name).getVehicleStateCommand().gear ==
              autoware_auto_msgs::msg::VehicleStateReport::GEAR_REVERSE
            ? -1.0
            : 1.0;
#endif

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
  return ego_entities.at(name).ready();
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

  if (not std::exchange(autoware_initialized, true)) {
    ego_entities.at(name).initialize(getStatus().pose);
    ego_entities.at(name).plan(route);
    // NOTE: engage() will be executed at simulation-time 0.
  } else {
    ego_entities.at(name).plan(route);
    ego_entities.at(name).engage();
  }
}

void EgoEntity::requestLaneChange(const std::int64_t)
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", name,
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination.");
}

bool EgoEntity::setStatus(const openscenario_msgs::msg::EntityStatus & status)
{
  // NOTE Currently, setStatus always succeeds.
  const bool success = VehicleEntity::setStatus(status);

  const auto current_pose = getStatus().pose;

  if (autoware_initialized) {
    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    ego_entities.at(name).set(current_pose);
    ego_entities.at(name).set(current_twist);
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
      {
        v << 0, 0, 0;
      }

      (*vehicle_model_ptr_).setState(v);
    } break;

    case VehicleModelType::DELAY_STEER: {
      Eigen::VectorXd v(5);
      {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
        v << 0, 0, 0, value, 0;
#endif
#ifdef AUTOWARE_AUTO
        // non-zero initial speed prevents behavioral planner from planning
        v << 0, 0, 0, 0, 0;
#endif
      }

      (*vehicle_model_ptr_).setState(v);
    } break;

    case VehicleModelType::DELAY_STEER_ACC: {
      Eigen::VectorXd v(6);
      {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
        v << 0, 0, 0, value, 0, 0;
#endif
#ifdef AUTOWARE_AUTO
        // non-zero initial speed prevents behavioral planner from planning
        v << 0, 0, 0, 0, 0, 0;
#endif
      }

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
