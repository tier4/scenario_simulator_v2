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
auto getParameter(const std::string & name, const T & alternate)
{
  rclcpp::Node node{"get_parameter", "simulation"};

  auto value = alternate;

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

namespace traffic_simulator
{
namespace entity
{
std::unordered_map<std::string, concealer::Autoware> EgoEntity::autowares{};

EgoEntity::EgoEntity(
  const std::string & name,  //
  const boost::filesystem::path & lanelet2_map_osm,
  const double step_time,  //
  const openscenario_msgs::msg::VehicleParameters & parameters)
: VehicleEntity(name, parameters),
  vehicle_model_ptr_(std::make_shared<SimModelTimeDelaySteer>(  // XXX: HARD CODING!!!
    parameters.performance.max_speed, parameters.axles.front_axle.max_steering,
    parameters.performance.max_acceleration,
    5.0,  // steer_rate_lim,
    parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x, step_time,
    0.25,  // vel_time_delay,
    0.5,   // vel_time_constant,
    0.3,   // steer_time_delay,
    0.3,   // steer_time_constant,
    0.0    // deadzone_delta_steer
    ))
{
  entity_type_.type = openscenario_msgs::msg::EntityType::EGO;

  autowares.emplace(
    std::piecewise_construct, std::forward_as_tuple(name),
    std::forward_as_tuple(
      getParameter("autoware_launch_package", std::string("")),
      getParameter("autoware_launch_file", std::string("")),
      "map_path:=" + lanelet2_map_osm.parent_path().string(),
      "lanelet2_map_file:=" + lanelet2_map_osm.filename().string(),
      "sensor_model:=" + getParameter("sensor_model", std::string("")),
      "vehicle_model:=" + getParameter("vehicle_model", std::string("")),
      "rviz_config:=" + ament_index_cpp::get_package_share_directory("scenario_test_runner") +
        "/planning_simulator_v2.rviz",
      "scenario_simulation:=true"));
}

EgoEntity::~EgoEntity() { autowares.erase(name); }

void EgoEntity::engage()
{
  // while (not ready()) {  // guard
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }

  autowares.at(name).engage();
}

auto EgoEntity::getCurrentAction() const -> const std::string
{
  std::stringstream message;
  {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
    // *** getAutowareStatus - FundamentalAPI dependency ***
    const auto state{autowares.at(name).getAutowareStatus().autoware_state};

    message << (state.empty() ? "Starting" : state)  //
            << "_(t_=_"                              //
            << std::fixed                            //
            << std::setprecision(2)                  //
            << (status_ ? status_->time : 0)         //
            << ")";
#endif
#ifdef AUTOWARE_AUTO
    // TODO: implement
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

    const auto closest_lanelet_id = hdmap_utils_ptr_->getClosetLanletId(status.pose);
    if (!closest_lanelet_id) {
      THROW_SEMANTIC_ERROR("failed to find closest lane, lane is too far away.");
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
  // *** getTrajectory - MiscellaneousAPI dependency ***
  for (const auto & point : autowares.at(name).getTrajectory().points) {
    waypoints.waypoints.emplace_back(point.pose.position);
  }
#endif
#ifdef AUTOWARE_AUTO
    // TODO: implement
#endif

  return waypoints;
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
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

    autowares.at(name).set(current_pose);

    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    autowares.at(name).set(current_twist);
  } else {
    Eigen::VectorXd input(2);
    {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
      input <<  //
        autowares.at(name).getVehicleCommand().control.velocity,
        autowares.at(name).getVehicleCommand().control.steering_angle;
#endif
#ifdef AUTOWARE_AUTO
      input <<  //
        autowares.at(name).getVehicleControlCommand().long_accel_mps2,
        autowares.at(name).getVehicleControlCommand().front_wheel_angle_rad;
#endif
    }

    (*vehicle_model_ptr_).setInput(input);
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
  return autowares.at(name).ready();
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
    autowares.at(name).initialize(getStatus().pose);
  }

  autowares.at(name).plan(route);
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

    autowares.at(name).set(current_pose);
    autowares.at(name).set(current_twist);
  }

  if (!initial_pose_) {
    initial_pose_ = current_pose;
  }

  return success;
}

void EgoEntity::setTargetSpeed(double value, bool)
{
  Eigen::VectorXd v(5);
  {
    v << 0, 0, 0, value, 0;
  }

  (*vehicle_model_ptr_).setState(v);
}
}  // namespace entity
}  // namespace traffic_simulator
