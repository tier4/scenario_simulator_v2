#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory/vehicle_model.hpp>

#include <quaternion_operation/quaternion_operation.h>

namespace traffic_simulator
{
namespace follow_trajectory
{
auto Vehicle::createUpdatedStatus(const geometry_msgs::msg::Vector3 & velocity, double step_time)
  const -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator/;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+=;

  auto updated_status = status;

  updated_status.pose.position += velocity * step_time;

  updated_status.pose.orientation = [&]() {
    geometry_msgs::msg::Vector3 direction;
    direction.x = 0;
    direction.y = 0;
    direction.z = std::atan2(velocity.y, velocity.x);
    return quaternion_operation::convertEulerAngleToQuaternion(direction);
  }();

  updated_status.action_status.twist.linear.x = math::geometry::norm(velocity);
  updated_status.action_status.twist.linear.y = 0;
  updated_status.action_status.twist.linear.z = 0;

  updated_status.action_status.twist.angular =
    quaternion_operation::convertQuaternionToEulerAngle(
      quaternion_operation::getRotation(status.pose.orientation, updated_status.pose.orientation)) /
    step_time;

  updated_status.action_status.accel.linear =
    (updated_status.action_status.twist.linear - status.action_status.twist.linear) / step_time;

  updated_status.action_status.accel.angular =
    (updated_status.action_status.twist.angular - status.action_status.twist.angular) / step_time;

  updated_status.time = status.time + step_time;

  updated_status.lanelet_pose_valid = false;

  return updated_status;
}

auto Vehicle::createUpdatedStatus(double steering, double speed, double step_time) const
  -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator/;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+=;

  auto updated_status = status;

  updated_status.pose.position += [&]() {
    geometry_msgs::msg::Vector3 change;
    change.x = std::cos(getOrientation().z) * speed * step_time;
    change.y = std::sin(getOrientation().z) * speed * step_time;
    change.z = 0.0;
    return change;
  }();

  updated_status.pose.orientation = [&]() {
    geometry_msgs::msg::Vector3 direction;
    direction.x = 0.0;
    direction.y = 0.0;
    direction.z = getOrientation().z + speed * std::tan(steering) * step_time / getWheelBase();
    return quaternion_operation::convertEulerAngleToQuaternion(direction);
  }();

  updated_status.action_status.twist.linear.x = speed;
  updated_status.action_status.twist.linear.y = 0;
  updated_status.action_status.twist.linear.z = 0;

  updated_status.action_status.twist.angular =
    quaternion_operation::convertQuaternionToEulerAngle(
      quaternion_operation::getRotation(status.pose.orientation, updated_status.pose.orientation)) /
    step_time;

  updated_status.action_status.accel.linear =
    (updated_status.action_status.twist.linear - status.action_status.twist.linear) / step_time;

  updated_status.action_status.accel.angular =
    (updated_status.action_status.twist.angular - status.action_status.twist.angular) / step_time;

  updated_status.time = getTime() + step_time;

  updated_status.lanelet_pose_valid = false;

  return updated_status;
}

auto Vehicle::getCurrentPosition() const -> geometry_msgs::msg::Point
{
  const auto position = status.pose.position;

  if (any(is_infinity_or_nan, position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(status.name), " coordinate value contains NaN or infinity. The value is [",
      position.x, ", ", position.y, ", ", position.z, "].");
  }

  return position;
}

auto Vehicle::getFrontPosition() const -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point front_axle_coords;
  front_axle_coords.x = getCurrentPosition().x + std::cos(getOrientation().z) * getWheelBase();
  front_axle_coords.y = getCurrentPosition().y + std::sin(getOrientation().z) * getWheelBase();
  front_axle_coords.z = 0.0;
  return front_axle_coords;
}

auto Vehicle::getCurrentAcceleration() const -> double
{
  const auto acceleration = status.action_status.accel.linear.x;  // m/s^2

  if (std::isinf(acceleration) or std::isnan(acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(status.name), "'s acceleration value is NaN or infinity. The value is ",
      acceleration, ".");
  }
  return acceleration;
}

auto Vehicle::getCurrentSpeed() const -> double
{
  const auto speed = status.action_status.twist.linear.x;  // [m/s]
  if (std::isinf(speed) or std::isnan(speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(status.name), "'s speed value is NaN or infinity. The value is ", speed, ".");
  }
  return speed;
}

auto Vehicle::getName() const -> std::string
{
  return status.name;
}

auto Vehicle::getTime() const -> double
{
  return status.time;
}

auto Vehicle::getMaxSteering() const -> double
{
  if (!vehicle_parameters) {
    throw common::Error(
      "Access to vehicle_parameters of Vehicle: ", status.name,
      ", but vehicle_parameters is not set");
  }
  return vehicle_parameters.value().axles.front_axle.max_steering;
}

auto Vehicle::getWheelBase() const -> double
{
  if (!vehicle_parameters) {
    throw common::Error(
      "Access to vehicle_parameters of Vehicle: ", status.name,
      ", but vehicle_parameters is not set");
  }
  return vehicle_parameters.value().axles.front_axle.position_x;
}

auto Vehicle::getOrientation() const -> geometry_msgs::msg::Vector3
{
  return quaternion_operation::convertQuaternionToEulerAngle(status.pose.orientation);
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
