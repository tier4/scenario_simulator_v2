#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_

#include <traffic_simulator/behavior/follow_trajectory/utils.hpp>

#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

#include <optional>

namespace traffic_simulator
{
namespace follow_trajectory
{

class Vehicle
{
public:
  Vehicle() = default;
  explicit Vehicle(const traffic_simulator_msgs::msg::EntityStatus & entity_status)
  : status{entity_status}, vehicle_parameters{std::nullopt} {};
  Vehicle(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters)
  : status{entity_status}, vehicle_parameters{vehicle_parameters} {};

  auto getCurrentPosition() const -> geometry_msgs::msg::Point;
  auto getFrontPosition() const -> geometry_msgs::msg::Point;
  auto getCurrentSpeed() const -> double;
  auto getCurrentAcceleration() const -> double;
  auto getName() const -> std::string;
  auto getTime() const -> double;
  auto getMaxSteering() const -> double;
  auto getWheelBase() const -> double;
  auto getOrientation() const -> geometry_msgs::msg::Vector3;

  auto createUpdatedStatus(double steering, double speed, double step_time) const
    -> traffic_simulator_msgs::msg::EntityStatus;
  auto createUpdatedStatus(const geometry_msgs::msg::Vector3 & velocity, double step_time) const
    -> traffic_simulator_msgs::msg::EntityStatus;

private:
  const traffic_simulator_msgs::msg::EntityStatus status;
  const std::optional<const traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_
