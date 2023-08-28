#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_

#include <traffic_simulator/behavior/follow_trajectory/utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

class Vehicle
{
  traffic_simulator_msgs::msg::EntityStatus const * status = nullptr;
  traffic_simulator_msgs::msg::VehicleParameters const * vehicle_parameters = nullptr;

public:
  Vehicle() = default;
  Vehicle(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters)
  : status{&entity_status}, vehicle_parameters{&vehicle_parameters} {};
  Vehicle(const traffic_simulator_msgs::msg::EntityStatus & entity_status)
  : status{&entity_status} {};

  auto getCurrentPosition() const -> geometry_msgs::msg::Point;
  auto getFrontPosition() const -> geometry_msgs::msg::Point;
  auto getCurrentSpeed() const -> double;
  auto getCurrentAcceleration() const -> double;
  auto getName() const -> std::string;
  auto getTime() const -> double;
  auto getMaxSteering() const -> double;
  auto getWheelBase() const -> double;
  auto getOrientation() const -> geometry_msgs::msg::Vector3;
  auto createUpdatedStatus(const double steering, const double speed, const double step_time) const
    -> traffic_simulator_msgs::msg::EntityStatus;
  auto createUpdatedStatus(const geometry_msgs::msg::Vector3 & velocity, const double step_time)
    const -> traffic_simulator_msgs::msg::EntityStatus;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__VEHICLE_MODEL_HPP_