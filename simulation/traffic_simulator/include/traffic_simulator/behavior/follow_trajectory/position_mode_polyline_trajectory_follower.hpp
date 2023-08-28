#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__POSITION_MODE_POLYLINE_TRAJECTORY_FOLLOWER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__POSITION_MODE_POLYLINE_TRAJECTORY_FOLLOWER_HPP_

#include <traffic_simulator/behavior/follow_trajectory/polyline_trajectory_follower.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

class PositionModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  PositionModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};

  virtual std::optional<traffic_simulator_msgs::msg::EntityStatus> followTrajectory(
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory)
    override;
  virtual auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, const double,
    const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters) -> void override;
  virtual auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, const double) -> void override;

private:
  auto getUpdatedVelocity(
    const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
    -> geometry_msgs::msg::Vector3;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> override;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> override;
  auto createUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus;
  virtual void discardTheFrontWaypointFromTrajectory() const override;

  auto getDesiredVelocity(
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    double desired_speed) -> geometry_msgs::msg::Vector3;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif