#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_MODE_POLYLINE_TRAJECTORY_FOLLOWER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_MODE_POLYLINE_TRAJECTORY_FOLLOWER_HPP_

#include <traffic_simulator/behavior/follow_trajectory/polyline_trajectory_follower.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

class FollowModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  FollowModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};

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
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> override;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> override;

  virtual void discardTheFrontWaypointFromTrajectory() const override;

  auto getSteering(const geometry_msgs::msg::Point & target_position, double desired_speed)
    -> double;

  mutable std::optional<geometry_msgs::msg::Point> previous_target;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif