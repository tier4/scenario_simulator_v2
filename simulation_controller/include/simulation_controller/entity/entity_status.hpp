#ifndef SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_
#define SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>

namespace simulation_controller
{
namespace entity
{
enum CoordinateFrameTypes
{
  WORLD = 0,
  LANE = 1,
};

class EntityStatus
{
public:
  EntityStatus() = default;
  EntityStatus(
    double t,
    geometry_msgs::msg::Pose pose,
    geometry_msgs::msg::Twist twist,
    geometry_msgs::msg::Accel accel);
  EntityStatus(
    double t,
    int lanelet_id, double s, double offset,
    geometry_msgs::msg::Vector3 rpy,
    geometry_msgs::msg::Twist twist,
    geometry_msgs::msg::Accel accel);
  EntityStatus & operator=(const EntityStatus & obj)
  {
    this->time = obj.time;
    this->coordinate = obj.coordinate;
    this->twist = obj.twist;
    this->accel = obj.accel;
    this->lanelet_id = obj.lanelet_id;
    this->offset = obj.offset;
    this->s = obj.s;
    this->rpy = obj.rpy;
    this->pose = obj.pose;
    return *this;
  }
  double time;
  CoordinateFrameTypes coordinate;
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Accel accel;
  /* Field for Lene Pose */
  int lanelet_id;
  double offset;
  double s;
  geometry_msgs::msg::Vector3 rpy;
  /* Field for world pose */
  geometry_msgs::msg::Pose pose;
};
}
}

#endif  // SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_
