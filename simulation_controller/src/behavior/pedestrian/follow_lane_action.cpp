#include <simulation_controller/behavior/pedestrian/follow_lane_action.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>

#include <iostream>

namespace entity_behavior
{
namespace pedestrian
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::ActionNode(name, config)
{

}

BT::NodeStatus FollowLaneAction::tick()
{
  std::string request;
  if (!getInput("request", request)) {
    throw BehaviorTreeRuntimeError("failed to get input request in FollowLaneAction");
  }
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  double step_time, current_time;
  if (!getInput<double>("step_time", step_time)) {
    throw BehaviorTreeRuntimeError("failed to get input step_time in FollowLaneAction");
  }
  if (!getInput<double>("current_time", current_time)) {
    throw BehaviorTreeRuntimeError("failed to get input current_time in FollowLaneAction");
  }
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils_ptr)) {
    throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in FollowLaneAction");
  }

  simulation_controller::entity::EntityStatus entity_status;
  if (!getInput<simulation_controller::entity::EntityStatus>("entity_status", entity_status)) {
    throw BehaviorTreeRuntimeError("failed to get input entity_status in FollowLaneAction");
  }

  boost::optional<double> target_speed;
  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  std::shared_ptr<simulation_controller::entity::PedestrianParameters> pedestrian_param_ptr;
  if (!getInput<std::shared_ptr<simulation_controller::entity::PedestrianParameters>>(
      "pedestrian_parameters", pedestrian_param_ptr))
  {
    throw BehaviorTreeRuntimeError("failed to get input pedestrian_parameters in FollowLaneAction");
  }

  if (entity_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::WORLD) {
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;

    geometry_msgs::msg::Twist twist_new;
    twist_new.linear.x = entity_status.twist.linear.x + entity_status.accel.linear.x * step_time;
    twist_new.linear.y = entity_status.twist.linear.y + entity_status.accel.linear.y * step_time;
    twist_new.linear.z = entity_status.twist.linear.z + entity_status.accel.linear.z * step_time;
    twist_new.angular.x = entity_status.twist.angular.x + entity_status.accel.angular.x * step_time;
    twist_new.angular.y = entity_status.twist.angular.y + entity_status.accel.angular.y * step_time;
    twist_new.angular.z = entity_status.twist.angular.z + entity_status.accel.angular.z * step_time;

    geometry_msgs::msg::Pose pose_new;
    geometry_msgs::msg::Vector3 angular_trans_vec;
    angular_trans_vec.z = twist_new.angular.z * step_time;
    geometry_msgs::msg::Quaternion angular_trans_quat =
      quaternion_operation::convertEulerAngleToQuaternion(angular_trans_vec);
    pose_new.orientation =
      quaternion_operation::rotation(entity_status.pose.orientation, angular_trans_quat);
    Eigen::Vector3d trans_vec;
    trans_vec(0) = twist_new.linear.x * step_time;
    trans_vec(1) = twist_new.linear.y * step_time;
    trans_vec(2) = 0;
    Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(pose_new.orientation);
    trans_vec = rotation_mat * trans_vec;
    pose_new.position.x = trans_vec(0) + entity_status.pose.position.x;
    pose_new.position.y = trans_vec(1) + entity_status.pose.position.y;
    pose_new.position.z = trans_vec(2) + entity_status.pose.position.z;

    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
      pose_new, twist_new,
      accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  if (entity_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::LANE) {
    auto following_lanelets = hdmap_utils_ptr->getFollowingLanelets(entity_status.lanelet_id);
    if (!target_speed) {
      target_speed = hdmap_utils_ptr->getSpeedLimit(following_lanelets);
    }
    std::vector<geometry_msgs::msg::Point> following_trajectory;
    following_trajectory = hdmap_utils_ptr->clipTrajectoryFromLaneletIds(entity_status.lanelet_id,
        entity_status.s,
        following_lanelets);
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;

    double target_accel = (target_speed.get() - entity_status.twist.linear.x) / step_time;
    if (entity_status.twist.linear.x > target_speed.get()) {
      target_accel = boost::algorithm::clamp(target_accel, -5, 0);
      //target_accel = boost::algorithm::clamp(target_accel, -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);
    } else {
      target_accel = boost::algorithm::clamp(target_accel, 0, 3);
      //target_accel = boost::algorithm::clamp(target_accel, -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);
    }
    accel_new.linear.x = target_accel;
    geometry_msgs::msg::Twist twist_new;
    twist_new.linear.x = boost::algorithm::clamp(
      entity_status.twist.linear.x + accel_new.linear.x * step_time,
      0, 5.0);
    twist_new.linear.y = 0.0;
    twist_new.linear.z = 0.0;
    twist_new.angular.x = 0.0;
    twist_new.angular.y = 0.0;
    twist_new.angular.z = 0.0;

    double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
      step_time;
    geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
      entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
    setOutput("updated_status", entity_status_updated);
    setOutput("trajectory", following_trajectory);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}
}      // namespace pedestrian
}  // namespace entity_behavior
