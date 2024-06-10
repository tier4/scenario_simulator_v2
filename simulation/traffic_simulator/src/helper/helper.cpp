// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <string>
#include <traffic_simulator/helper/helper.hpp>

namespace traffic_simulator
{
namespace helper
{
traffic_simulator_msgs::msg::ActionStatus constructActionStatus(
  double linear_vel, double angular_vel, double linear_accel, double angular_accel)
{
  traffic_simulator_msgs::msg::ActionStatus status;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_vel;
  twist.angular.z = angular_vel;
  geometry_msgs::msg::Accel accel;
  accel.linear.x = linear_accel;
  accel.angular.z = angular_accel;
  status.twist = twist;
  status.accel = accel;
  return status;
}

LaneletPose constructLaneletPose(
  lanelet::Id lanelet_id, double s, double offset, double roll, double pitch, double yaw)
{
  LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s;
  lanelet_pose.offset = offset;
  lanelet_pose.rpy.x = roll;
  lanelet_pose.rpy.y = pitch;
  lanelet_pose.rpy.z = yaw;
  return lanelet_pose;
}

geometry_msgs::msg::Vector3 constructRPY(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}

geometry_msgs::msg::Vector3 constructRPYfromQuaternion(geometry_msgs::msg::Quaternion quaternion)
{
  return quaternion_operation::convertQuaternionToEulerAngle(quaternion);
}

geometry_msgs::msg::Pose constructPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation =
    quaternion_operation::convertEulerAngleToQuaternion(constructRPY(roll, pitch, yaw));
  return pose;
}

const simulation_api_schema::DetectionSensorConfiguration constructDetectionSensorConfiguration(
  const std::string & entity, const std::string & architecture_type, const double update_duration,
  const double range, const bool detect_all_objects_in_range, const double pos_noise_stddev,
  const int random_seed, const double probability_of_lost, const double object_recognition_delay,
  const double object_recognition_ground_truth_delay,
  std::unique_ptr<simulation_api_schema::EllipseBasedNoiseConfiguration> noise_config)
{
  simulation_api_schema::DetectionSensorConfiguration configuration;
  configuration.set_entity(entity);
  configuration.set_architecture_type(architecture_type);
  configuration.set_update_duration(update_duration);
  configuration.set_range(range);
  configuration.set_detect_all_objects_in_range(detect_all_objects_in_range);
  configuration.set_pos_noise_stddev(pos_noise_stddev);
  configuration.set_random_seed(random_seed);
  configuration.set_probability_of_lost(probability_of_lost);
  configuration.set_object_recognition_delay(object_recognition_delay);
  configuration.set_object_recognition_ground_truth_delay(object_recognition_ground_truth_delay);

  if (!noise_config) {
      noise_config = std::make_unique<simulation_api_schema::EllipseBasedNoiseConfiguration>(constructEllipseBasedNoiseConfiguration());
  }
  configuration.set_noise_config(*noise_config);

  return configuration;
}

const simulation_api_schema::EllipseBasedNoiseConfiguration constructEllipseBasedNoiseConfiguration(
  const std::vector<double>& ellipse_y_radius_values,
  const double ellipse_normalized_x_radius_masking,
  const double ellipse_normalized_x_radius_distance_mean,
  const double ellipse_normalized_x_radius_distance_std,
  const double ellipse_normalized_x_radius_yaw_mean,
  const double ellipse_normalized_x_radius_yaw_std,
  const std::vector<double>& masking_probability_values,
  const std::vector<double>& distance_mean_values,
  const std::vector<double>& distance_std_values,
  const std::vector<double>& yaw_mean_values,
  const std::vector<double>& yaw_std_values)
{
  simulation_api_schema::EllipseBasedNoiseConfiguration configuration;
  for (const auto& value : ellipse_y_radius_values) {
    configuration.add_ellipse_y_radius_values(value);
  }
  configuration.set_ellipse_normalized_x_radius_masking(ellipse_normalized_x_masking);
  configuration.set_ellipse_normalized_x_radius_distance_mean(ellipse_normalized_x_radius_distance_mean);
  configuration.set_ellipse_normalized_x_radius_distance_std(ellipse_normalized_x_radius_distance_std);
  configuration.set_ellipse_normalized_x_radius_yaw_mean(ellipse_normalized_x_radius_yaw_mean);
  configuration.set_ellipse_normalized_x_radius_yaw_std(ellipse_normalized_x_radius_yaw_std);
  for (const auto& value : masking_probability_values) {
    configuration.add_masking_probability_values(value);
  }
  for (const auto& value : distance_mean_values) {
    configuration.add_distance_mean_values(value);
  }
  for (const auto& value : distance_std_values) {
    configuration.add_distance_std_values(value);
  }
  for (const auto& value : yaw_mean_values) {
    configuration.add_yaw_mean_values(value);
  }
  for (const auto& value : yaw_std_values) {
    configuration.add_yaw_std_values(value);
  }
  return configuration;
}

const simulation_api_schema::LidarConfiguration constructLidarConfiguration(
  const LidarType type, const std::string & entity, const std::string & architecture_type,
  const double lidar_sensor_delay, const double horizontal_resolution)
{
  simulation_api_schema::LidarConfiguration configuration;
  configuration.set_horizontal_resolution(horizontal_resolution);
  configuration.set_architecture_type(architecture_type);
  configuration.set_entity(entity);
  configuration.set_lidar_sensor_delay(lidar_sensor_delay);
  switch (type) {
    case LidarType::VLP16:
      configuration.set_scan_duration(0.1);
      configuration.add_vertical_angles(-15.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(15.0 / 180.0 * M_PI);
      break;
    case LidarType::VLP32:
      configuration.set_scan_duration(0.1);
      configuration.add_vertical_angles(-15.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-13.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-12.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-9.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-8.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-5.66666667 / 180 * M_PI);
      configuration.add_vertical_angles(-4.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-1.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-0.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(2.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(3.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(6.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(7.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(10.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(11.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(14.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(15.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(17.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(18.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(19.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(21.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(22.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(23.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(25.0 / 180.0 * M_PI);
      break;
  }
  return configuration;
}

}  // namespace helper
}  // namespace traffic_simulator

std::ostream & operator<<(std::ostream & os, const traffic_simulator::LaneletPose & ll_pose)
{
  os << "lanelet id : " << ll_pose.lanelet_id << "\ns : " << ll_pose.s;
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Point & point)
{
  os << "x : " << point.x << ",y : " << point.y << ",z : " << point.z << std::endl;
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Vector3 & vector)
{
  os << "x : " << vector.x << ",y : " << vector.y << ",z : " << vector.z << std::endl;
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Quaternion & quat)
{
  os << "x : " << quat.x << ",y : " << quat.y << ",z : " << quat.z << ",w : " << quat.w
     << std::endl;
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Pose & pose)
{
  os << "position : " << std::endl;
  os << pose.position << std::endl;
  os << "orientation : " << std::endl;
  os << pose.orientation << std::endl;
  return os;
}
