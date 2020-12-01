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

#include <simulation_api/math/hermite_curve.hpp>

#include <vector>

namespace simulation_api
{
namespace math
{
HermiteCurve::HermiteCurve(
  double ax, double bx, double cx, double dx,
  double ay, double by, double cy, double dy,
  double az, double bz, double cz, double dz)
: ax_(ax), bx_(bx), cx_(cx), dx_(dx),
  ay_(ay), by_(by), cy_(cy), dy_(dy),
  az_(az), bz_(bz), cz_(cz), dz_(dz)
{}

HermiteCurve::HermiteCurve(
  geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose goal_pose,
  geometry_msgs::msg::Vector3 start_vec, geometry_msgs::msg::Vector3 goal_vec)
{
  ax_ = 2 * start_pose.position.x - 2 * goal_pose.position.x + start_vec.x + goal_vec.x;
  bx_ = -3 * start_pose.position.x + 3 * goal_pose.position.x - 2 * start_vec.x - goal_vec.x;
  cx_ = start_vec.x;
  dx_ = start_pose.position.x;

  ay_ = 2 * start_pose.position.y - 2 * goal_pose.position.y + start_vec.y + goal_vec.y;
  by_ = -3 * start_pose.position.y + 3 * goal_pose.position.y - 2 * start_vec.y - goal_vec.y;
  cy_ = start_vec.y;
  dy_ = start_pose.position.y;

  az_ = 2 * start_pose.position.z - 2 * goal_pose.position.z + start_vec.z + goal_vec.z;
  bz_ = -3 * start_pose.position.z + 3 * goal_pose.position.z - 2 * start_vec.z - goal_vec.z;
  cz_ = start_vec.z;
  dz_ = start_pose.position.z;
}

std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory()
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (int i = 0; i <= 100; i++) {
    double t = static_cast<double>(i) / 100.0;
    geometry_msgs::msg::Point p = getPoint(t);
    ret.push_back(p);
  }
  return ret;
}

const geometry_msgs::msg::Vector3 HermiteCurve::getTangentVector(double s, bool autoscale)
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Vector3 vec;
  vec.x = 3 * ax_ * s * s + 2 * bx_ * s + cx_;
  vec.y = 3 * ay_ * s * s + 2 * by_ * s + cy_;
  vec.z = 3 * az_ * s * s + 2 * bz_ * s + cz_;
  return vec;
}

const geometry_msgs::msg::Pose HermiteCurve::getPose(double s, bool autoscale)
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 tangent_vec = getTangentVector(s, false);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = 0.0;
  rpy.y = 0.0;
  rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
  pose.position = getPoint(s);
  return pose;
}

double HermiteCurve::get2DCurvature(double s, bool autoscale)
{
  if (autoscale) {
    s = s / getLength();
  }
  double s2 = s * s;
  double x_dot = 3 * ax_ * s2 + 2 * bx_ * s + cx_;
  double x_dot_dot = 6 * ax_ * s + 2 * bx_;
  double y_dot = 3 * ay_ * s2 + 2 * by_ * s + cy_;
  double y_dot_dot = 6 * ay_ * s + 2 * by_;
  return (x_dot * y_dot_dot - x_dot_dot * y_dot) / std::pow(x_dot * x_dot + y_dot * y_dot, 1.5);
}

double HermiteCurve::getMaximu2DCurvature()
{
  std::vector<double> curvatures;
  for (double s = 0; s <= 1; s = s + 0.01) {
    double curvature = get2DCurvature(s);
    curvatures.push_back(curvature);
  }
  return *std::max_element(curvatures.begin(), curvatures.end());
}

double HermiteCurve::getLength()
{
  auto trajectory = getTrajectory();
  double ret = 0.0;
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    ret = ret + std::sqrt(std::pow(trajectory[i + 1].x - trajectory[i].x, 2) +
        std::pow(trajectory[i + 1].y - trajectory[i].y, 2) +
        std::pow(trajectory[i + 1].z - trajectory[i].z, 2));
  }
  return ret;
}

const geometry_msgs::msg::Point HermiteCurve::getPoint(double s, bool autoscale)
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Point p;
  p.x = ax_ * std::pow(s, 3) + bx_ * std::pow(s, 2) + cx_ * s + dx_;
  p.y = ay_ * std::pow(s, 3) + by_ * std::pow(s, 2) + cy_ * s + dy_;
  p.z = az_ * std::pow(s, 3) + bz_ * std::pow(s, 2) + cz_ * s + dz_;
  return p;
}
}  // namespace math
}  // namespace simulation_api
