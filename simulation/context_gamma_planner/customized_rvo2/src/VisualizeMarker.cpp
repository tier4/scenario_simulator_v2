// Copyright 2022 Tier IV, Inc All rights reserved.
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

#ifndef CUSTOMIZED_RVO2__VISUALIZEMARKER_CPP_
#define CUSTOMIZED_RVO2__VISUALIZEMARKER_CPP_

#include "customized_rvo2/VisualizeMarker.hpp"

#include <Eigen/Dense>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>
#include <vector>

namespace RVO
{
/**
 *
 */

void VisualizeMarker::visualizeAgents(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  size_t i = 0;
  for (const auto agent : sim_.getAgents()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.id = i;
    marker.ns = "mpc";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    auto position =
      agent.second->getPosition() + Vector2(agent.second->getAgentConfig().base_link_to_center, 0.0)
                                      .rotate(agent.second->getOrientationYaw());
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    double diameter = agent.second->getAgentConfig().radius * 2;
    marker.scale.x = diameter;  // diameter in x direction
    marker.scale.y =
      diameter * agent.second->getAgentConfig().ellipticity;  // diameter in x direction
    marker.scale.z = 0.5;                                     // height
    marker.pose.orientation = agent.second->getOrientation();
    marker.color.a = 0.5;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.emplace_back(marker);
    ++i;
  }
}

/**
 *
 * @param marker_array
 */
void VisualizeMarker::visualizeObstacles(
  visualization_msgs::msg::MarkerArray & marker_array, RVOSimulator & sim_)
{
  visualization_msgs::msg::Marker obstacle_marker;
  obstacle_marker.header.frame_id = "map";
  obstacle_marker.header.stamp = rclcpp::Clock().now();
  obstacle_marker.id = 0;
  obstacle_marker.ns = "obstacle";
  obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
  obstacle_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  obstacle_marker.scale.x = 0.2;
  obstacle_marker.color.a = 1;
  obstacle_marker.color.r = 1;
  obstacle_marker.color.g = 1;
  obstacle_marker.color.b = 0;
  auto lines = sim_.getObstacleLines();
  for (auto p : lines) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = 0.0;
    obstacle_marker.points.emplace_back(pt);
  }
  obstacle_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(obstacle_marker);
}

void VisualizeMarker::visualizeObstacleORCALine(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.ns = "obstacle_orca";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 0.5;

  for (auto agent : sim_.getAgents()) {
    auto pos = agent.second->getPosition();
    for (auto line : agent.second->obst_orca_lines_) {
      geometry_msgs::msg::Point msg;
      msg.x = pos.x() + line.point.x() - line.direction.x() * 2.0f;
      msg.y = pos.y() + line.point.y() - line.direction.y() * 2.0f;
      msg.z = 0.0;
      marker.points.push_back(msg);

      msg.x += line.direction.x() * 4.0f;
      msg.y += line.direction.y() * 4.0f;
      marker.points.emplace_back(msg);
    }
  }
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(marker);
}

void VisualizeMarker::visualizeAgentORCALine(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.ns = "agent_orca";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 1;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  for (auto agent : sim_.getAgents()) {
    auto pos = agent.second->getPosition();
    for (auto line : agent.second->agent_orca_lines_) {
      geometry_msgs::msg::Point msg;
      msg.x = pos.x() + line.point.x() - line.direction.x() * 2.0f;
      msg.y = pos.y() + line.point.y() - line.direction.y() * 2.0f;
      msg.z = 0.0;
      marker.points.push_back(msg);

      msg.x += line.direction.x() * 4.0f;
      msg.y += line.direction.y() * 4.0f;
      marker.points.emplace_back(msg);
    }
  }
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(marker);
}

void VisualizeMarker::visualizeVelocitySpace(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  namespace bg = boost::geometry;
  typedef bg::model::d2::point_xy<double> point;
  typedef bg::model::polygon<point> polygon;
  //typedef bg::model::box<point> box;
  namespace buffer = boost::geometry::strategy::buffer;

  typedef boost::geometry::model::d2::point_xy<double> point;
  typedef boost::geometry::model::polygon<point> polygon;

  int index = -1;
  //#pragma omp parallel for
  for (auto agent : sim_.getAgents()) {
    index += 1;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.id = index;
    marker.ns = "velocity_space";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 1;
    marker.color.b = 0.5;

    auto pos = agent.second->getPosition();
    const double radius = 10.0;  // radius of circle
    point pt;
    pt.x(pos.x());
    pt.y(pos.y());
    boost::geometry::model::multi_polygon<polygon> result;
    {
      const int points_per_circle = 36;
      buffer::distance_symmetric<double> distance_strategy(radius);
      buffer::join_round join_strategy(points_per_circle);
      buffer::end_round end_strategy(points_per_circle);
      buffer::point_circle circle_strategy(points_per_circle);
      buffer::side_straight side_strategy;
      boost::geometry::buffer(
        pt, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
    }
    if (result.empty()) {
      return;
    }
    const auto max_velocity_circle = result.front();
    auto velocity_space = max_velocity_circle;
    for (auto line : agent.second->agent_orca_lines_) {
      RVO::Vector2 scaled_dir = line.direction * (radius * 2);
      RVO::Vector2 scaled_dir_norm(scaled_dir.y(), -scaled_dir.x());

      polygon orca_line;
      auto base_point = pos + line.point;
      auto p1 = base_point + scaled_dir;
      auto p2 = base_point - scaled_dir;
      auto p3 = p2 + scaled_dir_norm;
      auto p4 = p1 + scaled_dir_norm;
      orca_line.outer().push_back({p1.x(), p1.y()});
      orca_line.outer().push_back({p2.x(), p2.y()});
      orca_line.outer().push_back({p3.x(), p3.y()});
      orca_line.outer().push_back({p4.x(), p4.y()});

      std::vector<polygon> out;
      bg::intersection(velocity_space, orca_line, out);
      if (out.size() == 1) {
        velocity_space = out[0];
      }
    }

    for (auto line : agent.second->obst_orca_lines_) {
      RVO::Vector2 scaled_dir = line.direction * (radius * 2);
      RVO::Vector2 scaled_dir_norm(scaled_dir.y(), -scaled_dir.x());

      polygon orca_line;
      auto base_point = pos + line.point;
      auto p1 = base_point + scaled_dir;
      auto p2 = base_point - scaled_dir;
      auto p3 = p2 + scaled_dir_norm;
      auto p4 = p1 + scaled_dir_norm;
      orca_line.outer().push_back({p1.x(), p1.y()});
      orca_line.outer().push_back({p2.x(), p2.y()});
      orca_line.outer().push_back({p3.x(), p3.y()});
      orca_line.outer().push_back({p4.x(), p4.y()});

      std::vector<polygon> out;
      bg::intersection(velocity_space, orca_line, out);
      if (out.size() == 1) {
        velocity_space = out[0];
      }
    }

    for (auto p : velocity_space.outer()) {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = p.x();
      point_msg.y = p.y();
      point_msg.z = 0.0;
      marker.points.push_back(point_msg);
    }
    geometry_msgs::msg::Point first_point_msg;
    first_point_msg.x = velocity_space.outer().front().x();
    first_point_msg.y = velocity_space.outer().front().y();
    first_point_msg.z = 0.0;
    marker.points.push_back(first_point_msg);
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.emplace_back(marker);
  }
}

void VisualizeMarker::visualizeAgentVelocity(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.ns = "velocity";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  for (auto agent : sim_.getAgents()) {
    auto vel = agent.second->getVelocity();
    auto pos = agent.second->getPosition();
    geometry_msgs::msg::Point msg;
    msg.x = pos.x();
    msg.y = pos.y();
    msg.z = 1.0;
    marker.points.push_back(msg);

    msg.x += vel.x();
    msg.y += vel.y();
    marker.points.emplace_back(msg);
  }
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(marker);
}

void VisualizeMarker::visualizeAgentPrefVelocity(
  visualization_msgs::msg::MarkerArray & marker_array, const RVOSimulator & sim_)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.ns = "pref_velocity";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 1;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  for (auto agent : sim_.getAgents()) {
    auto pref_vel = agent.second->getPrefVelocity();
    auto pos = agent.second->getPosition();
    geometry_msgs::msg::Point msg;
    msg.x = pos.x();
    msg.y = pos.y();
    msg.z = 1.0;
    marker.points.push_back(msg);

    msg.x += pref_vel.x();
    msg.y += pref_vel.y();
    marker.points.push_back(msg);
  }
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(marker);
}

/**
 *
 */
void VisualizeMarker::visualizeAllMarkers(
  visualization_msgs::msg::MarkerArray & marker_array, RVOSimulator & sim_)
{
  visualizeAgents(marker_array, sim_);
  visualizeObstacles(marker_array, sim_);
  visualizeObstacleORCALine(marker_array, sim_);
  visualizeAgentORCALine(marker_array, sim_);
  visualizeVelocitySpace(marker_array, sim_);
  visualizeAgentVelocity(marker_array, sim_);
  visualizeAgentPrefVelocity(marker_array, sim_);
}

void VisualizeMarker::visualizeNextGoal(
  visualization_msgs::msg::MarkerArray & marker_array, const geometry_msgs::msg::Point & goal_point)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.ns = "nextgoal";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;

  marker.pose.position = goal_point;
  marker.pose.orientation.w = 1.0;

  double diameter = 0.5;
  marker.scale.x = diameter;  // diameter in x direction
  marker.scale.y = diameter;  // diameter in x direction
  marker.scale.z = 1;         // height

  marker.color.a = 0.5;
  marker.color.r = 0;
  marker.color.g = 1.0;
  marker.color.b = 0;
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  marker_array.markers.emplace_back(marker);
}

void VisualizeMarker::visualizeGoalPoses(
  visualization_msgs::msg::MarkerArray & marker_array,
  const std::vector<geometry_msgs::msg::Pose> & goal_poses)
{
  int id = 0;
  for (const auto goal_pose : goal_poses) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.id = id;
    marker.ns = "goalposes";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;

    marker.pose.position = goal_pose.position;
    marker.pose.orientation.w = 1.0;

    double diameter = 0.15;
    marker.scale.x = diameter;  // diameter in x direction
    marker.scale.y = diameter;  // diameter in x direction
    marker.scale.z = 1;         // height

    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.emplace_back(marker);
    ++id;
  }
}

}  // namespace RVO

#endif  // CUSTOMIZED_RVO2__VISUALIZEMARKER_CPP_
