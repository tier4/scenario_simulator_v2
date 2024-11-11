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

/**
 * @mainpage ROS 2 visualization component for OpenSCENARIO entities
 * @tableofcontents
 * @image html images/rviz.png width=1280px height=540px
 * @author Masaya Kataoka
 * @date 2020-11-19
 * @section interface
  <table>
    <caption id="multi_row">ROS 2 Topic interface</caption>
    <tr>
      <th>Name</th>
      <th>Type</th>
      <th>Pub/Sub</th>
      <th>description</th>
    </tr>
    <tr>
      <td>/entity/marker</td>
      <td>visualization_msgs/msg/MarkerArray</td>
      <td>Publish</td>
      <td>Visualization results of the marker.</td>
    </tr>
    <tr>
      <td>/entity/status</td>
      <td>traffic_simulator_msgs/msg/EntityStatusWithTrajectoryArray</td>
      <td>Subscribe</td>
      <td>Topics for publishing entity status in simulation.</td>
    </tr>
  </table>
 */

#include <algorithm>
#include <cmath>
#include <color_names/color_names.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <traffic_simulator/visualization/visualization_component.hpp>
#include <vector>

namespace traffic_simulator
{
VisualizationComponent::VisualizationComponent(const rclcpp::NodeOptions & options)
: Node("visualization", options)
{
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("entity/marker", 1);
  entity_status_sub_ =
    this->create_subscription<traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray>(
      "entity/status", 1,
      std::bind(&VisualizationComponent::entityStatusCallback, this, std::placeholders::_1));
}

void VisualizationComponent::entityStatusCallback(
  const traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray::ConstSharedPtr msg)
{
  visualization_msgs::msg::MarkerArray current_marker;
  std::vector<std::string> entity_name_lists;
  for (const auto & data : msg->data) {
    entity_name_lists.emplace_back(data.status.name);
  }
  std::vector<std::string> erase_names;
  for (const auto & marker : markers_) {
    auto itr = std::find(entity_name_lists.begin(), entity_name_lists.end(), marker.first);
    if (itr == entity_name_lists.end()) {
      auto delete_marker = generateDeleteMarker(marker.first);
      std::copy(
        delete_marker.markers.begin(), delete_marker.markers.end(),
        std::back_inserter(current_marker.markers));
      erase_names.emplace_back(marker.first);
    }
  }
  for (const auto & name : erase_names) {
    markers_.erase(markers_.find(name));
  }
  for (const auto & data : msg->data) {
    auto marker_array =
      generateMarker(data.status, data.goal_pose, data.waypoint, data.obstacle, data.obstacle_find);
    std::copy(
      marker_array.markers.begin(), marker_array.markers.end(),
      std::back_inserter(current_marker.markers));
    markers_[data.name] = marker_array;
  }
  marker_pub_->publish(current_marker);
}

const visualization_msgs::msg::MarkerArray VisualizationComponent::generateDeleteMarker(
  std::string ns)
{
  auto ret = visualization_msgs::msg::MarkerArray();
  auto stamp = get_clock()->now();
  for (const auto & marker : markers_[ns].markers) {
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.action = marker_msg.DELETE;
    marker_msg.header.frame_id = ns;
    marker_msg.header.stamp = stamp;
    marker_msg.ns = marker.ns;
    marker_msg.id = marker.id;
    ret.markers.emplace_back(marker_msg);
  }
  return ret;
}

const visualization_msgs::msg::MarkerArray VisualizationComponent::generateMarker(
  const traffic_simulator_msgs::msg::EntityStatus & status,
  const std::vector<geometry_msgs::msg::Pose> & goal_pose,
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
  const traffic_simulator_msgs::msg::Obstacle & obstacle, bool obstacle_find)
{
  constexpr auto default_quaternion = rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY;
  auto ret = visualization_msgs::msg::MarkerArray();
  auto stamp = get_clock()->now();

  const auto color = [&]() {
    switch (status.type.type) {
      case status.type.EGO:
        return color_names::makeColorMsg("limegreen", 0.99);
      case status.type.PEDESTRIAN:
        return color_names::makeColorMsg("orange", 0.99);
      case status.type.VEHICLE:
        return color_names::makeColorMsg("lightskyblue", 0.99);
      default:
      case status.type.MISC_OBJECT:
        return color_names::makeColorMsg("magenta", 0.99);
    }
  }();

  if (goal_pose.size() != 0) {
    goal_pose_max_size = std::max(goal_pose_max_size, int(goal_pose.size()));
    for (std::vector<geometry_msgs::msg::Pose>::size_type i = 0; i < unsigned(goal_pose_max_size);
         i++) {
      if (i < goal_pose.size()) {
        visualization_msgs::msg::Marker goal_pose_marker;
        goal_pose_marker.header.frame_id = "map";
        goal_pose_marker.header.stamp = stamp;
        goal_pose_marker.ns = status.name;
        goal_pose_marker.id = 10 + int(goal_pose_max_size - goal_pose.size() + i);
        goal_pose_marker.action = goal_pose_marker.ADD;
        goal_pose_marker.type = 0;  //arrow
        goal_pose_marker.pose = goal_pose[i];
        goal_pose_marker.color = color;
        goal_pose_marker.scale.x = 1.6;
        goal_pose_marker.scale.y = 0.2;
        goal_pose_marker.scale.z = 0.2;
        goal_pose_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        ret.markers.emplace_back(goal_pose_marker);

        visualization_msgs::msg::Marker goal_pose_text_marker;
        goal_pose_text_marker.type = goal_pose_text_marker.TEXT_VIEW_FACING;
        goal_pose_text_marker.header.frame_id = "map";
        goal_pose_text_marker.header.stamp = stamp;
        goal_pose_text_marker.ns = status.name;
        goal_pose_text_marker.id = 100 + int(goal_pose_max_size - goal_pose.size() + i);
        goal_pose_text_marker.action = goal_pose_text_marker.ADD;
        goal_pose_text_marker.pose.position.x = goal_pose[i].position.x;
        goal_pose_text_marker.pose.position.y = goal_pose[i].position.y;
        goal_pose_text_marker.pose.position.z = goal_pose[i].position.z + 1.0;
        goal_pose_text_marker.pose.orientation = geometry_msgs::msg::Quaternion(default_quaternion);
        goal_pose_text_marker.type = goal_pose_text_marker.TEXT_VIEW_FACING;
        goal_pose_text_marker.scale.x = 0.0;
        goal_pose_text_marker.scale.y = 0.0;
        goal_pose_text_marker.scale.z = 0.6;
        goal_pose_text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        goal_pose_text_marker.text =
          status.name + "_goal_" + std::to_string(int(goal_pose_max_size - goal_pose.size() + i));
        goal_pose_text_marker.color = color_names::makeColorMsg("white", 0.99);
        ret.markers.emplace_back(goal_pose_text_marker);
      } else {
        visualization_msgs::msg::Marker goal_pose_marker;
        goal_pose_marker.action = goal_pose_marker.DELETE;
        goal_pose_marker.id = 10 + int(i - (goal_pose_max_size - goal_pose.size()));
        goal_pose_marker.ns = status.name;
        ret.markers.emplace_back(goal_pose_marker);
        visualization_msgs::msg::Marker goal_pose_text_marker;
        goal_pose_text_marker.action = goal_pose_text_marker.DELETE;
        goal_pose_text_marker.id = 100 + int(i - (goal_pose_max_size - goal_pose.size()));
        goal_pose_text_marker.ns = status.name;
        ret.markers.emplace_back(goal_pose_text_marker);
      }
    }
  } else {
    visualization_msgs::msg::Marker goal_pose_marker;
    goal_pose_marker.action = goal_pose_marker.DELETE;
    goal_pose_marker.id = 10 + int(goal_pose_max_size - 1);
    goal_pose_marker.ns = status.name;
    ret.markers.emplace_back(goal_pose_marker);
    visualization_msgs::msg::Marker goal_pose_text_marker;
    goal_pose_text_marker.action = goal_pose_text_marker.DELETE;
    goal_pose_text_marker.id = 100 + int(goal_pose_max_size - 1);
    goal_pose_text_marker.ns = status.name;
    ret.markers.emplace_back(goal_pose_text_marker);
  }

  visualization_msgs::msg::Marker bbox;
  bbox.header.frame_id = "map";
  bbox.header.stamp = stamp;
  bbox.ns = status.name;
  bbox.id = 0;
  bbox.action = bbox.ADD;
  bbox.type = bbox.LINE_LIST;
  bbox.lifetime = rclcpp::Duration::from_seconds(0.1);
  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;

  p0.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p0.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p0.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  p0 = math::geometry::transformPoint(status.pose, p0);

  p1.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p1.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p1.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  p1 = math::geometry::transformPoint(status.pose, p1);

  p2.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p2.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p2.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  p2 = math::geometry::transformPoint(status.pose, p2);

  p3.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p3.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p3.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  p3 = math::geometry::transformPoint(status.pose, p3);

  p4.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p4.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p4.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  p4 = math::geometry::transformPoint(status.pose, p4);

  p5.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p5.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p5.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  p5 = math::geometry::transformPoint(status.pose, p5);

  p6.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p6.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p6.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;
  p6 = math::geometry::transformPoint(status.pose, p6);

  p7.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p7.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p7.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  p7 = math::geometry::transformPoint(status.pose, p7);

  bbox.points.emplace_back(p0);
  bbox.points.emplace_back(p3);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p3);
  bbox.points.emplace_back(p6);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p6);
  bbox.points.emplace_back(p2);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p2);
  bbox.points.emplace_back(p0);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p0);
  bbox.points.emplace_back(p1);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p3);
  bbox.points.emplace_back(p5);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p6);
  bbox.points.emplace_back(p7);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p2);
  bbox.points.emplace_back(p4);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p1);
  bbox.points.emplace_back(p5);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p5);
  bbox.points.emplace_back(p7);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p7);
  bbox.points.emplace_back(p4);
  bbox.colors.emplace_back(color);

  bbox.points.emplace_back(p4);
  bbox.points.emplace_back(p1);
  bbox.colors.emplace_back(color);

  bbox.color = color;
  bbox.scale.x = 0.1;
  bbox.scale.y = 0.1;
  bbox.scale.z = 0.1;
  ret.markers.emplace_back(bbox);

  visualization_msgs::msg::Marker text;
  text.header.frame_id = "map";
  text.header.stamp = stamp;
  text.ns = status.name;
  text.id = 1;
  text.action = text.ADD;
  text.pose.position.x = status.bounding_box.center.x;
  text.pose.position.y = status.bounding_box.center.y;
  text.pose.position.z =
    status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5 + 1.0;
  text.pose.position = math::geometry::transformPoint(status.pose, text.pose.position);
  text.pose.orientation = status.pose.orientation;
  text.type = text.TEXT_VIEW_FACING;
  text.scale.x = 0.0;
  text.scale.y = 0.0;
  text.scale.z = 0.6;
  text.lifetime = rclcpp::Duration::from_seconds(0.1);
  text.text = status.name;
  text.color = color_names::makeColorMsg("white", 0.99);
  ret.markers.emplace_back(text);

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "map";
  arrow.header.stamp = stamp;
  arrow.ns = status.name;
  arrow.id = 2;
  arrow.action = arrow.ADD;

  // constexpr double arrow_size = 0.3;
  double arrow_size = 0.4 * status.bounding_box.dimensions.y;
  constexpr double arrow_ratio = 1.0;
  geometry_msgs::msg::Point pf, pl, pr;
  pf.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5 + 1.0;
  pf.y = status.bounding_box.center.y;
  pf.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  pf = math::geometry::transformPoint(status.pose, pf);

  pl.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5 + 1.0 -
         arrow_size * arrow_ratio;
  pl.y = status.bounding_box.center.y + arrow_size;
  pl.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  pl = math::geometry::transformPoint(status.pose, pl);

  pr.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5 + 1.0 -
         arrow_size * arrow_ratio;
  pr.y = status.bounding_box.center.y - arrow_size;
  pr.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;
  pr = math::geometry::transformPoint(status.pose, pr);
  arrow.points = {pf, pl, pr};
  arrow.colors = {color};
  arrow.pose.orientation = status.pose.orientation;
  arrow.type = arrow.TRIANGLE_LIST;
  arrow.scale.x = 1.0;
  arrow.scale.y = 1.0;
  arrow.scale.z = 1.0;
  arrow.lifetime = rclcpp::Duration::from_seconds(0.1);
  arrow.color = color_names::makeColorMsg("red", 0.99);
  ret.markers.emplace_back(arrow);

  visualization_msgs::msg::Marker text_action;
  text_action.header.frame_id = "map";
  text_action.header.stamp = stamp;
  text_action.ns = status.name;
  text_action.id = 3;
  text_action.action = text_action.ADD;
  text_action.pose.position =
    math::geometry::transformPoint(status.pose, status.bounding_box.center);
  text_action.pose.orientation = status.pose.orientation;
  text_action.type = text_action.TEXT_VIEW_FACING;
  text_action.scale.x = 0.0;
  text_action.scale.y = 0.0;
  text_action.scale.z = 0.4;
  text_action.lifetime = rclcpp::Duration::from_seconds(0.1);
  text_action.text = status.action_status.current_action;
  if (status.lanelet_pose_valid) {
    text_action.text = text_action.text + "\nid:" + std::to_string(status.lanelet_pose.lanelet_id) +
                       "\ns:" + std::to_string(status.lanelet_pose.s) +
                       "\noffset:" + std::to_string(status.lanelet_pose.offset);
  }
  const auto & velocity = status.action_status.twist.linear;
  double speed =
    std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z) * 3600. /
    1000.;
  text_action.text += "\n" + std::to_string(speed) + "km/h";

  text_action.color = color_names::makeColorMsg("white", 0.99);
  ret.markers.emplace_back(text_action);

  if (waypoints.waypoints.size() > 2) {
    math::geometry::CatmullRomSpline spline(waypoints.waypoints);

    /**
     * @brief generate marker for waypoints
     */
    visualization_msgs::msg::Marker waypoints_marker;
    waypoints_marker.header.frame_id = "map";
    waypoints_marker.header.stamp = stamp;
    waypoints_marker.ns = status.name;
    waypoints_marker.id = 4;
    waypoints_marker.action = waypoints_marker.ADD;
    waypoints_marker.type = waypoints_marker.TRIANGLE_LIST;
    size_t num_points = 20;
    waypoints_marker.points = spline.getPolygon(status.bounding_box.dimensions.y, num_points);
    waypoints_marker.color = color;
    waypoints_marker.color.a = 0.8;
    waypoints_marker.colors =
      std::vector<std_msgs::msg::ColorRGBA>(num_points * 2, waypoints_marker.color);
    waypoints_marker.scale.x = 1.0;
    waypoints_marker.scale.y = 1.0;
    waypoints_marker.scale.z = 1.0;
    ret.markers.emplace_back(waypoints_marker);
    if (obstacle_find) {
      /**
       * @brief generate marker for obstacle
       */
      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.header.frame_id = "map";
      obstacle_marker.header.stamp = stamp;
      obstacle_marker.ns = status.name;
      obstacle_marker.id = 5;
      obstacle_marker.action = obstacle_marker.ADD;
      obstacle_marker.type = obstacle_marker.CUBE;
      obstacle_marker.pose = spline.getPose(obstacle.s);
      obstacle_marker.pose.position.z =
        obstacle_marker.pose.position.z + status.bounding_box.dimensions.z * 0.5;
      obstacle_marker.color = color_names::makeColorMsg("red", 0.5);
      obstacle_marker.scale.x = 0.3;
      obstacle_marker.scale.y = status.bounding_box.dimensions.y + 0.3;
      obstacle_marker.scale.z = status.bounding_box.dimensions.z;
      ret.markers.emplace_back(obstacle_marker);
    } else {
      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.action = obstacle_marker.DELETE;
      obstacle_marker.id = 5;
      obstacle_marker.ns = status.name;
      ret.markers.emplace_back(obstacle_marker);
    }
  } else {
    visualization_msgs::msg::Marker waypoints_marker;
    waypoints_marker.action = waypoints_marker.DELETE;
    waypoints_marker.id = 4;
    waypoints_marker.ns = status.name;
    ret.markers.emplace_back(waypoints_marker);
    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.action = obstacle_marker.DELETE;
    obstacle_marker.id = 5;
    obstacle_marker.ns = status.name;
    ret.markers.emplace_back(obstacle_marker);
  }
  return ret;
}

const visualization_msgs::msg::MarkerArray VisualizationComponent::generateDeleteMarker() const
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.emplace_back(marker);
  return ret;
}
}  // namespace traffic_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_simulator::VisualizationComponent)
