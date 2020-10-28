// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <openscenario_visualization/openscenario_visualization_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <string>
#include <vector>
#include <algorithm>

namespace openscenario_visualization
{
OpenscenarioVisualizationComponent::OpenscenarioVisualizationComponent(
  const rclcpp::NodeOptions & options)
: Node("openscenario_visualization", options)
{
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/entity/marker", 1);
  entity_status_sub_ = this->create_subscription<openscenario_msgs::msg::EntityStatusArray>(
    "/entity/status", 1,
    std::bind(&OpenscenarioVisualizationComponent::entityStatusCallback, this,
    std::placeholders::_1));
}

void OpenscenarioVisualizationComponent::entityStatusCallback(
  const openscenario_msgs::msg::EntityStatusArray::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray current_marker;
  std::vector<std::string> entity_name_lists;
  for (const auto & status : msg->status) {
    entity_name_lists.emplace_back(status.name);
  }
  std::vector<std::string> erase_names;
  for (const auto & marker : markers_) {
    auto itr = std::find(entity_name_lists.begin(), entity_name_lists.end(), marker.first);
    if (itr == entity_name_lists.end()) {
      auto delete_marker = generateDeleteMarker(marker.first);
      std::copy(delete_marker.markers.begin(), delete_marker.markers.end(),
        std::back_inserter(current_marker.markers));
      erase_names.emplace_back(marker.first);
    }
  }
  for (const auto & name : erase_names) {
    markers_.erase(markers_.find(name));
  }
  for (const auto & status : msg->status) {
    auto marker_array = generateMarker(status);
    std::copy(marker_array.markers.begin(), marker_array.markers.end(),
      std::back_inserter(current_marker.markers));
    markers_[status.name] = marker_array;
  }
  marker_pub_->publish(current_marker);
}

const visualization_msgs::msg::MarkerArray OpenscenarioVisualizationComponent::generateDeleteMarker(
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

const visualization_msgs::msg::MarkerArray OpenscenarioVisualizationComponent::generateMarker(
  const openscenario_msgs::msg::EntityStatus & status)
{
  auto ret = visualization_msgs::msg::MarkerArray();
  auto stamp = get_clock()->now();
  std_msgs::msg::ColorRGBA color;
  switch (status.type) {
    case status.EGO:
      color = color_utils::makeColorMsg("forestgreen", 0.99);
      break;
    case status.PEDESTRIAN:
      color = color_utils::makeColorMsg("orange", 0.99);
      break;
    case status.VEHICLE:
      color = color_utils::makeColorMsg("steelblue", 0.99);
      break;
  }
  visualization_msgs::msg::Marker bbox;
  bbox.header.frame_id = status.name;
  bbox.header.stamp = stamp;
  bbox.ns = status.name;
  bbox.id = 0;
  bbox.action = bbox.ADD;
  bbox.pose.orientation.x = 0.0;
  bbox.pose.orientation.y = 0.0;
  bbox.pose.orientation.z = 0.0;
  bbox.pose.orientation.w = 1.0;
  bbox.type = bbox.LINE_LIST;
  bbox.lifetime = rclcpp::Duration(0.1);
  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;

  p0.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p0.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p0.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;

  p1.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p1.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p1.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;

  p2.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p2.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p2.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;

  p3.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p3.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p3.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;

  p4.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  p4.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p4.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;

  p5.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p5.y = status.bounding_box.center.y + status.bounding_box.dimensions.y * 0.5;
  p5.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;

  p6.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p6.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p6.z = status.bounding_box.center.z + status.bounding_box.dimensions.z * 0.5;

  p7.x = status.bounding_box.center.x - status.bounding_box.dimensions.x * 0.5;
  p7.y = status.bounding_box.center.y - status.bounding_box.dimensions.y * 0.5;
  p7.z = status.bounding_box.center.z - status.bounding_box.dimensions.z * 0.5;

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
  ret.markers.push_back(bbox);

  visualization_msgs::msg::Marker text;
  text.header.frame_id = status.name;
  text.header.stamp = stamp;
  text.ns = status.name;
  text.id = 1;
  text.action = text.ADD;
  text.pose.position.x = status.bounding_box.center.x;
  text.pose.position.y = status.bounding_box.center.y;
  text.pose.position.z = status.bounding_box.center.z +
    status.bounding_box.dimensions.z * 0.5 + 1.0;
  text.pose.orientation.x = 0.0;
  text.pose.orientation.y = 0.0;
  text.pose.orientation.z = 0.0;
  text.pose.orientation.w = 1.0;
  text.type = text.TEXT_VIEW_FACING;
  text.scale.x = 0.0;
  text.scale.y = 0.0;
  text.scale.z = 1.0;
  text.lifetime = rclcpp::Duration(0.1);
  text.text = status.name;
  text.color = color;
  ret.markers.push_back(text);

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = status.name;
  arrow.header.stamp = stamp;
  arrow.ns = status.name;
  arrow.id = 2;
  arrow.action = arrow.ADD;
  arrow.pose.position.x = status.bounding_box.center.x + status.bounding_box.dimensions.x * 0.5;
  arrow.pose.position.y = status.bounding_box.center.y;
  arrow.pose.position.z = status.bounding_box.center.z;
  arrow.pose.orientation.x = 0.0;
  arrow.pose.orientation.y = 0.0;
  arrow.pose.orientation.z = 0.0;
  arrow.pose.orientation.w = 1.0;
  arrow.type = arrow.ARROW;
  arrow.scale.x = 1.5;
  arrow.scale.y = 0.1;
  arrow.scale.z = 0.1;
  arrow.lifetime = rclcpp::Duration(0.1);
  arrow.color = color_utils::makeColorMsg("red", 0.99);
  ret.markers.push_back(arrow);
  return ret;
}

const visualization_msgs::msg::MarkerArray
OpenscenarioVisualizationComponent::generateDeleteMarker() const
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}
}  // namespace openscenario_visualization

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_visualization::OpenscenarioVisualizationComponent)
