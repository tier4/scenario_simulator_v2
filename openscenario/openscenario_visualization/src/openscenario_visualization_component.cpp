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
  for (const auto & name : erase_names)
  {
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
      color = color_utils::makeColorMsg("forestgreen", 0.9);
      break;
    case status.PEDESTRIAN:
      color = color_utils::makeColorMsg("orange", 0.9);
      break;
    case status.VEHICLE:
      color = color_utils::makeColorMsg("steelblue", 0.9);
      break;
  }
  visualization_msgs::msg::Marker bbox;
  bbox.header.frame_id = status.name;
  bbox.header.stamp = stamp;
  bbox.ns = status.name;
  bbox.id = 0;
  bbox.action = bbox.ADD;
  bbox.pose.position.x = status.bounding_box.center.x;
  bbox.pose.position.y = status.bounding_box.center.y;
  bbox.pose.position.z = status.bounding_box.center.z;
  bbox.pose.orientation.x = 0.0;
  bbox.pose.orientation.y = 0.0;
  bbox.pose.orientation.z = 0.0;
  bbox.pose.orientation.w = 1.0;
  bbox.type = bbox.CUBE;
  bbox.lifetime = rclcpp::Duration(0.1);
  bbox.scale.x = status.bounding_box.dimensions.x;
  bbox.scale.y = status.bounding_box.dimensions.y;
  bbox.scale.z = status.bounding_box.dimensions.z;
  bbox.color = color;
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
    status.bounding_box.dimensions.z * 0.5 + 0.8;
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
  return ret;
}

const visualization_msgs::msg::MarkerArray OpenscenarioVisualizationComponent::generateDeleteMarker()
const
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}
}  // status.ns openscenario_visualization

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_visualization::OpenscenarioVisualizationComponent)
