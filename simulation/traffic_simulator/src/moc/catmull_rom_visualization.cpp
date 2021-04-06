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

#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>
#include <memory>

class CatmullRomSplineVisualization : public rclcpp::Node
{
public:
  explicit CatmullRomSplineVisualization(const rclcpp::NodeOptions & option)
  : Node("catumull_rom_spline_viz", option)
  {
    marker_pub_ptr_ = create_publisher<visualization_msgs::msg::MarkerArray>("/spline/marker", 1);
    clicked_points_sub_ptr_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 1,
      std::bind(
        &CatmullRomSplineVisualization::goalPoseCallback, this,
        std::placeholders::_1));
  }

private:
  std::string frame_;
  std::vector<geometry_msgs::msg::Point> points_;
  void goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (frame_ == "") {
      frame_ = msg->header.frame_id;
      points_.emplace_back(msg->pose.position);
    } else {
      if (frame_ != msg->header.frame_id) {
        RCLCPP_ERROR(get_logger(), "frame does not match.");
      } else {
        points_.emplace_back(msg->pose.position);
      }
    }
    if (points_.size() >= 3) {
      marker_pub_ptr_->publish(generateDeleteMarker());
      marker_pub_ptr_->publish(generateSplineMarker());
    }
  }
  const visualization_msgs::msg::MarkerArray
  generateDeleteMarker() const
  {
    visualization_msgs::msg::MarkerArray ret;
    visualization_msgs::msg::Marker marker;
    marker.action = marker.DELETEALL;
    ret.markers.push_back(marker);
    return ret;
  }
  const visualization_msgs::msg::MarkerArray
  generateSplineMarker() const
  {
    visualization_msgs::msg::MarkerArray ret;
    visualization_msgs::msg::Marker marker;
    marker.action = marker.ADD;
    marker.header.frame_id = frame_;
    auto spline = traffic_simulator::math::CatmullRomSpline(points_);
    marker.points = spline.getTrajectory(100);
    marker.type = marker.LINE_STRIP;
    marker.color = color_utils::makeColorMsg("red", 0.99);
    marker.ns = "spiline";
    marker.id = 0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    ret.markers.push_back(marker);
    return ret;
  }
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_points_sub_ptr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<CatmullRomSplineVisualization>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
