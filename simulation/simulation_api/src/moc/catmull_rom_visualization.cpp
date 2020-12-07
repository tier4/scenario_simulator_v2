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
#include <simulation_api/math/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class CatmullRomSplineVisualization : public rclcpp::Node
{
public:
  explicit CatmullRomSplineVisualization(const rclcpp::NodeOptions & option)
  : Node("catumull_rom_spline_viz", option)
  {
    marker_pub_ptr_ = create_publisher<visualization_msgs::msg::MarkerArray>("/spline/marker", 1);
    clicked_points_sub_ptr_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 1,
      std::bind(&CatmullRomSplineVisualization::ClickedPointsCallback, this,
      std::placeholders::_1));
  }

private:
  std::string frame_;
  std::vector<geometry_msgs::msg::Point> points_;
  void ClickedPointsCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (frame_ == "") {
      frame_ = msg->header.frame_id;
      points_.emplace_back(msg->point);
    } else {
      if (frame_ != msg->header.frame_id) {
        RCLCPP_ERROR(get_logger(), "frame does not match.");
      } else {
        points_.emplace_back(msg->point);
      }
    }
    if (points_.size() >= 3) {
      auto spline = simulation_api::math::CatmullRomSpline(points_);
      marker_pub_ptr_->publish(generateDeleteMarker());
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_points_sub_ptr_;
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
