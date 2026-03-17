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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__PERCEPTION_REPRODUCER_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__PERCEPTION_REPRODUCER_SENSOR_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/bag_stream.hpp>
#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/traffic_light_bag_stream.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

namespace simple_sensor_simulator
{
inline namespace experimental
{

class TFStreamFromOdometry : public BagStreamBase<nav_msgs::msg::Odometry>
{
public:
  TFStreamFromOdometry(
    const std::string & topic_name, const std::string & frame_id, rclcpp::Node & node)
  : BagStreamBase<nav_msgs::msg::Odometry>(topic_name),
    frame_id_(frame_id),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node))
  {
  }

  auto broadcastTf(double time_s, const rclcpp::Time & ros_time) -> geometry_msgs::msg::Pose;

  auto findNearestIndex(const geometry_msgs::msg::Pose & ego_pose) const -> size_t
  {
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t nearest = 0;
    for (size_t i = 0; i < data_.size(); ++i) {
      const auto & pos = data_[i].second.pose.pose.position;
      const double dx = pos.x - ego_pose.position.x;
      const double dy = pos.y - ego_pose.position.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        nearest = i;
      }
    }
    return nearest;
  }

  auto getTimeAt(size_t idx) const -> double { return data_[idx].first; }

  auto getPoseAt(size_t idx) const -> const geometry_msgs::msg::Pose &
  {
    return data_[idx].second.pose.pose;
  }

  auto reset() -> void { index_ = 0; }

protected:
  auto pushMessage(double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data)
    -> void override;

private:
  const std::string frame_id_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  size_t index_ = 0;
};

class PerceptionReproducerSensor
{
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

public:
  struct ReplayConfig
  {
    // Use position-based data selection instead of time-sequential replay
    bool use_position_based_replay = false;
  };

  PerceptionReproducerSensor(
    const std::string & bag_path, double start_time_s, const ReplayConfig & config,
    rclcpp::Node & node);

  auto update(
    double current_scenario_time, const rclcpp::Time & current_ros_time,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) -> void;

  auto reset() -> void;

private:
  static constexpr const char * detected_objects_topic_ =
    "/perception/object_recognition/detection/objects";

  static constexpr const char * odometry_topic_ = "/localization/kinematic_state";

  static constexpr const char * trajectory_topic_ = "/planning/trajectory";

  static constexpr const char * occupancy_grid_topic_ = "/perception/occupancy_grid_map/map";

  static constexpr const char * traffic_light_topic_ =
    "/perception/traffic_light_recognition/traffic_signals";

  auto loadAllBagData(const std::string & bag_path, double start_time_s) -> void;

  auto updateTimeBased(double current_scenario_time, const rclcpp::Time & current_ros_time) -> void;

  auto updatePositionBased(
    const geometry_msgs::msg::Pose & ego_pose, double current_scenario_time,
    const rclcpp::Time & current_ros_time) -> void;

  auto publishVehicleMarker(
    const geometry_msgs::msg::Pose & pose, const rclcpp::Time & ros_time) const -> void;

  rclcpp::Logger logger_;

  ReplayConfig config_;

  BagStream<DetectedObjects> detected_objects_stream_;

  BagStream<Trajectory> trajectory_stream_;

  TFStreamFromOdometry odometry_stream_;

  BagStream<OccupancyGrid> occupancy_grid_stream_;

#ifdef PERCEPTION_REPRODUCER_HAS_TRAFFIC_LIGHT_GROUP_ARRAY
  std::unique_ptr<TrafficLightBagStream> traffic_light_stream_;
#endif

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_marker_pub_;
};

}  // namespace experimental
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__PERCEPTION_REPRODUCER_SENSOR__PERCEPTION_REPRODUCER_SENSOR_HPP_
