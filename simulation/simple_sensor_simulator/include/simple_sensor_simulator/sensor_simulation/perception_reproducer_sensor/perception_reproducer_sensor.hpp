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

#include <algorithm>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
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

  /// @note The search range can be restricted to [lo, hi) so that the caller can keep a
  /// monotonic playhead: on self-overlapping courses (e.g. a rotary or a loop) the global
  /// nearest sample may belong to a far-away arc of the recording, which would teleport the
  /// replayed objects.
  auto findNearestIndex(
    const geometry_msgs::msg::Pose & ego_pose, size_t lo = 0,
    size_t hi = std::numeric_limits<size_t>::max()) const -> size_t
  {
    double min_dist_sq = std::numeric_limits<double>::max();
    hi = std::min(hi, data_.size());
    lo = std::min(lo, hi > 0 ? hi - 1 : 0);
    size_t nearest = lo;
    for (size_t i = lo; i < hi; ++i) {
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

  auto size() const -> size_t { return data_.size(); }

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
  using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
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
    const std::optional<geometry_msgs::msg::Pose> & ego_pose,
    const std::optional<double> & ego_speed) -> void;

  auto reset() -> void;

private:
  static constexpr const char * detected_objects_topic_ =
    "/perception/object_recognition/detection/objects";

  /// @note Real vehicle logs often record only the tracking output (no detection topic). The
  /// tracked objects are replayed verbatim so that consumers of the tracking topic (e.g.
  /// DiffusionPlanner, map_based_prediction) receive the real-world objects even when the
  /// Autoware tracker stays silent due to the suppressed detection sensor.
  static constexpr const char * tracked_objects_topic_ =
    "/perception/object_recognition/tracking/objects";

  static constexpr const char * odometry_topic_ = "/localization/kinematic_state";

  static constexpr const char * trajectory_topic_ = "/planning/trajectory";

  static constexpr const char * occupancy_grid_topic_ = "/perception/occupancy_grid_map/map";

  static constexpr const char * traffic_light_topic_ =
    "/perception/traffic_light_recognition/traffic_signals";

  auto loadAllBagData(const std::string & bag_path, double start_time_s) -> void;

#ifdef PERCEPTION_REPRODUCER_HAS_TRAFFIC_LIGHT_GROUP_ARRAY
  /// @note Restrict the replayed traffic lights to the groups governing the lanelets that the
  /// recorded ego actually drove. Replaying cross-traffic signals too makes the behavior
  /// planner misinterpret a red cross signal as the ego's own stop signal and falsely stop
  /// (observed with the former Python sidecar reproducer). Applied lazily on the first
  /// update() because the lanelet map is activated by the InitializeRequest, which arrives
  /// after this sensor is constructed.
  auto applyGoverningSignalFilter() -> void;
#endif

  auto updateTimeBased(double current_scenario_time, const rclcpp::Time & current_ros_time) -> void;

  auto updatePositionBased(
    const geometry_msgs::msg::Pose & ego_pose, double ego_speed, double current_scenario_time,
    const rclcpp::Time & current_ros_time) -> void;

  auto publishVehicleMarker(
    const geometry_msgs::msg::Pose & pose, const rclcpp::Time & ros_time) const -> void;

  rclcpp::Logger logger_;

  ReplayConfig config_;

  /// @note Playhead (odometry sample index) for position-based replay. Monotonic: it never
  /// moves backwards, and the nearest-neighbour search is restricted to a window around the
  /// previous playhead so that self-overlapping courses cannot teleport the replay to a
  /// far-away arc of the recording. The window widths are in odometry samples (~50 Hz, so
  /// 50 back ≒ 1 s, 600 forward ≒ 12 s).
  std::optional<size_t> playhead_;

  static constexpr size_t playhead_window_back_ = 50;

  static constexpr size_t playhead_window_forward_ = 600;

  /// @note While the sim ego is stopped (speed <= stop_velocity_threshold_), the playhead
  /// advances at the recording's real pace instead of pose-sync. Pure pose-sync would freeze
  /// the playhead while the ego is stopped, so the recorded lead vehicle would never depart
  /// and the ego would never be released (deadlock). Advancing in recorded time replays the
  /// dwell -> departure sequence exactly as the real vehicle experienced it.
  struct DwellAnchor
  {
    double scenario_time;
    double bag_time;
  };

  std::optional<DwellAnchor> dwell_anchor_;

  static constexpr double stop_velocity_threshold_ = 0.5;  // [m/s]

  bool signal_filter_applied_ = false;

  BagStream<DetectedObjects> detected_objects_stream_;

  BagStream<TrackedObjects> tracked_objects_stream_;

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
