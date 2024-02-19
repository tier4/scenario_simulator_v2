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
 * @brief definition of visualizer component
 */

#ifndef TRAFFIC_SIMULATOR__VISUALIZATION__VISUALIZATION_COMPONENT_HPP_
#define TRAFFIC_SIMULATOR__VISUALIZATION__VISUALIZATION_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_EXPORT __attribute__((dllexport))
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_EXPORT __declspec(dllexport)
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_BUILDING_DLL
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC \
  TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_EXPORT
#else
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC \
  TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_IMPORT
#endif
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC_TYPE \
  TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_LOCAL
#else
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_EXPORT __attribute__((visibility("default")))
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_LOCAL
#endif
#define TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
/**
 * @brief ROS 2 component for visualizing simulation result.
 */
class VisualizationComponent : public rclcpp::Node
{
public:
  TRAFFIC_SIMULATOR_VISUALIZATION_COMPONENT_PUBLIC
  explicit VisualizationComponent(const rclcpp::NodeOptions &);

private:
  /**
   * @brief callback function when subscribe entity status array.
   * @param msg entity status array message from openscenario interpreter.
   */
  void entityStatusCallback(
    const traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray::ConstSharedPtr msg);
  /**
   * @brief generate delete marker for target namespace.
   * @param ns namespace of the marker which you want to delete.
   * @return const visualization_msgs::msg::MarkerArray delete marker messages.
   */
  const visualization_msgs::msg::MarkerArray generateDeleteMarker(std::string ns);
  /**
   * @brief generate delete marker for all namespace.
   * @return const visualization_msgs::msg::MarkerArray delete marker messages. (action is DELETE_ALL)
   */
  const visualization_msgs::msg::MarkerArray generateDeleteMarker() const;
  /**
   * @brief generate marker from entity status
   * @param status entity status message
   * @param waypoints waypoints message
   * @param obstacle obstacles in waypoint
   * @return const visualization_msgs::msg::MarkerArray markers which describes entity bounding box and it's status.
   */
  int goal_pose_max_size = 0;
  const visualization_msgs::msg::MarkerArray generateMarker(
    const traffic_simulator_msgs::msg::EntityStatus & status,
    const std::vector<geometry_msgs::msg::Pose> & goal_pose,
    const traffic_simulator_msgs::msg::WaypointsArray & waypoints,
    const traffic_simulator_msgs::msg::Obstacle & obstacle, bool obstacle_find);
  /**
   * @brief publisher of marker topic.
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  /**
   * @brief subscriber of entity status array topic.
   */
  rclcpp::Subscription<traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray>::SharedPtr
    entity_status_sub_;
  /**
   * @brief buffers for generated markers.
   */
  std::unordered_map<std::string, visualization_msgs::msg::MarkerArray> markers_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__VISUALIZATION__VISUALIZATION_COMPONENT_HPP_
