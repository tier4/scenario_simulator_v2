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

#ifndef OPENSCENARIO_VISUALIZATION__OPENSCENARIO_VISUALIZATION_COMPONENT_HPP_
#define OPENSCENARIO_VISUALIZATION__OPENSCENARIO_VISUALIZATION_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_EXPORT __attribute__(( \
      dllexport))
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_IMPORT __attribute__(( \
      dllimport))
#else
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_EXPORT __declspec(dllexport)
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_BUILDING_DLL
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC \
  OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_EXPORT
#else
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC \
  OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_IMPORT
#endif
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC_TYPE \
  OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_LOCAL
#else
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_EXPORT __attribute__(( \
      visibility("default")))
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC __attribute__(( \
      visibility("default")))
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_LOCAL __attribute__(( \
      visibility("hidden")))
#else
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_LOCAL
#endif
#define OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <openscenario_msgs/msg/entity_status_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simulation_api/color_utils/color_utils.hpp>

#include <unordered_map>
#include <string>

namespace openscenario_visualization
{
/**
 * @brief ROS2 component for visualizing simulation result.
 */
class OpenscenarioVisualizationComponent : public rclcpp::Node
{
public:
  OPENSCENARIO_VISUALIZATION_OPENSCENARIO_VISUALIZATION_COMPONENT_PUBLIC
  explicit OpenscenarioVisualizationComponent(const rclcpp::NodeOptions &);

private:
  void entityStatusCallback(const openscenario_msgs::msg::EntityStatusArray::SharedPtr msg);
  const visualization_msgs::msg::MarkerArray generateDeleteMarker(std::string ns);
  const visualization_msgs::msg::MarkerArray generateDeleteMarker() const;
  const visualization_msgs::msg::MarkerArray generateMarker(
    const openscenario_msgs::msg::EntityStatus & status);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<openscenario_msgs::msg::EntityStatusArray>::SharedPtr entity_status_sub_;
  std::unordered_map<std::string, visualization_msgs::msg::MarkerArray> markers_;
};
}  // namespace openscenario_visualization

#endif  // OPENSCENARIO_VISUALIZATION__OPENSCENARIO_VISUALIZATION_COMPONENT_HPP_
