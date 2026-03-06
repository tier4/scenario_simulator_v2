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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_

#include <geometry/spline/catmull_rom_subspline.hpp>
#include <optional>
#include <string>
#include <traffic_simulator/behavior/follow_trajectory.hpp>
#include <traffic_simulator/data_type/behavior.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>

namespace entity_behavior
{
using EntityStatusDict =
  std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>;

using EuclideanDistancesMap = std::unordered_map<std::pair<std::string, std::string>, double>;

class BehaviorPluginBase
{
public:
  virtual ~BehaviorPluginBase() = default;
  virtual auto configure(const rclcpp::Logger & logger) -> void = 0;
  virtual auto update(const double current_time, const double step_time) -> void = 0;
  virtual auto getCurrentAction() const -> const std::string & = 0;

  // clang-format off
#define DEFINE_GETTER_SETTER(NAME, KEY, TYPE)                \
  virtual auto get##NAME() -> TYPE = 0;                      \
  virtual auto set##NAME(const TYPE & value) -> void = 0;    \
  /*   */ auto get##NAME##Key() const -> const std::string & \
  {                                                          \
    static const std::string key = KEY;                      \
    return key;                                              \
  }

  DEFINE_GETTER_SETTER(BehaviorParameter,                                "behavior_parameter",                             traffic_simulator_msgs::msg::BehaviorParameter)
  DEFINE_GETTER_SETTER(CanonicalizedEntityStatus,                        "canonicalized_entity_status",                    std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>)
  DEFINE_GETTER_SETTER(CurrentTime,                                      "current_time",                                   double)
  DEFINE_GETTER_SETTER(DebugMarker,                                      "debug_marker",                                   std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DefaultMatchingDistanceForLaneletPoseCalculation, "matching_distance_for_lanelet_pose_calculation", double)
  DEFINE_GETTER_SETTER(EuclideanDistancesMap,                            "euclidean_distances_map",                        std::shared_ptr<EuclideanDistancesMap>)
  DEFINE_GETTER_SETTER(GoalPoses,                                        "goal_poses",                                     std::vector<geometry_msgs::msg::Pose>)
  DEFINE_GETTER_SETTER(LaneChangeParameters,                             "lane_change_parameters",                         traffic_simulator::lane_change::Parameter)
  DEFINE_GETTER_SETTER(LateralCollisionThreshold,                        "lateral_collision_threshold",                    std::optional<double>)
  DEFINE_GETTER_SETTER(Obstacle,                                         "obstacle",                                       std::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus,                                "other_entity_status",                            EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters,                             "pedestrian_parameters",                          traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(PolylineTrajectory,                               "polyline_trajectory",                            std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>)
  DEFINE_GETTER_SETTER(ReferenceTrajectory,                              "reference_trajectory",                           std::shared_ptr<math::geometry::CatmullRomSpline>)
  DEFINE_GETTER_SETTER(Request,                                          "request",                                        traffic_simulator::behavior::Request)
  DEFINE_GETTER_SETTER(RouteLanelets,                                    "route_lanelets",                                 lanelet::Ids)
  DEFINE_GETTER_SETTER(StepTime,                                         "step_time",                                      double)
  DEFINE_GETTER_SETTER(TargetSpeed,                                      "target_speed",                                   std::optional<double>)
  DEFINE_GETTER_SETTER(TrafficLights,                                    "traffic_lights",                                 std::shared_ptr<traffic_simulator::TrafficLightsBase>)
  DEFINE_GETTER_SETTER(VehicleParameters,                                "vehicle_parameters",                             traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints,                                        "waypoints",                                      traffic_simulator_msgs::msg::WaypointsArray)
  // clang-format on
#undef DEFINE_GETTER_SETTER
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
