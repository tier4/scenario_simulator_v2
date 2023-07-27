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

#ifndef SIMPLE_SENSOR_SIMULATOR__SIMPLE_SENSOR_SIMULATOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SIMPLE_SENSOR_SIMULATOR_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp>
#include <simple_sensor_simulator/vehicle_simulation/ego_entity_simulation.hpp>
#include <simulation_interface/zmq_multi_server.hpp>
#include <string>
#include <thread>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_EXPORT __attribute__((dllexport))
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_EXPORT __declspec(dllexport)
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_BUILDING_DLL
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC \
  SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_EXPORT
#else
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC \
  SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_IMPORT
#endif
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC_TYPE \
  SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_LOCAL
#else
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_LOCAL
#endif
#define SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

namespace simple_sensor_simulator
{
class ScenarioSimulator : public rclcpp::Node
{
public:
  SIMPLE_SENSOR_SIMULATOR_SIMPLE_SENSOR_SIMULATOR_COMPONENT_PUBLIC
  explicit ScenarioSimulator(const rclcpp::NodeOptions & options);
  ~ScenarioSimulator();

private:
  SensorSimulation sensor_sim_;
  void initialize(
    const simulation_api_schema::InitializeRequest & req,
    simulation_api_schema::InitializeResponse & res);
  void updateFrame(
    const simulation_api_schema::UpdateFrameRequest & req,
    simulation_api_schema::UpdateFrameResponse & res);
  void updateEntityStatus(
    const simulation_api_schema::UpdateEntityStatusRequest & req,
    simulation_api_schema::UpdateEntityStatusResponse & res);
  void spawnVehicleEntity(
    const simulation_api_schema::SpawnVehicleEntityRequest & req,
    simulation_api_schema::SpawnVehicleEntityResponse & res);
  void spawnPedestrianEntity(
    const simulation_api_schema::SpawnPedestrianEntityRequest & req,
    simulation_api_schema::SpawnPedestrianEntityResponse & res);
  void spawnMiscObjectEntity(
    const simulation_api_schema::SpawnMiscObjectEntityRequest &,
    simulation_api_schema::SpawnMiscObjectEntityResponse &);
  void despawnEntity(
    const simulation_api_schema::DespawnEntityRequest & req,
    simulation_api_schema::DespawnEntityResponse & res);
  void attachDetectionSensor(
    const simulation_api_schema::AttachDetectionSensorRequest & req,
    simulation_api_schema::AttachDetectionSensorResponse & res);
  void attachLidarSensor(
    const simulation_api_schema::AttachLidarSensorRequest & req,
    simulation_api_schema::AttachLidarSensorResponse & res);
  void attachOccupancyGridSensor(
    const simulation_api_schema::AttachOccupancyGridSensorRequest & req,
    simulation_api_schema::AttachOccupancyGridSensorResponse & res);
  void updateTrafficLights(
    const simulation_api_schema::UpdateTrafficLightsRequest & req,
    simulation_api_schema::UpdateTrafficLightsResponse & res);
  int getSocketPort();
  std::vector<traffic_simulator_msgs::VehicleParameters> ego_vehicles_;
  std::vector<traffic_simulator_msgs::VehicleParameters> vehicles_;
  std::vector<traffic_simulator_msgs::PedestrianParameters> pedestrians_;
  std::vector<traffic_simulator_msgs::MiscObjectParameters> misc_objects_;
  double realtime_factor_;
  double step_time_;
  double current_time_;
  rclcpp::Time current_ros_time_;
  bool initialized_;
  std::map<std::string, simulation_api_schema::EntityStatus> entity_status_;
  std::vector<autoware_auto_perception_msgs::msg::TrafficSignal> traffic_signals_states_;
  traffic_simulator_msgs::BoundingBox getBoundingBox(const std::string & name);
  zeromq::MultiServer server_;
  geographic_msgs::msg::GeoPoint getOrigin();
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  std::shared_ptr<vehicle_simulation::EgoEntitySimulation> ego_entity_simulation_;

  bool isEgo(const std::string & name);
  bool isEntityExists(const std::string & name);
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SIMPLE_SENSOR_SIMULATOR_HPP_
