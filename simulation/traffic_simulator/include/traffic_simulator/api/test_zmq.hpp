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

#ifndef TEST_ZMQ__API__API_HPP_
#define TEST_ZMQ__API__API_HPP_

#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

#include <chrono>
#include <fstream>

namespace traffic_simulator
{

class TestZMQ : public rclcpp::Node
{
public:
  TestZMQ()
  : Node("test_zmq_client"),
    zeromq_client_(simulation_interface::protocol, "localhost", getZMQSocketPort())
  {
  }

  int getZMQSocketPort()
  {
    if (!has_parameter("port")) declare_parameter("port", 5555);
    return get_parameter("port").as_int();
  }

  void closeZMQConnection() { zeromq_client_.closeConnection(); }

  void tick()
  {
    number_of_ticks++;
    auto start = std::chrono::high_resolution_clock::now();
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "\n=========== = TICK = ===========");
    if (!initialize()) RCLCPP_ERROR_STREAM(get_logger(), "Errow with initialize");
    for (auto & v : entities) {
      if (!spawn(v)) RCLCPP_ERROR_STREAM(get_logger(), "Error with spawn");
      if (!attachDetectionSensor(v)) RCLCPP_ERROR_STREAM(get_logger(), "Errow with detection");
      if (!attachOccupancyGridSensor(v)) RCLCPP_ERROR_STREAM(get_logger(), "Errow with occupancy");
      if (!attachLidarSensor(v)) RCLCPP_ERROR_STREAM(get_logger(), "Errow with lidar");
    }
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "Try to updateFrame...");
    for (unsigned i = 0; i < 100; ++i) {
      if (!updateFrame()) RCLCPP_ERROR_STREAM(get_logger(), "Errow with updateFrame");
      if (!updateEntityStatusInSim()) RCLCPP_ERROR_STREAM(get_logger(), "Errow with updateEntity");
      if (!updateTrafficLightsInSim()) RCLCPP_ERROR_STREAM(get_logger(), "Errow with updateLights");
      if (!updateSensorFrame()) RCLCPP_ERROR_STREAM(get_logger(), "Errow with updateSensorFrame");
    }
    for (auto & v : entities)
      if (!despawn(v)) RCLCPP_ERROR_STREAM(get_logger(), "Errow with despawn");
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    execution_time_us += duration.count();
    writeToFile(std::to_string(duration.count()));
    RCLCPP_WARN_STREAM(get_logger(), "Tick microseconds: " << duration.count());
  }

  bool spawn(const std::string & name)
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), name + ": Try to spawn...");
    traffic_simulator_msgs::msg::VehicleParameters parameters;
    simulation_api_schema::SpawnVehicleEntityRequest req;
    simulation_api_schema::SpawnVehicleEntityResponse res;
    simulation_interface::toProto(parameters, *req.mutable_parameters());
    req.mutable_parameters()->set_name(name);
    req.set_is_ego(false);
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool despawn(const std::string & name)
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), name + ": Try to despawn...");
    simulation_api_schema::DespawnEntityRequest req;
    simulation_api_schema::DespawnEntityResponse res;
    req.set_name(name);
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool initialize()
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "Try to initialize...");
    clock_.initialize(0, 0.05);
    double realtime_factor = 1;
    double step_time = 0.05;
    simulation_api_schema::InitializeRequest req;
    req.set_step_time(step_time);
    req.set_realtime_factor(realtime_factor);
    simulation_api_schema::InitializeResponse res;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool attachDetectionSensor(const std::string & name)
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), name + ": Try to attachDetectionSensor...");
    auto configuration = helper::constructDetectionSensorConfiguration(name, "awf/universe", 0.1);
    simulation_api_schema::AttachDetectionSensorRequest req;
    simulation_api_schema::AttachDetectionSensorResponse res;
    *req.mutable_configuration() = configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool attachOccupancyGridSensor(const std::string & name)
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), name + ": Try to attachOccupancyGridSensor...");
    simulation_api_schema::OccupancyGridSensorConfiguration configuration;
    configuration.set_architecture_type("awf/universe");
    configuration.set_entity(name);
    configuration.set_filter_by_range(true);
    configuration.set_height(200);
    configuration.set_range(300);
    configuration.set_resolution(0.5);
    configuration.set_update_duration(0.1);
    configuration.set_width(200);
    simulation_api_schema::AttachOccupancyGridSensorRequest req;
    simulation_api_schema::AttachOccupancyGridSensorResponse res;
    *req.mutable_configuration() = configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool attachLidarSensor(const std::string & name)
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), name + ": Try to attachLidarSensor...");
    auto configuration =
      helper::constructLidarConfiguration(helper::LidarType::VLP16, name, "awf/universe");

    simulation_api_schema::AttachLidarSensorRequest req;
    simulation_api_schema::AttachLidarSensorResponse res;
    *req.mutable_configuration() = configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool updateSensorFrame()
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "Try to updateSensorFrame...");
    simulation_api_schema::UpdateSensorFrameRequest req;
    req.set_current_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTimeAsMsg().clock, *req.mutable_current_ros_time());
    simulation_api_schema::UpdateSensorFrameResponse res;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool updateTrafficLightsInSim()
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "Try to updateTrafficLightsInSim...");
    simulation_api_schema::UpdateTrafficLightsRequest req;
    simulation_api_schema::UpdateTrafficLightsResponse res;
    for (unsigned i = 0; i < 100; i++) {
      auto traffic_light = getTrafficSignal();
      simulation_api_schema::TrafficLightState state;
      simulation_interface::toProto(
        static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light), state);
      *req.add_states() = state;
    }
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool updateEntityStatusInSim()
  {
    if (verbose) RCLCPP_INFO_STREAM(get_logger(), "Try to updateEntityStatusInSim...");
    simulation_api_schema::UpdateEntityStatusRequest req;
    req.set_ego_entity_status_before_update_is_empty(false);
    // ego v
    std::tuple<
      autoware_auto_control_msgs::msg::AckermannControlCommand,
      autoware_auto_vehicle_msgs::msg::GearCommand>
      v_cmd;
    simulation_interface::toProto(v_cmd, *req.mutable_vehicle_command());
    // ego state
    traffic_simulator_msgs::msg::EntityStatus ego_bef_update;
    simulation_interface::toProto(ego_bef_update, *req.mutable_ego_entity_status_before_update());

    // entities
    for (const auto & name : entities) {
      traffic_simulator_msgs::msg::EntityStatus status;
      status.name = name;
      traffic_simulator_msgs::EntityStatus proto;
      simulation_interface::toProto(status, proto);
      *req.add_status() = proto;
    }
    // response
    simulation_api_schema::UpdateEntityStatusResponse res;
    zeromq_client_.call(req, res);
    return res.result().success();
  }

  bool updateFrame()
  {
    simulation_api_schema::UpdateFrameRequest req;
    req.set_current_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTimeAsMsg().clock, *req.mutable_current_ros_time());
    simulation_api_schema::UpdateFrameResponse res;
    zeromq_client_.call(req, res);
    clock_.update();
    return res.result().success();
  }

  auto getTrafficSignal() -> autoware_auto_perception_msgs::msg::TrafficSignal
  {
    auto color = [](auto value) {
      autoware_auto_perception_msgs::msg::TrafficLight traffic_light;
      traffic_light.color = value;

      return traffic_light;
    };

    auto shape = [](auto value) {
      autoware_auto_perception_msgs::msg::TrafficLight traffic_light;
      traffic_light.shape = value;
      return traffic_light;
    };

    auto status = [](auto value) {
      autoware_auto_perception_msgs::msg::TrafficLight traffic_light;
      traffic_light.status = value;
      return traffic_light;
    };

    autoware_auto_perception_msgs::msg::TrafficSignal message;
    message.map_primitive_id = 123;
    message.lights = {
      status(autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN),
      color(autoware_auto_perception_msgs::msg::TrafficLight::RED),
      color(autoware_auto_perception_msgs::msg::TrafficLight::GREEN),
      color(autoware_auto_perception_msgs::msg::TrafficLight::AMBER),
      shape(autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW),
      shape(autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW),
      shape(autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW),
      shape(autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW),
    };
    return message;
  }

  unsigned number_of_ticks{0};
  double execution_time_us{0.0};

  void writeToFile(std::string text)
  {
    std::ofstream f;
    f.open("test_zmq.txt", std::ios::out | std::ios::app);
    f << text << "\n";
    f.close();
  }

private:
  zeromq::MultiClient zeromq_client_;
  traffic_simulator::SimulationClock clock_;
  std::string ego{"ego"};
  std::vector<std::string> entities{"v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9"};
  bool verbose{false};
};
}  // namespace traffic_simulator
#endif  // TEST_ZMQ__API__API_HPP_
