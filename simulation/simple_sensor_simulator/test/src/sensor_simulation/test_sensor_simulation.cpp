#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

TEST(SensorSimulation, attachLidarSensor_wrongArchitecture)
{
  const double current_simulation_time = 3.0;
  const simulation_api_schema::LidarConfiguration configuration{};

  rclcpp::NodeOptions options;
  rclcpp::Node node{"name", options};

  auto sim = simple_sensor_simulator::SensorSimulation();

  EXPECT_THROW(
    sim.attachLidarSensor(current_simulation_time, configuration, node), std::runtime_error);
}

TEST(SensorSimulation, attachDetectionSensor_wrongArchitecture)
{
  const double current_simulation_time = 3.0;
  const simulation_api_schema::DetectionSensorConfiguration configuration{};

  rclcpp::NodeOptions options;
  rclcpp::Node node{"name", options};

  auto sim = simple_sensor_simulator::SensorSimulation();

  EXPECT_THROW(
    sim.attachDetectionSensor(current_simulation_time, configuration, node), std::runtime_error);
}

TEST(SensorSimulation, attachOccupancyGridSensor_wrongArchitecture)
{
  const double current_simulation_time = 3.0;
  const simulation_api_schema::OccupancyGridSensorConfiguration configuration{};

  rclcpp::NodeOptions options;
  rclcpp::Node node{"name", options};

  auto sim = simple_sensor_simulator::SensorSimulation();

  EXPECT_THROW(
    sim.attachOccupancyGridSensor(current_simulation_time, configuration, node),
    std::runtime_error);
}
