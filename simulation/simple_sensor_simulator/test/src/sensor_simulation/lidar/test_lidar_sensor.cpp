#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(LidarSensor, update_noEgo)
{
  const double current_simulation_time{};
  const simulation_api_schema::LidarConfiguration configuration{};
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ptr{};

  auto sensor = simple_sensor_simulator::LidarSensor<sensor_msgs::msg::PointCloud2>(
    current_simulation_time, configuration, publisher_ptr);
  std::vector<traffic_simulator_msgs::EntityStatus> status{};
  const rclcpp::Time current_ros_time{};

  EXPECT_THROW(
    sensor.update(current_simulation_time + 1, status, current_ros_time), std::runtime_error);
}