#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Raycaster, addPrimitive_box)
{
  auto rc = simple_sensor_simulator::Raycaster();

  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 9.0;
  pose.position.y = 11.0;
  pose.position.z = 17.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  const float depth = 19.0;
  const float width = 23.0;
  const float height = 29.0;
  simple_sensor_simulator::primitives::Box box(depth, width, height, pose);

  EXPECT_NO_THROW(
    rc.addPrimitive<simple_sensor_simulator::primitives::Box>(std::string("box"), box));
}

TEST(Raycaster, addPrimitive_twoIdenticalNames)
{
  auto rc = simple_sensor_simulator::Raycaster();

  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 9.0;
  pose.position.y = 11.0;
  pose.position.z = 17.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  const float depth = 19.0;
  const float width = 23.0;
  const float height = 29.0;
  simple_sensor_simulator::primitives::Box box_0(depth, width, height, pose);
  simple_sensor_simulator::primitives::Box box_1(depth, width, height, pose);

  EXPECT_NO_THROW(
    rc.addPrimitive<simple_sensor_simulator::primitives::Box>(std::string("box"), box_0));
  EXPECT_THROW(
    rc.addPrimitive<simple_sensor_simulator::primitives::Box>(std::string("box"), box_1),
    std::runtime_error);
}