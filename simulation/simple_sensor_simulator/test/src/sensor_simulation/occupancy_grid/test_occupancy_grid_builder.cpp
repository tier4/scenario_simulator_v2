#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_builder.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>

auto getBox() -> simple_sensor_simulator::primitives::Box
{
  const std::string type("box");
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
  simple_sensor_simulator::primitives::Box prim(depth, width, height, pose);
  return prim;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(OccupancyGridBuilder, add_overLimit)
{
  const double resolution = 1000.0f;
  const size_t height = 11;
  const size_t width = 13;
  const int8_t occupied_cost = 17;
  const int8_t invisible_cost = 19;
  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    resolution, height, width, occupied_cost, invisible_cost);
  auto count_max = std::numeric_limits<int16_t>::max();
  for (int i = 0; i < count_max; i++) {
    EXPECT_NO_THROW(builder.add(getBox()));
  }
  EXPECT_THROW(builder.add(getBox()), std::runtime_error);
}

TEST(OccupancyGridBuilder, reset)
{
  const double resolution = 1000.0f;
  const size_t height = 11;
  const size_t width = 13;
  const int8_t occupied_cost = 17;
  const int8_t invisible_cost = 19;
  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    resolution, height, width, occupied_cost, invisible_cost);
  auto count_max = std::numeric_limits<int16_t>::max();
  for (int i = 0; i < count_max; i++) {
    EXPECT_NO_THROW(builder.add(getBox()));
  }
  builder.reset(geometry_msgs::msg::Pose{});
  for (int i = 0; i < count_max; i++) {
    EXPECT_NO_THROW(builder.add(getBox()));
  }
  EXPECT_THROW(builder.add(getBox()), std::runtime_error);
}

TEST(OccupancyGridBuilder, build_empty)
{
  const double resolution = 1000.0f;
  const size_t height = 11;
  const size_t width = 13;
  const int8_t occupied_cost = 17;
  const int8_t invisible_cost = 19;
  auto builder = simple_sensor_simulator::OccupancyGridBuilder(
    resolution, height, width, occupied_cost, invisible_cost);

  builder.build();
  auto grid = builder.get();

  EXPECT_TRUE(std::all_of(grid.begin(), grid.end(), [&](int elem) { return 0 == elem; }));
}
