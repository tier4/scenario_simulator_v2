#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(GridTraversal, begin)
{
  double start_x = 2.2;
  double start_y = 3.3;
  double end_x = 5.5;
  double end_y = 7.7;
  auto traversal = simple_sensor_simulator::GridTraversal(start_x, start_y, end_x, end_y);

  auto it = traversal.begin();

  EXPECT_TRUE((*it).first == static_cast<int32_t>(start_x));
  EXPECT_TRUE((*it).second == static_cast<int32_t>(start_y));
}

TEST(GridTraversal, operatorIncrement)
{
  double start_x = 2.2;
  double start_y = 3.3;
  double end_x = 5.5;
  double end_y = 7.7;
  auto traversal = simple_sensor_simulator::GridTraversal(start_x, start_y, end_x, end_y);

  auto it = traversal.begin();

  ++it;
  EXPECT_TRUE((*it).first == static_cast<int32_t>(start_x));
  EXPECT_TRUE((*it).second == static_cast<int32_t>(start_y) + 1);
  ++it;
  EXPECT_TRUE((*it).first == static_cast<int32_t>(start_x) + 1);
  EXPECT_TRUE((*it).second == static_cast<int32_t>(start_y) + 1);
}

TEST(GridTraversal, operatorNotEqual)
{
  double start_x = 0.0;
  double start_y = 0.0;
  double end_x = 10.5;
  double end_y = 0.0;
  auto traversal = simple_sensor_simulator::GridTraversal(start_x, start_y, end_x, end_y);

  auto it = traversal.begin();

  int curr = 0;
  for (int i = 0; i <= end_x; i++) {
    EXPECT_TRUE(it != traversal.end());
    EXPECT_TRUE(curr == (*it).first);
    EXPECT_TRUE(0 == (*it).second);
    ++it;
    curr++;
  }

  EXPECT_FALSE(it != traversal.end());
  EXPECT_TRUE(std::ceil(end_x) == (*it).first);
  EXPECT_TRUE(std::ceil(end_x) == curr);
}
