#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Box, Box)
{
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
  simple_sensor_simulator::primitives::Box prim(depth, width, height, pose);

  const std::vector<geometry_msgs::msg::Point> hull = prim.get2DConvexHull();
  auto p_min_x = std::min_element(
    hull.begin(), hull.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return a.x < b.x;
    });
  auto p_min_y = std::min_element(
    hull.begin(), hull.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return a.y < b.y;
    });
  auto p_max_x = std::min_element(
    hull.begin(), hull.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return a.x > b.x;
    });
  auto p_max_y = std::min_element(
    hull.begin(), hull.end(),
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
      return a.y > b.y;
    });

  EXPECT_FALSE(p_min_x == hull.end());
  EXPECT_FALSE(p_min_y == hull.end());
  EXPECT_FALSE(p_max_x == hull.end());
  EXPECT_FALSE(p_max_y == hull.end());

  EXPECT_TRUE(p_min_x->x == pose.position.x - depth / 2);
  EXPECT_TRUE(p_min_y->y == pose.position.y - width / 2);
  EXPECT_TRUE(p_max_x->x == pose.position.x + depth / 2);
  EXPECT_TRUE(p_max_y->y == pose.position.y + width / 2);
}