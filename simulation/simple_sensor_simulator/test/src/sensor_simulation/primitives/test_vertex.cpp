#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Vertex, toVertex_onePoint)
{
  geometry_msgs::msg::Point point{};
  point.x = -11.0;
  point.y = -13.0;
  point.z = -17.0;

  simple_sensor_simulator::Vertex v = simple_sensor_simulator::toVertex(point);

  EXPECT_EQ(point.x, v.x);
  EXPECT_EQ(point.y, v.y);
  EXPECT_EQ(point.z, v.z);
}

TEST(Vertex, toVertex_manyPoints)
{
  std::vector<geometry_msgs::msg::Point> points{};
  for (int i = 0; i < 10; i++) {
    geometry_msgs::msg::Point point{};
    point.x = -11.0 * i;
    point.y = -13.0 * i;
    point.z = -17.0 * i;
    points.push_back(point);
  }

  std::vector<simple_sensor_simulator::Vertex> vertices = simple_sensor_simulator::toVertex(points);

  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(points[i].x, vertices[i].x);
    EXPECT_EQ(points[i].y, vertices[i].y);
    EXPECT_EQ(points[i].z, vertices[i].z);
  }
}

TEST(Vertex, toVertex_empty)
{
  std::vector<geometry_msgs::msg::Point> points{};

  std::vector<simple_sensor_simulator::Vertex> vertices = simple_sensor_simulator::toVertex(points);

  EXPECT_TRUE(0 == vertices.size());
}

TEST(Vertex, toPoint_oneVertex)
{
  simple_sensor_simulator::Vertex v{};
  v.x = -11.0;
  v.y = -13.0;
  v.z = -17.0;

  geometry_msgs::msg::Point point = simple_sensor_simulator::toPoint(v);

  EXPECT_EQ(point.x, v.x);
  EXPECT_EQ(point.y, v.y);
  EXPECT_EQ(point.z, v.z);
}

TEST(Vertex, toPoints_manyVertices)
{
  std::vector<simple_sensor_simulator::Vertex> vertices{};
  for (int i = 0; i < 10; i++) {
    simple_sensor_simulator::Vertex v{};
    v.x = -11.0 * i;
    v.y = -13.0 * i;
    v.z = -17.0 * i;
    vertices.push_back(v);
  }

  std::vector<geometry_msgs::msg::Point> points = simple_sensor_simulator::toPoints(vertices);

  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(points[i].x, vertices[i].x);
    EXPECT_EQ(points[i].y, vertices[i].y);
    EXPECT_EQ(points[i].z, vertices[i].z);
  }
}

TEST(Vertex, toPoints_empty)
{
  std::vector<simple_sensor_simulator::Vertex> vertices{};

  std::vector<geometry_msgs::msg::Point> points = simple_sensor_simulator::toPoints(vertices);

  EXPECT_EQ(points.size(), 0);
}
