#include <gtest/gtest.h>

#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>

class JaggedSquare : public simple_sensor_simulator::primitives::Primitive
{
public:
  explicit JaggedSquare(
    float depth, float width, float height, const geometry_msgs::msg::Pose & pose)
  : simple_sensor_simulator::primitives::Primitive("Box", pose),
    depth(depth),
    width(width),
    height(height)
  {
    vertices_ = std::vector<simple_sensor_simulator::Vertex>(8);

    vertices_[0].x = -0.5 * depth;
    vertices_[0].y = -0.5 * width;
    vertices_[0].z = 0.0 * height;

    vertices_[1].x = 0.0 * depth;
    vertices_[1].y = -0.5 * width;
    vertices_[1].z = 0.0 * height;

    vertices_[2].x = 0.5 * depth;
    vertices_[2].y = -0.5 * width;
    vertices_[2].z = 0.0 * height;

    vertices_[3].x = -0.5 * depth;
    vertices_[3].y = 0.0 * width;
    vertices_[3].z = 0.0 * height;

    vertices_[4].x = 0.0 * depth;
    vertices_[4].y = 0.0 * width;
    vertices_[4].z = 0.0 * height;

    vertices_[5].x = 0.5 * depth;
    vertices_[5].y = 0.0 * width;
    vertices_[5].z = 0.0 * height;

    vertices_[6].x = -0.5 * depth;
    vertices_[6].y = 0.5 * width;
    vertices_[6].z = 0.0 * height;

    vertices_[7].x = 0.0 * depth;
    vertices_[7].y = 0.5 * width;
    vertices_[7].z = 0.0 * height;

    triangles_ = std::vector<simple_sensor_simulator::Triangle>(6);

    triangles_[0].v0 = 0;
    triangles_[0].v1 = 1;
    triangles_[0].v2 = 4;

    triangles_[1].v0 = 1;
    triangles_[1].v1 = 4;
    triangles_[1].v2 = 3;

    triangles_[2].v0 = 1;
    triangles_[2].v1 = 2;
    triangles_[2].v2 = 5;

    triangles_[3].v0 = 1;
    triangles_[3].v1 = 5;
    triangles_[3].v2 = 4;

    triangles_[4].v0 = 3;
    triangles_[4].v1 = 4;
    triangles_[4].v2 = 7;

    triangles_[5].v0 = 3;
    triangles_[5].v1 = 7;
    triangles_[5].v2 = 6;
  }
  std::vector<simple_sensor_simulator::Vertex> getVertices(geometry_msgs::msg::Pose pose)
  {
    return transform(pose);
  }
  ~JaggedSquare() = default;
  const float depth;
  const float width;
  const float height;
};

bool is2DSubset(
  const std::vector<simple_sensor_simulator::Vertex> & superset,
  const std::vector<simple_sensor_simulator::Vertex> & subset)
{
  for (const simple_sensor_simulator::Vertex & v_subset : subset) {
    auto it = std::find_if(
      superset.begin(), superset.end(), [&](const simple_sensor_simulator::Vertex & v_superset) {
        return v_superset.x == v_subset.x && v_superset.y == v_subset.y;
      });
    if (it == superset.end()) {
      return false;
    }
  }
  return true;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Primitive, get2DConvexHull_normal)
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
  JaggedSquare prim(depth, width, height, pose);

  std::vector<simple_sensor_simulator::Vertex> hull =
    simple_sensor_simulator::toVertex(prim.get2DConvexHull());
  std::vector<simple_sensor_simulator::Vertex> hull_superset = prim.getVertex();
  hull_superset.erase(hull_superset.begin() + 4);

  std::vector<simple_sensor_simulator::Vertex> hull_subset = prim.getVertex();
  hull_subset.erase(hull_subset.begin() + 4);
  hull_subset.erase(hull_subset.begin() + 3);
  hull_subset.erase(hull_subset.begin() + 1);

  EXPECT_TRUE(hull.size() >= 5);
  EXPECT_TRUE(hull_superset.size() == 7);
  EXPECT_TRUE(hull_subset.size() == 5);
  EXPECT_TRUE(is2DSubset(hull_superset, hull));
  EXPECT_TRUE(is2DSubset(hull, hull_subset));
}

TEST(Primitive, get2DConvexHull_withTransform)
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
  JaggedSquare prim(depth, width, height, pose);

  geometry_msgs::msg::Pose sensor_pose{};
  sensor_pose.position.x = 11.0;
  sensor_pose.position.y = 13.0;
  sensor_pose.position.z = 17.0;
  sensor_pose.orientation.x = 0.944;
  sensor_pose.orientation.y = 0.187;
  sensor_pose.orientation.z = 0.187;
  sensor_pose.orientation.w = 0.199;

  std::vector<simple_sensor_simulator::Vertex> hull =
    simple_sensor_simulator::toVertex(prim.get2DConvexHull(sensor_pose));
  std::vector<simple_sensor_simulator::Vertex> hull_superset = prim.getVertices(sensor_pose);
  std::vector<simple_sensor_simulator::Vertex> hull_subset = prim.getVertices(sensor_pose);

  hull_superset.erase(hull_superset.begin() + 4);

  hull_subset.erase(hull_subset.begin() + 4);
  hull_subset.erase(hull_subset.begin() + 3);
  hull_subset.erase(hull_subset.begin() + 1);

  EXPECT_TRUE(hull.size() >= 5);
  EXPECT_TRUE(hull_superset.size() == 7);
  EXPECT_TRUE(hull_subset.size() == 5);
  EXPECT_TRUE(is2DSubset(hull_superset, hull));
  EXPECT_TRUE(is2DSubset(hull, hull_subset));
}

TEST(Primitive, getMin_noTransform)
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

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> min_x = prim.getMin(axis_x);
  const std::optional<double> min_y = prim.getMin(axis_y);
  const std::optional<double> min_z = prim.getMin(axis_z);

  EXPECT_TRUE(min_x.has_value());
  EXPECT_TRUE(min_y.has_value());
  EXPECT_TRUE(min_z.has_value());
  EXPECT_TRUE(min_x.value() == pose.position.x - depth / 2);
  EXPECT_TRUE(min_y.value() == pose.position.y - width / 2);
  EXPECT_TRUE(min_z.value() == pose.position.z - height / 2);
}

TEST(Primitive, getMin_emptyPrimitive)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 11.0;
  pose.position.y = 13.0;
  pose.position.z = 17.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  simple_sensor_simulator::primitives::Primitive prim(type, pose);

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> min_x = prim.getMin(axis_x);
  const std::optional<double> min_y = prim.getMin(axis_y);
  const std::optional<double> min_z = prim.getMin(axis_z);

  EXPECT_FALSE(min_x.has_value());
  EXPECT_FALSE(min_y.has_value());
  EXPECT_FALSE(min_z.has_value());
}

TEST(Primitive, getMin_emptyPrimitiveWithPose)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  simple_sensor_simulator::primitives::Primitive prim(type, pose);
  geometry_msgs::msg::Pose sensor_pose{};
  sensor_pose.position.x = 11.0;
  sensor_pose.position.y = 13.0;
  sensor_pose.position.z = 17.0;
  sensor_pose.orientation.x = -0.5;
  sensor_pose.orientation.y = 0.5;
  sensor_pose.orientation.z = 0.5;
  sensor_pose.orientation.w = 0.5;

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> min_x = prim.getMin(axis_x, sensor_pose);
  const std::optional<double> min_y = prim.getMin(axis_y, sensor_pose);
  const std::optional<double> min_z = prim.getMin(axis_z, sensor_pose);

  EXPECT_FALSE(min_x.has_value());
  EXPECT_FALSE(min_y.has_value());
  EXPECT_FALSE(min_z.has_value());
}

TEST(Primitive, getMin_withTransform)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 5.0;
  pose.position.z = 7.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  const float depth = 19.0;
  const float width = 23.0;
  const float height = 29.0;
  simple_sensor_simulator::primitives::Box prim(depth, width, height, pose);

  geometry_msgs::msg::Pose sensor_pose{};
  sensor_pose.position.x = -11.0;
  sensor_pose.position.y = -13.0;
  sensor_pose.position.z = -17.0;
  sensor_pose.orientation.x = -0.5;
  sensor_pose.orientation.y = 0.5;
  sensor_pose.orientation.z = 0.5;
  sensor_pose.orientation.w = 0.5;

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> min_x = prim.getMin(axis_x, sensor_pose);
  const std::optional<double> min_y = prim.getMin(axis_y, sensor_pose);
  const std::optional<double> min_z = prim.getMin(axis_z, sensor_pose);

  EXPECT_TRUE(min_x.has_value());
  EXPECT_TRUE(min_y.has_value());
  EXPECT_TRUE(min_z.has_value());

  EXPECT_TRUE(min_x.value() == pose.position.x - sensor_pose.position.x - height / 2);
  EXPECT_TRUE(min_y.value() == pose.position.y - sensor_pose.position.y - depth / 2);
  EXPECT_TRUE(min_z.value() == pose.position.z - sensor_pose.position.z - width / 2);
}

TEST(Primitive, getMax_noTransform)
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

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> max_x = prim.getMax(axis_x);
  const std::optional<double> max_y = prim.getMax(axis_y);
  const std::optional<double> max_z = prim.getMax(axis_z);

  EXPECT_TRUE(max_x.has_value());
  EXPECT_TRUE(max_y.has_value());
  EXPECT_TRUE(max_z.has_value());
  EXPECT_TRUE(max_x.value() == pose.position.x + depth / 2);
  EXPECT_TRUE(max_y.value() == pose.position.y + width / 2);
  EXPECT_TRUE(max_z.value() == pose.position.z + height / 2);
}

TEST(Primitive, getMax_emptyPrimitive)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 11.0;
  pose.position.y = 13.0;
  pose.position.z = 17.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  simple_sensor_simulator::primitives::Primitive prim(type, pose);

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> max_x = prim.getMax(axis_x);
  const std::optional<double> max_y = prim.getMax(axis_y);
  const std::optional<double> max_z = prim.getMax(axis_z);

  EXPECT_FALSE(max_x.has_value());
  EXPECT_FALSE(max_y.has_value());
  EXPECT_FALSE(max_z.has_value());
}

TEST(Primitive, getMax_emptyPrimitiveWithPose)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  simple_sensor_simulator::primitives::Primitive prim(type, pose);
  geometry_msgs::msg::Pose sensor_pose{};
  sensor_pose.position.x = 11.0;
  sensor_pose.position.y = 13.0;
  sensor_pose.position.z = 17.0;
  sensor_pose.orientation.x = -0.5;
  sensor_pose.orientation.y = 0.5;
  sensor_pose.orientation.z = 0.5;
  sensor_pose.orientation.w = 0.5;

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> max_x = prim.getMax(axis_x, sensor_pose);
  const std::optional<double> max_y = prim.getMax(axis_y, sensor_pose);
  const std::optional<double> max_z = prim.getMax(axis_z, sensor_pose);

  EXPECT_FALSE(max_x.has_value());
  EXPECT_FALSE(max_y.has_value());
  EXPECT_FALSE(max_z.has_value());
}

TEST(Primitive, getMax_withTransform)
{
  const std::string type("name");
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 5.0;
  pose.position.z = 7.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  const float depth = 19.0;
  const float width = 23.0;
  const float height = 29.0;
  simple_sensor_simulator::primitives::Box prim(depth, width, height, pose);

  geometry_msgs::msg::Pose sensor_pose{};
  sensor_pose.position.x = -11.0;
  sensor_pose.position.y = -13.0;
  sensor_pose.position.z = -17.0;
  sensor_pose.orientation.x = -0.5;
  sensor_pose.orientation.y = 0.5;
  sensor_pose.orientation.z = 0.5;
  sensor_pose.orientation.w = 0.5;

  const auto axis_x = math::geometry::Axis::X;
  const auto axis_y = math::geometry::Axis::Y;
  const auto axis_z = math::geometry::Axis::Z;

  const std::optional<double> max_x = prim.getMax(axis_x, sensor_pose);
  const std::optional<double> max_y = prim.getMax(axis_y, sensor_pose);
  const std::optional<double> max_z = prim.getMax(axis_z, sensor_pose);

  EXPECT_TRUE(max_x.has_value());
  EXPECT_TRUE(max_y.has_value());
  EXPECT_TRUE(max_z.has_value());

  EXPECT_TRUE(max_x.value() == pose.position.x - sensor_pose.position.x + height / 2);
  EXPECT_TRUE(max_y.value() == pose.position.y - sensor_pose.position.y + depth / 2);
  EXPECT_TRUE(max_z.value() == pose.position.z - sensor_pose.position.z + width / 2);
}