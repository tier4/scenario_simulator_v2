#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#define PRINT(...) std::cout << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

auto makePoint3d(double x, double y, double z)
{
  static lanelet::Id id = 0;
  auto point = lanelet::Point3d(++id, x, y, z);
  point.attributes()["ele"] = 0;
  return point;
}

auto makeLineString3d(const lanelet::Point3d & begin, const lanelet::Point3d & end)
{
  static lanelet::Id id = 0;
  return lanelet::LineString3d(++id, {begin, end});
}

auto makeLane(double length = 1000, double width = 10, double curvature = 0)
{
  auto x = 0.0;
  auto y = 0.0;
  auto z = 0.0;

  auto p1 = makePoint3d(x, y - width / 2, z);
  auto p2 = makePoint3d(x, y + width / 2, z);
  auto p3 = makePoint3d(p1.x() + length, p1.y(), p1.z());
  auto p4 = makePoint3d(p2.x() + length, p2.y(), p2.z());

  static lanelet::Id id = 0;

  auto lane = lanelet::Lanelet(++id, makeLineString3d(p1, p3), makeLineString3d(p2, p4));

  lane.attributes()["subtype"] = "road";

  return lane;
}

int main()
{
  auto map = lanelet::LaneletMap();

  map.add(makeLane());

  lanelet::write(
    "/tmp/lanelet2_map.osm", map,
    lanelet::projection::UtmProjector(lanelet::Origin({35.624285, 139.742570})));

  return EXIT_SUCCESS;
}
