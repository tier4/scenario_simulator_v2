#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

template <typename... Ts>
auto makePoint3d(Ts &&... xs)
{
  static lanelet::Id id = 0;
  auto point = lanelet::Point3d(++id, std::forward<decltype(xs)>(xs)...);
  point.attributes()["ele"] = 0;
  return point;
}

int main()
{
  auto width = 6;

  auto length = 1000;

  auto left_begin = makePoint3d(0, -width / 2, 0);

  auto left_end = makePoint3d(length, -width / 2, 0);

  auto right_begin = makePoint3d(0, width / 2, 0);

  auto right_end = makePoint3d(length, width / 2, 0);

  auto linestring_id = 0;

  auto left = lanelet::LineString3d(++linestring_id, {left_begin, left_end});
  auto right = lanelet::LineString3d(++linestring_id, {right_begin, right_end});

  auto lane_id = 0;

  auto lane = lanelet::Lanelet(++lane_id, left, right);
  lane.attributes()["subtype"] = "road";

  auto map = lanelet::LaneletMap();

  map.add(lane);

  lanelet::write(
    "/tmp/lanelet2_map.osm", map,
    lanelet::projection::UtmProjector(lanelet::Origin({35.624285, 139.742570})));

  return EXIT_SUCCESS;
}
