#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

int main()
{
  auto point_id = 0;

  auto width = 6;

  auto length = 1000;

  auto left_begin = lanelet::Point3d(++point_id, 0, -width / 2, 0);
  left_begin.attributes()["ele"] = 0;

  auto left_end = lanelet::Point3d(++point_id, length, -width / 2, 0);
  left_end.attributes()["ele"] = 0;

  auto right_begin = lanelet::Point3d(++point_id, 0, width / 2, 0);
  right_begin.attributes()["ele"] = 0;

  auto right_end = lanelet::Point3d(++point_id, length, width / 2, 0);
  right_end.attributes()["ele"] = 0;

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
