// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lanelet2_projection/UTM.h>

#include <filesystem>

namespace map_fragment
{
auto directory() -> const auto &
{
  static const auto directory = std::filesystem::path("/tmp/map_fragment");
  return directory;
}

auto origin() -> const auto &
{
  static const auto origin = lanelet::Origin({35.624285, 139.742570});
  return origin;
}

auto projector() -> const auto &
{
  static const auto projector = lanelet::projection::UtmProjector(origin());
  return projector;
}

auto makePoint3d(double x, double y, double z, double elevation = 0.0)
{
  static lanelet::Id id = 0;
  auto point = lanelet::Point3d(++id, x, y, z);
  point.attributes()["ele"] = elevation;
  return point;
}

auto makeLineString3d(
  const lanelet::Point3d & origin, double length, double radius, std::size_t resolution)
{
  static lanelet::Id id = 0;

  if (std::isinf(radius)) {
    return lanelet::LineString3d(
      ++id, {origin, makePoint3d(origin.x() + length, origin.y(), origin.z())});
  } else {
    auto line = lanelet::LineString3d(++id);

    if (M_PI * 2 < length / radius) {
      std::exit(EXIT_FAILURE);
    }

    const auto radian_step = length / radius / resolution;

    auto total_length = 0.0;

    for (auto radian = 0.0; 0 < resolution; radian += radian_step, --resolution) {
      auto x = origin.x() + radius * std::sin(radian);
      auto y = origin.y() + radius * std::cos(radian) - radius;
      auto z = origin.z();
      line.push_back(makePoint3d(x, y, z));
      total_length += radius * radian_step;
    }

    auto x = origin.x() + radius * std::sin(length / radius);
    auto y = origin.y() + radius * std::cos(length / radius) - radius;
    auto z = origin.z();
    line.push_back(makePoint3d(x, y, z));

    return line;
  }
}

auto makeLanelet(double length, double width, double curvature, double resolution)
{
  const auto x = 0.0;
  const auto y = 0.0;
  const auto z = 0.0;

  const auto p1 = makePoint3d(x, y - width / 2, z);
  const auto p2 = makePoint3d(x, y + width / 2, z);

  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto p1_radius = radius - width / 2;
  const auto p2_radius = radius + width / 2;

  const auto p1_offset = std::isinf(radius) or std::isinf(p1_radius)
                           ? 0.0
                           : p1_radius * (length / radius - length / p1_radius);

  const auto p2_offset = std::isinf(radius) or std::isinf(p2_radius)
                           ? 0.0
                           : p2_radius * (length / radius - length / p2_radius);

  static lanelet::Id id = 0;
  auto lane = lanelet::Lanelet(
    ++id, makeLineString3d(p1, length + p1_offset, p1_radius, resolution),
    makeLineString3d(p2, length + p2_offset, p2_radius, resolution));
  lane.attributes()["subtype"] = "road";
  return lane;
}
}  // namespace map_fragment
