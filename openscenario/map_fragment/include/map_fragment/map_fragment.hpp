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

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#define DEBUG(...) std::cerr << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

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

template <typename... Ts>
auto makePoint3d(const Eigen::Vector3d & v, Ts &&... xs) -> decltype(auto)
{
  return makePoint3d(v.x(), v.y(), v.z(), std::forward<decltype(xs)>(xs)...);
}

auto makePerpendicularDirection(const lanelet::Point3d p1, const lanelet::Point3d & p2)
{
  const auto r = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  const auto p = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  const auto y = Eigen::AngleAxisd(
    std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) + M_PI_2, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(r * p * y).matrix();
}

auto makeLineString3d(
  const lanelet::Point3d & origin, double length, double radius, const Eigen::Matrix3d rotation,
  std::size_t resolution)
{
  static lanelet::Id id = 0;

  if (std::isinf(radius)) {
    auto point = origin.basicPoint() + rotation * Eigen::Vector3d::UnitX() * length;
    return lanelet::LineString3d(++id, {origin, makePoint3d(point)});
  } else if (M_PI * 2 < length / radius) {
    std::exit(EXIT_FAILURE);
  } else {
    auto line = lanelet::LineString3d(++id);

    const auto radian_step = length / radius / resolution;

    const auto point3d_at = [&](auto radian) {
      return makePoint3d(
        origin.basicPoint() +
        rotation *
          Eigen::Vector3d(radius * std::sin(radian), radius * std::cos(radian) - radius, 0));
    };

    for (auto radian = 0.0; 0 < resolution; radian += radian_step, --resolution) {
      line.push_back(point3d_at(radian));
    }

    line.push_back(point3d_at(length / radius));

    return line;
  }
}

auto makeLanelet(const lanelet::LineString3d & left, const lanelet::LineString3d & right)
{
  static lanelet::Id id = 0;
  auto lane = lanelet::Lanelet(++id, left, right);
  lane.attributes()["subtype"] = "road";
  return lane;
}

template <typename... Ts>
auto makeLanelet(const std::tuple<Ts...> & left, const std::tuple<Ts...> & right)
{
  return makeLanelet(std::apply(makeLineString3d, left), std::apply(makeLineString3d, right));
}

auto makeLanelet(
  const lanelet::Point3d & p1, const lanelet::Point3d & p2,  //
  double length, double curvature, double resolution)
{
  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto width = lanelet::geometry::distance3d(p1, p2);

  const auto p1_radius = radius + width / 2;
  const auto p2_radius = radius - width / 2;

  const auto aligned_length = [&](auto pn_radius) {
    return std::isinf(radius) ? length : length * pn_radius / radius;
  };

  const auto rotation = makePerpendicularDirection(p1, p2);

  return makeLanelet(
    std::forward_as_tuple(p1, aligned_length(p1_radius), p1_radius, rotation, resolution),
    std::forward_as_tuple(p2, aligned_length(p2_radius), p2_radius, rotation, resolution));
}

auto makeLanelet(
  const lanelet::Point3d & origin, double width, double length, double curvature, double resolution)
{
  return makeLanelet(
    makePoint3d(origin.x(), origin.y() + width / 2, origin.z()),
    makePoint3d(origin.x(), origin.y() - width / 2, origin.z()),  //
    length, curvature, resolution);
}

auto makeLanelet(lanelet::Lanelet & lanelet, double length, double curvature, double resolution)
{
  return makeLanelet(
    lanelet.leftBound().back(), lanelet.rightBound().back(), length, curvature, resolution);
}

auto makeLanelet(double width, double length, double curvature, double resolution)
{
  return makeLanelet(makePoint3d(0.0, 0.0, 0.0), width, length, curvature, resolution);
}

auto write(const lanelet::LaneletMap & map, const std::filesystem::path & output_directory)
{
  if (std::filesystem::remove_all(output_directory);
      not std::filesystem::create_directories(output_directory)) {
    throw std::runtime_error(std::string("failed to create directory ") + output_directory.c_str());
  } else {
    lanelet::write(output_directory / "lanelet2_map.osm", map, projector());

    std::filesystem::create_symlink(
      std::filesystem::canonical(
        std::filesystem::path(ament_index_cpp::get_package_share_directory("kashiwanoha_map")) /
        "map/pointcloud_map.pcd"),
      output_directory / "pointcloud_map.pcd");
  }
}

auto makeCurvature(double arc_length, double degree)
{
  auto degree_to_radian = [](auto degree) constexpr { return degree * M_PI / 180; };

  return degree_to_radian(degree) / arc_length;
}
}  // namespace map_fragment
