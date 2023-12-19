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

#ifndef MAP_FRAGMENT__MAP_FRAGMENT_HPP_
#define MAP_FRAGMENT__MAP_FRAGMENT_HPP_

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <limits>

#define DEBUG(...) std::cerr << #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

#define LINE() std::cerr << __FILE__ << ":" << __LINE__ << std::endl

namespace map_fragment
{
namespace default_value
{
/*
   The following default values are not based on technical basis. It can be
   any number that a typical scenario test would require.
*/
constexpr auto curvature = 0.0;

constexpr auto angle = 0.0;

auto directory() -> const auto &
{
  static const auto directory = std::filesystem::path("/tmp/map_fragment");
  return directory;
}

constexpr auto filename = "lanelet2_map.osm";

constexpr auto length = 100.0;

constexpr auto number_of_lanes = 1;

constexpr auto resolution = 100;

constexpr auto turn_radius = 50.0;

constexpr auto width = 10.0;
}  // namespace default_value

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

auto vehicleTrafficRules() -> const auto &
{
  static const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  return *rules;
}

auto pedestrianTrafficRules() -> const auto &
{
  static const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  return *rules;
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

template <typename Point1, typename Point2>
auto makePerpendicularAngle(const Point1 & p1, const Point2 & p2)
{
  return std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) + M_PI_2;
}

template <typename Point1, typename Point2>
auto makePerpendicularDirection(const Point1 & p1, const Point2 & p2)
{
  const auto r = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  const auto p = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
  const auto y = Eigen::AngleAxisd(makePerpendicularAngle(p1, p2), Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(r * p * y).matrix();
}

auto generateNextLineStringId()
{
  static lanelet::Id id = 0;
  return ++id;
}

auto makeLineString3d(
  const lanelet::Point3d & origin, double length, double radius, const Eigen::Matrix3d rotation,
  std::size_t resolution)
{
  if (std::isinf(radius)) {
    auto point = origin.basicPoint() + rotation * Eigen::Vector3d::UnitX() * length;
    return lanelet::LineString3d(generateNextLineStringId(), {origin, makePoint3d(point)});
  } else if (M_PI * 2 < length / radius) {
    std::exit(EXIT_FAILURE);
  } else {
    auto line = lanelet::LineString3d(generateNextLineStringId());

    const auto radian_step = length / radius / resolution;

    const auto point3d_at = [&](auto radian) {
      return makePoint3d(
        origin.basicPoint() + rotation * Eigen::Vector3d(
                                           radius * std::cos(radian - M_PI_2),
                                           radius * std::sin(radian - M_PI_2) + radius, 0));
    };

    line.push_back(origin);

    for (auto radian = radian_step; 1 < resolution; radian += radian_step, --resolution) {
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
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lane.attributes()[lanelet::AttributeName::OneWay] = false;
  return lane;
}

auto makeLanelet(
  const lanelet::Point3d & p1, const lanelet::Point3d & p2,  //
  double length, double curvature, double resolution)
{
  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto width = lanelet::geometry::distance3d(p1, p2);

  const auto p1_radius = radius - width / 2;
  const auto p2_radius = radius + width / 2;

  const auto aligned_length = [&](auto pn_radius) {
    return std::isinf(radius) ? length : length * pn_radius / radius;
  };

  const auto rotation = makePerpendicularDirection(p1, p2);

  return makeLanelet(
    makeLineString3d(p1, aligned_length(p1_radius), p1_radius, rotation, resolution),
    makeLineString3d(p2, aligned_length(p2_radius), p2_radius, rotation, resolution));
}

auto makeLanelet(
  const lanelet::Point3d & p1, const lanelet::LineString3d & right,  //
  double length, double curvature, double resolution)
{
  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto width = lanelet::geometry::distance3d(p1, right.front());

  const auto p1_radius = radius - width;

  const auto aligned_length = [&](auto pn_radius) {
    return std::isinf(radius) ? length : length * pn_radius / radius;
  };

  const auto rotation = makePerpendicularDirection(p1, right.front());

  return makeLanelet(
    makeLineString3d(p1, aligned_length(p1_radius), p1_radius, rotation, resolution), right);
}

auto makeLanelet(
  const lanelet::LineString3d & left, const lanelet::Point3d & p2,  //
  double length, double curvature, double resolution)
{
  const auto radius = curvature == 0 ? std::numeric_limits<double>::infinity() : 1 / curvature;

  const auto width = lanelet::geometry::distance3d(left.front(), p2);

  const auto p2_radius = radius + width;

  const auto aligned_length = [&](auto pn_radius) {
    return std::isinf(radius) ? length : length * pn_radius / radius;
  };

  const auto rotation = makePerpendicularDirection(left.front(), p2);

  return makeLanelet(
    left, makeLineString3d(p2, aligned_length(p2_radius), p2_radius, rotation, resolution));
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

auto length(const lanelet::ConstLineString3d & linestring) -> double
{
  return lanelet::geometry::length(linestring);
}

template <typename Point>
auto angle2d(const Point & p0, const Point & p1, const Point & p2)
{
  auto angle2d = [](const auto & v, const auto & w) {
    return std::atan2(w.y() - v.y(), w.x() - v.x());
  };
  auto canonicalize = [](auto radian) {
    while (M_PI < radian) {
      radian -= 2 * M_PI;
    }
    while (radian < -M_PI) {
      radian += 2 * M_PI;
    }
    return radian;
  };
  return canonicalize(
    angle2d(p1.basicPoint(), p2.basicPoint()) - angle2d(p0.basicPoint(), p1.basicPoint()));
}

auto curvature2d(const lanelet::ConstLineString3d & points)
{
  if (points.size() < 3) {
    return 0.0;
  } else {
    auto angle = 0.0;

    for (auto iter = points.begin(); std::next(iter, 2) != points.end(); ++iter) {
      angle += angle2d(*std::next(iter, 0), *std::next(iter, 1), *std::next(iter, 2));
    }

    const auto arc_length = length(points);

    const auto arc_angle = angle * (points.size() - 1) / (points.size() - 2);

    const auto curvature_radius = arc_length / arc_angle;

    return 1 / curvature_radius;
  }
}

auto makeLaneletLeft(lanelet::Lanelet & right, double resolution)
{
  return makeLanelet(
    makePoint3d(
      2 * right.leftBound().front().basicPoint() - right.rightBound().front().basicPoint()),
    right.leftBound(), length(right.leftBound()), curvature2d(right.leftBound()), resolution);
}

auto makeLaneletLeft(lanelet::Lanelet & right, lanelet::Lanelet & previous, double resolution)
{
  return makeLanelet(
    previous.leftBound().back(), right.leftBound(), length(right.leftBound()),
    curvature2d(right.leftBound()), resolution);
}

auto makeInvertedLaneletLeft(lanelet::Lanelet & right, double resolution)
{
  return makeLanelet(
    makePoint3d(2 * right.rightBound().back().basicPoint() - right.leftBound().back().basicPoint()),
    right.rightBound().invert(), length(right.rightBound().invert()),
    curvature2d(right.rightBound().invert()), resolution);
}

auto makeInvertedLaneletLeft(
  lanelet::Lanelet & right, lanelet::Lanelet & previous, double resolution)
{
  return makeLanelet(
    previous.leftBound().back(), right.rightBound().invert(), length(right.rightBound().invert()),
    curvature2d(right.rightBound().invert()), resolution);
}

auto makeLaneletRight(lanelet::Lanelet & left, double resolution)
{
  return makeLanelet(
    left.rightBound(),
    makePoint3d(2 * left.rightBound().front().basicPoint() - left.leftBound().front().basicPoint()),
    length(left.rightBound()), curvature2d(left.rightBound()), resolution);
}

auto makeLaneletRight(lanelet::Lanelet & left, lanelet::Lanelet & previous, double resolution)
{
  return makeLanelet(
    left.rightBound(), previous.rightBound().back(), length(left.rightBound()),
    curvature2d(left.rightBound()), resolution);
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

#endif  // MAP_FRAGMENT__MAP_FRAGMENT_HPP_
