// Copyright 2015-2019 Tier IV, Inc. All rights reserved.
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

// Authors: Ryohsuke Mitsudome

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <lanelet2_extension_psim/io/autoware_osm_parser.hpp>
#include <lanelet2_extension_psim/utility/message_conversion.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lanelet
{
namespace io_handlers
{
// non-const converter for const lanelet primitive types
struct NonConstConverter
{
  auto operator()(const ConstPoint3d & c)
  {
    return Point3d(c.id(), c.basicPoint(), c.attributes());
  }

  auto operator()(const ConstLineString3d & c)
  {
    return LineString3d(c.id(), std::vector<Point3d>(c.begin(), c.end()), c.attributes());
  }

  auto operator()(const ConstLineStrings3d & cs)
  {
    auto ls = LineStrings3d{};
    ls.reserve(cs.size());
    for (auto && c : cs) {
      ls.emplace_back(NonConstConverter()(c));
    }
    return ls;
  }

  auto operator()(const ConstInnerBounds & cs)
  {
    auto ib = InnerBounds{};
    ib.reserve(cs.size());
    for (auto && c : cs) {
      ib.emplace_back(NonConstConverter()(c));
    }
    return ib;
  }

  auto operator()(const ConstPolygon3d & c)
  {
    return Polygon3d(c.id(), std::vector<Point3d>(c.begin(), c.end()), c.attributes());
  }

  auto operator()(const ConstWeakArea & e)
  {
    auto c = e.lock();
    return WeakArea(Area(std::const_pointer_cast<AreaData>(c.constData())));
  }

  auto operator()(const ConstWeakLanelet & e)
  {
    auto c = e.lock();
    return WeakLanelet(Lanelet(std::const_pointer_cast<LaneletData>(c.constData())));
  }
};

std::unique_ptr<LaneletMap> AutowareOsmParser::parse(
  const std::string & filename, ErrorMessages & errors) const
{
  std::unique_ptr<LaneletMap> map = OsmParser::parse(filename, errors);
  // overwrite x and y values if there are local_x, local_y tags
  for (Point3d point : map->pointLayer) {
    if (point.hasAttribute("local_x")) {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y")) {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }
  autoware_auto_mapping_msgs::msg::HADMapBin map_bin_msg;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);
  lanelet::utils::conversion::fromBinMsg(map_bin_msg, map);
  return map;
}

namespace
{
RegisterParser<AutowareOsmParser> regParser;
}

void AutowareOsmParser::parseVersions(
  const std::string & filename, std::string * format_version, std::string * map_version)
{
  if (format_version == nullptr || map_version == nullptr) {
    std::cerr << __FUNCTION__ << ": either format_version or map_version is null pointer!";
    return;
  }

  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError(
      std::string("Errors occured while parsing osm file: ") + result.description());
  }

  auto osmNode = doc.child("osm");
  auto metainfo = osmNode.child("MetaInfo");
  if (metainfo.attribute("format_version")) {
    *format_version = metainfo.attribute("format_version").value();
  }
  if (metainfo.attribute("map_version")) {
    *map_version = metainfo.attribute("map_version").value();
  }
}

}  // namespace io_handlers
}  // namespace lanelet
