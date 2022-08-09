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

// Authors: Simon Thompson, Ryohsuke Mitsudome

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <lanelet2_extension_psim/exception.hpp>
#include <lanelet2_extension_psim/projection/mgrs_projector.hpp>
#include <lanelet2_extension_psim/utility/message_conversion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

namespace lanelet
{
namespace utils
{
namespace conversion
{
void toBinMsg(
  const std::unique_ptr<LaneletMap> & map, autoware_auto_mapping_msgs::msg::HADMapBin * msg)
{
  if (msg == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "msg is null pointer!";
    lanelet::HdMapException(sstream.str());
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void toBinMsg(const lanelet::LaneletMapPtr & map, autoware_auto_mapping_msgs::msg::HADMapBin * msg)
{
  if (msg == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "msg is null pointer!";
    lanelet::HdMapException(sstream.str());
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg, lanelet::LaneletMapPtr map)
{
  if (!map) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "msg is null pointer!";
    lanelet::HdMapException(sstream.str());
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  // *map = std::move(laneletMap);
}

void fromBinMsg(
  const autoware_auto_mapping_msgs::msg::HADMapBin & msg, const std::unique_ptr<LaneletMap> & map)
{
  if (!map) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "msg is null pointer!";
    lanelet::HdMapException(sstream.str());
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  // *map = std::move(laneletMap);
}

void fromBinMsg(
  const autoware_auto_mapping_msgs::msg::HADMapBin & msg, lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph)
{
  fromBinMsg(msg, map);
  *traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  *routing_graph = lanelet::routing::RoutingGraph::build(*map, **traffic_rules);
}

void toGeomMsgPt(const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "pointer is null!";
    lanelet::HdMapException(sstream.str());
    return;
  }
  dst->x = src.x;
  dst->y = src.y;
  dst->z = src.z;
}
void toGeomMsgPt(const Eigen::Vector3d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "pointer is null!";
    lanelet::HdMapException(sstream.str());
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint3d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "pointer is null!";
    lanelet::HdMapException(sstream.str());
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint2d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "pointer is null!";
    lanelet::HdMapException(sstream.str());
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = 0;
}

void toGeomMsgPt32(const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst)
{
  if (dst == nullptr) {
    std::stringstream sstream;
    sstream << __FUNCTION__ << "pointer is null!";
    lanelet::HdMapException(sstream.str());
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}

geometry_msgs::msg::Point toGeomMsgPt(const geometry_msgs::msg::Point32 & src)
{
  geometry_msgs::msg::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point toGeomMsgPt(const Eigen::Vector3d & src)
{
  geometry_msgs::msg::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint3d & src)
{
  geometry_msgs::msg::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint2d & src)
{
  geometry_msgs::msg::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}

lanelet::ConstPoint3d toLaneletPoint(const geometry_msgs::msg::Point & src)
{
  lanelet::ConstPoint3d dst;
  toLaneletPoint(src, &dst);
  return dst;
}

void toLaneletPoint(const geometry_msgs::msg::Point & src, lanelet::ConstPoint3d * dst)
{
  *dst = lanelet::Point3d(lanelet::InvalId, src.x, src.y, src.z);
}

void toGeomMsgPoly(const lanelet::ConstPolygon3d & ll_poly, geometry_msgs::msg::Polygon * geom_poly)
{
  geom_poly->points.clear();
  geom_poly->points.reserve(ll_poly.size());
  for (const auto & ll_pt : ll_poly) {
    geometry_msgs::msg::Point32 geom_pt32;
    utils::conversion::toGeomMsgPt32(ll_pt.basicPoint(), &geom_pt32);
    geom_poly->points.push_back(geom_pt32);
  }
}

}  // namespace conversion
}  // namespace utils
}  // namespace lanelet
