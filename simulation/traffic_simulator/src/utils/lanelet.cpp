// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry/transform.hpp>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/utils/lanelet.hpp>

namespace traffic_simulator
{
namespace lanelet2
{

boost::filesystem::path getLaneletPath()
{
  return ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map/" +
         "private_road_and_walkway_ele_fix/lanelet2_map.osm";
}

geographic_msgs::msg::GeoPoint getOrigin()
{
  rclcpp::Node node{"lanelet_get_parameter", "simulation"};
  geographic_msgs::msg::GeoPoint origin;
  if (!node.has_parameter("origin_latitude")) {
    node.declare_parameter("origin_latitude", 0.0);
  }
  if (!node.has_parameter("origin_longitude")) {
    node.declare_parameter("origin_longitude", 0.0);
  }
  node.get_parameter("origin_latitude", origin.latitude);
  node.get_parameter("origin_longitude", origin.longitude);
  return origin;
}

class Memory
{
private:
  static std::unique_ptr<Memory> instance;
  static std::mutex mutex_;

  auto overwriteLaneletsCenterline() -> void
  {
    for (auto & lanelet_obj : lanelet_map_ptr_->laneletLayer) {
      if (!lanelet_obj.hasCustomCenterline()) {
        const auto fine_center_line = generateFineCenterline(lanelet_obj, 2.0);
        lanelet_obj.setCenterline(fine_center_line);
      }
    }
  }

public:
  Memory(const Memory &) = delete;
  Memory & operator=(const Memory &) = delete;
  Memory() = delete;
  ~Memory() = default;

  static Memory & getInstance()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance) {
      std::cout << std::endl
                << " ###### make instance ###### " << getLaneletPath() << std::endl
                << std::endl;
      instance.reset(new Memory(getLaneletPath(), getOrigin()));
    }
    return *instance;
  }

  Memory(const boost::filesystem::path & lanelet2_map_path, const geographic_msgs::msg::GeoPoint &)
  {
    lanelet::projection::MGRSProjector projector;

    lanelet::ErrorMessages errors;

    lanelet_map_ptr_ = lanelet::load(lanelet2_map_path.string(), projector, &errors);

    if (not errors.empty()) {
      std::stringstream ss;
      const auto * separator = "";
      for (const auto & error : errors) {
        ss << separator << error;
        separator = "\n";
      }
      THROW_SIMULATION_ERROR("Failed to load lanelet map (", ss.str(), ")");
    }
    overwriteLaneletsCenterline();
    traffic_rules_vehicle_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    vehicle_routing_graph_ptr_ =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_vehicle_ptr_);
    traffic_rules_pedestrian_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
    pedestrian_routing_graph_ptr_ =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_pedestrian_ptr_);
    std::vector<lanelet::routing::RoutingGraphConstPtr> all_graphs;
    all_graphs.push_back(vehicle_routing_graph_ptr_);
    all_graphs.push_back(pedestrian_routing_graph_ptr_);
    shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(
      lanelet::utils::query::laneletLayer(lanelet_map_ptr_));
  }

  mutable hdmap_utils::RouteCache route_cache_;
  mutable hdmap_utils::CenterPointsCache center_points_cache_;
  mutable hdmap_utils::LaneletLengthCache lanelet_length_cache_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;
};

std::unique_ptr<Memory> Memory::instance = nullptr;
std::mutex Memory::mutex_;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::Id lanelet_id,
  const double matching_distance) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  const auto spline = getCenterPointsSpline(lanelet_id);
  const auto s = spline->getSValue(pose, matching_distance);
  if (!s) {
    return std::nullopt;
  }
  auto pose_on_centerline = spline->getPose(s.value());
  auto rpy = quaternion_operation::convertQuaternionToEulerAngle(
    quaternion_operation::getRotation(pose_on_centerline.orientation, pose.orientation));
  double offset = std::sqrt(spline->getSquaredDistanceIn2D(pose.position, s.value()));
  /**
   * @note Hard coded parameter
   */
  double yaw_threshold = 0.25;
  if (M_PI * yaw_threshold < std::fabs(rpy.z) && std::fabs(rpy.z) < M_PI * (1 - yaw_threshold)) {
    return std::nullopt;
  }
  double inner_prod = math::geometry::innerProduct(
    spline->getNormalVector(s.value()), spline->getSquaredDistanceVector(pose.position, s.value()));
  if (inner_prod < 0) {
    offset = offset * -1;
  }
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s.value();
  lanelet_pose.offset = offset;
  lanelet_pose.rpy = rpy;
  return lanelet_pose;
}

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const bool include_crosswalk,
  const double matching_distance) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  const auto lanelet_ids = getNearbyLaneletIds(pose.position, 0.1, include_crosswalk);
  if (lanelet_ids.empty()) {
    return std::nullopt;
  }
  for (const auto & id : lanelet_ids) {
    const auto lanelet_pose = toLaneletPose(pose, id, matching_distance);
    if (lanelet_pose) {
      return lanelet_pose;
    }
  }
  return std::nullopt;
}

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::Ids & lanelet_ids,
  const double matching_distance) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  for (const auto id : lanelet_ids) {
    if (const auto lanelet_pose = toLaneletPose(pose, id, matching_distance); lanelet_pose) {
      return lanelet_pose.value();
    }
  }
  return std::nullopt;
}

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const bool include_crosswalk, const double matching_distance)
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>
{
  const auto lanelet_id = matchToLane(pose, bbox, include_crosswalk, matching_distance);
  if (!lanelet_id) {
    return toLaneletPose(pose, include_crosswalk, matching_distance);
  }
  const auto pose_in_target_lanelet = toLaneletPose(pose, lanelet_id.value(), matching_distance);
  if (pose_in_target_lanelet) {
    return pose_in_target_lanelet;
  }
  const auto previous = getPreviousLaneletIds(lanelet_id.value());
  for (const auto id : previous) {
    const auto pose_in_previous = toLaneletPose(pose, id, matching_distance);
    if (pose_in_previous) {
      return pose_in_previous;
    }
  }
  const auto next = getNextLaneletIds(lanelet_id.value());
  for (const auto id : previous) {
    const auto pose_in_next = toLaneletPose(pose, id, matching_distance);
    if (pose_in_next) {
      return pose_in_next;
    }
  }
  return toLaneletPose(pose, include_crosswalk);
}

auto toLaneletPoses(
  const geometry_msgs::msg::Pose & pose, const lanelet::Id lanelet_id,
  const double matching_distance, const bool include_opposite_direction)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> ret;
  traffic_simulator_msgs::msg::EntityType type;
  type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  std::vector lanelet_ids = {lanelet_id};
  lanelet_ids += getLeftLaneletIds(lanelet_id, type, include_opposite_direction);
  lanelet_ids += getRightLaneletIds(lanelet_id, type, include_opposite_direction);
  lanelet_ids += getPreviousLaneletIds(lanelet_ids);
  lanelet_ids += getNextLaneletIds(lanelet_ids);
  for (const auto & id : sortAndUnique(lanelet_ids)) {
    if (const auto & lanelet_pose = toLaneletPose(pose, id, matching_distance)) {
      ret.emplace_back(lanelet_pose.value());
    }
  }
  return ret;
}

auto getCenterPointsSpline(const lanelet::Id lanelet_id)
  -> std::shared_ptr<math::geometry::CatmullRomSpline>
{
  getCenterPoints(lanelet_id);
  return Memory::getInstance().center_points_cache_.getCenterPointsSpline(lanelet_id);
}

auto getCenterPoints(const lanelet::Ids & lanelet_ids) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto lanelet_id : lanelet_ids) {
    ret += getCenterPoints(lanelet_id);
  }
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}
auto getNearbyLaneletIds(
  const geometry_msgs::msg::Point & position, const double distance_threshold,
  const std::size_t search_count) -> lanelet::Ids
{
  lanelet::Ids lanelet_ids;
  lanelet::BasicPoint2d search_point(position.x, position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet = lanelet::geometry::findNearest(
    Memory::getInstance().lanelet_map_ptr_->laneletLayer, search_point, search_count);
  if (nearest_lanelet.empty()) {
    return {};
  }
  for (const auto & lanelet : nearest_lanelet) {
    if (lanelet.first <= distance_threshold) {
      lanelet_ids.emplace_back(lanelet.second.id());
    }
  }
  return lanelet_ids;
}

auto getNearbyLaneletIds(
  const geometry_msgs::msg::Point & point, const double distance_thresh,
  const bool include_crosswalk, const std::size_t search_count) -> lanelet::Ids
{
  lanelet::Ids lanelet_ids;
  lanelet::BasicPoint2d search_point(point.x, point.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet = lanelet::geometry::findNearest(
    Memory::getInstance().lanelet_map_ptr_->laneletLayer, search_point, search_count);
  if (include_crosswalk) {
    if (nearest_lanelet.empty()) {
      return {};
    }
    if (nearest_lanelet.front().first > distance_thresh) {
      return {};
    }
    for (const auto & lanelet : nearest_lanelet) {
      lanelet_ids.emplace_back(lanelet.second.id());
    }
  } else {
    const auto nearest_road_lanelet =
      excludeSubtypeLanelets(nearest_lanelet, lanelet::AttributeValueString::Crosswalk);
    if (nearest_road_lanelet.empty()) {
      return {};
    }
    if (nearest_road_lanelet.front().first > distance_thresh) {
      return {};
    }
    for (const auto & lanelet : nearest_lanelet) {
      lanelet_ids.emplace_back(lanelet.second.id());
    }
  }
  return lanelet_ids;
}

auto getLeftLaneletIds(
  const lanelet::Id lanelet_id, const traffic_simulator_msgs::msg::EntityType & type,
  const bool include_opposite_direction) -> lanelet::Ids
{
  switch (type.type) {
    case traffic_simulator_msgs::msg::EntityType::EGO:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->lefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->adjacentLefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    case traffic_simulator_msgs::msg::EntityType::VEHICLE:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->lefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->adjacentLefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    case traffic_simulator_msgs::msg::EntityType::PEDESTRIAN:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().pedestrian_routing_graph_ptr_->lefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().pedestrian_routing_graph_ptr_->adjacentLefts(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    default:
    case traffic_simulator_msgs::msg::EntityType::MISC_OBJECT:
      return {};
  }
}

auto getRightLaneletIds(
  lanelet::Id lanelet_id, traffic_simulator_msgs::msg::EntityType type,
  bool include_opposite_direction) -> lanelet::Ids
{
  switch (type.type) {
    case traffic_simulator_msgs::msg::EntityType::EGO:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->rights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->adjacentRights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    case traffic_simulator_msgs::msg::EntityType::VEHICLE:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->rights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().vehicle_routing_graph_ptr_->adjacentRights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    case traffic_simulator_msgs::msg::EntityType::PEDESTRIAN:
      if (include_opposite_direction) {
        return getLaneletIds(Memory::getInstance().pedestrian_routing_graph_ptr_->rights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      } else {
        return getLaneletIds(Memory::getInstance().pedestrian_routing_graph_ptr_->adjacentRights(
          Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id)));
      }
    default:
    case traffic_simulator_msgs::msg::EntityType::MISC_OBJECT:
      return {};
  }
}

auto matchToLane(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const bool include_crosswalk, const double matching_distance, const double reduction_ratio)
  -> std::optional<lanelet::Id>
{
  std::optional<lanelet::Id> id;
  lanelet::matching::Object2d obj;
  obj.pose.translation() = toPoint2d(pose.position);
  obj.pose.linear() = Eigen::Rotation2D<double>(
                        quaternion_operation::convertQuaternionToEulerAngle(pose.orientation).z)
                        .matrix();
  obj.absoluteHull = absoluteHull(
    lanelet::matching::Hull2d{
      lanelet::BasicPoint2d{
        bbox.center.x + bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y + bbox.dimensions.y * 0.5 * reduction_ratio},
      lanelet::BasicPoint2d{
        bbox.center.x - bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y - bbox.dimensions.y * 0.5 * reduction_ratio}},
    obj.pose);
  auto matches = lanelet::matching::getDeterministicMatches(
    *Memory::getInstance().lanelet_map_ptr_, obj, matching_distance);
  if (!include_crosswalk) {
    matches = lanelet::matching::removeNonRuleCompliantMatches(
      matches, Memory::getInstance().traffic_rules_vehicle_ptr_);
  }
  if (matches.empty()) {
    return std::nullopt;
  }
  std::vector<std::pair<lanelet::Id, double>> id_and_distance;
  for (const auto & match : matches) {
    if (const auto lanelet_pose = toLaneletPose(pose, match.lanelet.id(), matching_distance)) {
      id_and_distance.emplace_back(lanelet_pose->lanelet_id, lanelet_pose->offset);
    }
  }
  if (id_and_distance.empty()) {
    return std::nullopt;
  }
  const auto min_id_and_distance = std::min_element(
    id_and_distance.begin(), id_and_distance.end(),
    [](auto const & lhs, auto const & rhs) { return lhs.second < rhs.second; });
  return min_id_and_distance->first;
}

auto getNextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::getInstance().vehicle_routing_graph_ptr_->following(lanelet)) {
    ids.push_back(llt.id());
  }
  for (const auto & id : getNextRoadShoulderLanelet(lanelet_id)) {
    ids.push_back(id);
  }
  return ids;
}

auto getNextLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id);
  }
  return sortAndUnique(ids);
}

auto getNextLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::getInstance().vehicle_routing_graph_ptr_->following(lanelet)) {
    if (llt.attributeOr("turn_direction", "else") == turn_direction) {
      ids.push_back(llt.id());
    }
  }
  return ids;
}

auto getNextLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id, turn_direction);
  }
  return sortAndUnique(ids);
}

auto getPreviousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::getInstance().vehicle_routing_graph_ptr_->previous(lanelet)) {
    ids.push_back(llt.id());
  }
  for (const auto & id : getPreviousRoadShoulderLanelet(lanelet_id)) {
    ids.push_back(id);
  }
  return ids;
}

auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id);
  }
  return sortAndUnique(ids);
}

auto getPreviousLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::getInstance().vehicle_routing_graph_ptr_->previous(lanelet)) {
    if (llt.attributeOr("turn_direction", "else") == turn_direction) {
      ids.push_back(llt.id());
    }
  }
  return ids;
}

auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getPreviousLaneletIds(id, turn_direction);
  }
  return sortAndUnique(ids);
}

auto getCenterPoints(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (!Memory::getInstance().lanelet_map_ptr_) {
    THROW_SIMULATION_ERROR("lanelet map is null pointer");
  }
  if (Memory::getInstance().lanelet_map_ptr_->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("lanelet layer is empty");
  }
  if (Memory::getInstance().center_points_cache_.exists(lanelet_id)) {
    return Memory::getInstance().center_points_cache_.getCenterPoints(lanelet_id);
  }

  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto centerline = lanelet.centerline();
  for (const auto & point : centerline) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    ret.push_back(p);
  }
  if (static_cast<int>(ret.size()) == 2) {
    const auto p0 = ret[0];
    const auto p2 = ret[1];
    geometry_msgs::msg::Point p1;
    p1.x = (p0.x + p2.x) * 0.5;
    p1.y = (p0.y + p2.y) * 0.5;
    p1.z = (p0.z + p2.z) * 0.5;
    ret.clear();
    ret.push_back(p0);
    ret.push_back(p1);
    ret.push_back(p2);
  }
  Memory::getInstance().center_points_cache_.appendData(lanelet_id, ret);
  return ret;
}

auto getLaneletIds() -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet : Memory::getInstance().lanelet_map_ptr_->laneletLayer) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

auto getNextRoadShoulderLanelet(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & shoulder_lanelet : Memory::getInstance().shoulder_lanelets_) {
    if (lanelet::geometry::follows(lanelet, shoulder_lanelet)) {
      ids.push_back(shoulder_lanelet.id());
    }
  }
  return ids;
}

auto getPreviousRoadShoulderLanelet(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  for (const auto & shoulder_lanelet : Memory::getInstance().shoulder_lanelets_) {
    if (lanelet::geometry::follows(shoulder_lanelet, lanelet)) {
      ids.push_back(shoulder_lanelet.id());
    }
  }
  return ids;
}

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>
{
  std::vector<std::pair<double, lanelet::Lanelet>> exclude_subtype_lanelets;
  for (const auto & ll : lls) {
    if (ll.second.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ll.second.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() != subtype) {
        exclude_subtype_lanelets.push_back(ll);
      }
    }
  }
  return exclude_subtype_lanelets;
}

auto toPoint2d(const geometry_msgs::msg::Point & point) -> lanelet::BasicPoint2d
{
  return lanelet::BasicPoint2d{point.x, point.y};
}

auto absoluteHull(
  const lanelet::BasicPolygon2d & relativeHull, const lanelet::matching::Pose2d & pose)
  -> lanelet::BasicPolygon2d
{
  lanelet::BasicPolygon2d hullPoints;
  hullPoints.reserve(relativeHull.size());
  for (const auto & hullPt : relativeHull) {
    hullPoints.push_back(pose * hullPt);
  }
  return hullPoints;
}

auto generateFineCenterline(const lanelet::ConstLanelet & lanelet_obj, const double resolution)
  -> lanelet::LineString3d
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int32_t num_segments =
    std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
    const lanelet::Point3d center_point(
      lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
      center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

auto resamplePoints(const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
  -> lanelet::BasicPoints3d
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  lanelet::BasicPoints3d resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const double target_length =
      (static_cast<double>(i) / num_segments) * static_cast<double>(line_length);
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }
  return resampled_points;
}

auto calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
  -> std::vector<double>
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));
  return accumulated_lengths;
}

auto findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length)
  -> std::pair<std::size_t, std::size_t>
{
  // List size
  const auto N = accumulated_lengths.size();
  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }
  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i)) {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  THROW_SEMANTIC_ERROR("findNearestIndexPair(): No nearest point found.");
}

auto calculateSegmentDistances(const lanelet::ConstLineString3d & line_string)
  -> std::vector<double>
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);
  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }
  return segment_distances;
}

////////////////////////////////////////
/////////////////////////////////////// lanelet_pose
/////////////////////////////////////

auto getAllCanonicalizedLaneletPoses(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>
{
  /// @note Define lambda functions for canonicalizing previous/next lanelet.
  const auto canonicalize_to_previous_lanelet =
    [](const auto & lanelet_pose) -> std::vector<traffic_simulator_msgs::msg::LaneletPose> {
    if (const auto ids = getPreviousLaneletIds(lanelet_pose.lanelet_id); !ids.empty()) {
      std::vector<traffic_simulator_msgs::msg::LaneletPose> canonicalized_all;
      for (const auto id : ids) {
        const auto lanelet_pose_tmp = traffic_simulator::helper::constructLaneletPose(
          id, lanelet_pose.s + getLaneletLength(id), lanelet_pose.offset);
        if (const auto canonicalized_lanelet_poses =
              getAllCanonicalizedLaneletPoses(lanelet_pose_tmp);
            canonicalized_lanelet_poses.empty()) {
          canonicalized_all.emplace_back(lanelet_pose_tmp);
        } else {
          std::copy(
            canonicalized_lanelet_poses.begin(), canonicalized_lanelet_poses.end(),
            std::back_inserter(canonicalized_all));
        }
      }
      return canonicalized_all;
    } else {
      return {};
    }
  };
  const auto canonicalize_to_next_lanelet =
    [](const auto & lanelet_pose) -> std::vector<traffic_simulator_msgs::msg::LaneletPose> {
    if (const auto ids = getNextLaneletIds(lanelet_pose.lanelet_id); !ids.empty()) {
      std::vector<traffic_simulator_msgs::msg::LaneletPose> canonicalized_all;
      for (const auto id : ids) {
        const auto lanelet_pose_tmp = traffic_simulator::helper::constructLaneletPose(
          id, lanelet_pose.s - getLaneletLength(lanelet_pose.lanelet_id), lanelet_pose.offset);
        if (const auto canonicalized_lanelet_poses =
              getAllCanonicalizedLaneletPoses(lanelet_pose_tmp);
            canonicalized_lanelet_poses.empty()) {
          canonicalized_all.emplace_back(lanelet_pose_tmp);
        } else {
          std::copy(
            canonicalized_lanelet_poses.begin(), canonicalized_lanelet_poses.end(),
            std::back_inserter(canonicalized_all));
        }
      }
      return canonicalized_all;
    } else {
      return {};
    }
  };

  /// @note If s value under 0, it means this pose is on the previous lanelet.
  if (lanelet_pose.s < 0) {
    return canonicalize_to_previous_lanelet(lanelet_pose);
  }
  /// @note If s value overs it's lanelet length, it means this pose is on the next lanelet.
  else if (lanelet_pose.s > (getLaneletLength(lanelet_pose.lanelet_id))) {
    return canonicalize_to_next_lanelet(lanelet_pose);
  }
  /// @note If s value is in range [0,length_of_the_lanelet], return lanelet_pose.
  else {
    return {lanelet_pose};
  }
}

// If route is not specified, the lanelet_id with the lowest array index is used as a candidate for
// canonicalize destination.
auto canonicalizeLaneletPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>
{
  auto canonicalized = lanelet_pose;
  while (canonicalized.s < 0) {
    if (const auto ids = getPreviousLaneletIds(canonicalized.lanelet_id); ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s += getLaneletLength(ids[0]);
      canonicalized.lanelet_id = ids[0];
    }
  }
  while (canonicalized.s > getLaneletLength(canonicalized.lanelet_id)) {
    if (const auto ids = getNextLaneletIds(canonicalized.lanelet_id); ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s -= getLaneletLength(canonicalized.lanelet_id);
      canonicalized.lanelet_id = ids[0];
    }
  }
  return {canonicalized, std::nullopt};
}

auto canonicalizeLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
  const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>
{
  auto canonicalized = lanelet_pose;
  while (canonicalized.s < 0) {
    // When canonicalizing to backward lanelet_id, do not consider route
    if (const auto ids = getPreviousLaneletIds(canonicalized.lanelet_id); ids.empty()) {
      return {std::nullopt, canonicalized.lanelet_id};
    } else {
      canonicalized.s += getLaneletLength(ids[0]);
      canonicalized.lanelet_id = ids[0];
    }
  }
  while (canonicalized.s > getLaneletLength(canonicalized.lanelet_id)) {
    bool next_lanelet_found = false;
    // When canonicalizing to forward lanelet_id, consider route
    for (const auto id : getNextLaneletIds(canonicalized.lanelet_id)) {
      if (std::any_of(route_lanelets.begin(), route_lanelets.end(), [id](auto id_on_route) {
            return id == id_on_route;
          })) {
        canonicalized.s -= getLaneletLength(canonicalized.lanelet_id);
        canonicalized.lanelet_id = id;
        next_lanelet_found = true;
      }
    }
    if (!next_lanelet_found) {
      return {std::nullopt, canonicalized.lanelet_id};
    }
  }
  return {canonicalized, std::nullopt};
}

auto getRoute(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id, bool allow_lane_change)
  -> lanelet::Ids
{
  if (Memory::getInstance().route_cache_.exists(
        from_lanelet_id, to_lanelet_id, allow_lane_change)) {
    return Memory::getInstance().route_cache_.getRoute(
      from_lanelet_id, to_lanelet_id, allow_lane_change);
  }
  lanelet::Ids ids;
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    Memory::getInstance().vehicle_routing_graph_ptr_->getRoute(
      lanelet, to_lanelet, 0, allow_lane_change);
  if (!route) {
    Memory::getInstance().route_cache_.appendData(
      from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    Memory::getInstance().route_cache_.appendData(
      from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ids.push_back(lane_itr->id());
  }
  Memory::getInstance().route_cache_.appendData(
    from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
  return ids;
}

auto getLaneletLength(const lanelet::Id lanelet_id) -> double
{
  if (Memory::getInstance().lanelet_length_cache_.exists(lanelet_id)) {
    return Memory::getInstance().lanelet_length_cache_.getLength(lanelet_id);
  }
  double ret = lanelet::utils::getLaneletLength2d(
    Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id));
  Memory::getInstance().lanelet_length_cache_.appendData(lanelet_id, ret);
  return ret;
}

////////////////////////////////////////
/////////////////////////////////////// others
/////////////////////////////////////

auto getFollowingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids, const double distance,
  const bool include_self) -> lanelet::Ids
{
  if (candidate_lanelet_ids.empty()) {
    return {};
  }
  lanelet::Ids ids;
  double total_distance = 0.0;
  bool found = false;
  for (const auto id : candidate_lanelet_ids) {
    if (found) {
      ids.emplace_back(id);
      total_distance = total_distance + getLaneletLength(id);
      if (total_distance > distance) {
        return ids;
      }
    }
    if (id == lanelet_id) {
      found = true;
      if (include_self) {
        ids.push_back(id);
      }
    }
  }
  if (!found) {
    THROW_SEMANTIC_ERROR("lanelet id does not match");
  }
  if (total_distance > distance) {
    return ids;
  }
  // clang-format off
  return ids + getFollowingLanelets(
    candidate_lanelet_ids[candidate_lanelet_ids.size() - 1],
    distance - total_distance, false);
  // clang-format on
}

auto getFollowingLanelets(
  const lanelet::Id lanelet_id, const double distance, const bool include_self) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  if (include_self) {
    ret.push_back(lanelet_id);
  }
  lanelet::Id end_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_ids = getNextLaneletIds(end_lanelet_id, "straight");
        !straight_ids.empty()) {
      total_distance = total_distance + getLaneletLength(straight_ids[0]);
      ret.push_back(straight_ids[0]);
      end_lanelet_id = straight_ids[0];
      continue;
    } else if (const auto ids = getNextLaneletIds(end_lanelet_id); ids.size() != 0) {
      total_distance = total_distance + getLaneletLength(ids[0]);
      ret.push_back(ids[0]);
      end_lanelet_id = ids[0];
      continue;
    } else {
      break;
    }
  }
  return ret;
}

auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool
{
  return std::find_if(route.begin(), route.end(), [lanelet_id](const auto id) {
           return lanelet_id == id;
         }) != route.end();
}

auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids
{
  assert(isTrafficLight(traffic_light_way_id));
  lanelet::Ids traffic_light_regulatory_element_ids;
  for (const auto & regulatory_element :
       Memory::getInstance().lanelet_map_ptr_->regulatoryElementLayer) {
    if (regulatory_element->attribute(lanelet::AttributeName::Subtype).value() == "traffic_light") {
      for (const auto & ref_member :
           regulatory_element->getParameters<lanelet::ConstLineString3d>("refers")) {
        if (ref_member.id() == traffic_light_way_id) {
          traffic_light_regulatory_element_ids.push_back(regulatory_element->id());
        }
      }
    }
  }
  return traffic_light_regulatory_element_ids;
}

auto getLateralDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change)
  -> std::optional<double>
{
  const auto route = getRoute(from.lanelet_id, to.lanelet_id, allow_lane_change);
  if (route.empty()) {
    return std::nullopt;
  }
  if (allow_lane_change) {
    double lateral_distance_by_lane_change = 0.0;
    for (unsigned int i = 0; i < route.size() - 1; i++) {
      auto next_lanelet_ids = getNextLaneletIds(route[i]);
      if (auto next_lanelet = std::find_if(
            next_lanelet_ids.begin(), next_lanelet_ids.end(),
            [&route, i](const lanelet::Id & id) { return id == route[i + 1]; });
          next_lanelet == next_lanelet_ids.end()) {
        traffic_simulator_msgs::msg::LaneletPose next_lanelet_pose;
        next_lanelet_pose.lanelet_id = route[i + 1];
        next_lanelet_pose.s = 0.0;
        next_lanelet_pose.offset = 0.0;

        if (
          auto next_lanelet_origin_from_current_lanelet =
            toLaneletPose(toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
          lateral_distance_by_lane_change += next_lanelet_origin_from_current_lanelet->offset;
        } else {
          traffic_simulator_msgs::msg::LaneletPose current_lanelet_pose = next_lanelet_pose;
          current_lanelet_pose.lanelet_id = route[i];
          if (
            auto current_lanelet_origin_from_next_lanelet =
              toLaneletPose(toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
            lateral_distance_by_lane_change -= current_lanelet_origin_from_next_lanelet->offset;
          } else {
            return std::nullopt;
          }
        }
      }
    }
    return to.offset - from.offset + lateral_distance_by_lane_change;
  } else {
    return to.offset - from.offset;
  }
}

auto getLongitudinalDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change)
  -> std::optional<double>
{
  if (from.lanelet_id == to.lanelet_id) {
    if (from.s > to.s) {
      return std::nullopt;
    } else {
      return to.s - from.s;
    }
  }
  const auto route = getRoute(from.lanelet_id, to.lanelet_id, allow_lane_change);
  if (route.empty()) {
    return std::nullopt;
  }
  double distance = 0;

  auto with_lane_change = [](
                            const bool allow_lane_change, const lanelet::Id current_lanelet,
                            const lanelet::Id next_lanelet) -> bool {
    if (allow_lane_change) {
      auto next_lanelet_ids = getNextLaneletIds(current_lanelet);
      auto next_lanelet_itr = std::find_if(
        next_lanelet_ids.begin(), next_lanelet_ids.end(),
        [next_lanelet](const lanelet::Id & id) { return id == next_lanelet; });
      return next_lanelet_itr == next_lanelet_ids.end();
    } else {
      return false;
    }
  };

  /// @note in this for loop, some cases are marked by @note command. each case is explained in the document.
  /// @sa https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/DistanceCalculation/
  for (unsigned int i = 0; i < route.size(); i++) {
    if (i < route.size() - 1 && with_lane_change(allow_lane_change, route[i], route[i + 1])) {
      /// @note "the lanelet before the lane change" case
      traffic_simulator_msgs::msg::LaneletPose next_lanelet_pose;
      next_lanelet_pose.lanelet_id = route[i + 1];
      next_lanelet_pose.s = 0.0;
      next_lanelet_pose.offset = 0.0;

      if (
        auto next_lanelet_origin_from_current_lanelet =
          toLaneletPose(toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
        distance += next_lanelet_origin_from_current_lanelet->s;
      } else {
        traffic_simulator_msgs::msg::LaneletPose current_lanelet_pose = next_lanelet_pose;
        current_lanelet_pose.lanelet_id = route[i];
        if (
          auto current_lanelet_origin_from_next_lanelet =
            toLaneletPose(toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
          distance -= current_lanelet_origin_from_next_lanelet->s;
        } else {
          return std::nullopt;
        }
      }

      /// @note "first lanelet before the lane change" case
      if (route[i] == from.lanelet_id) {
        distance += getLaneletLength(route[i + 1]) - from.s;
        if (route[i + 1] == to.lanelet_id) {
          distance -= getLaneletLength(route[i + 1]) - to.s;
          return distance;
        }
      }
    } else {
      if (route[i] == from.lanelet_id) {
        /// @note "first lanelet" case
        distance = getLaneletLength(from.lanelet_id) - from.s;
      } else if (route[i] == to.lanelet_id) {
        /// @note "last lanelet" case
        distance += to.s;
      } else {
        ///@note "normal intermediate lanelet" case
        distance += getLaneletLength(route[i]);
      }
    }
  }
  return distance;
}

auto getLeftBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  return toPolygon(
    Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id).leftBound());
}

auto getRightBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  return toPolygon(
    Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id).rightBound());
}

auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  lanelet::CompoundPolygon3d lanelet_polygon =
    Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id).polygon3d();
  for (const auto & lanelet_point : lanelet_polygon) {
    geometry_msgs::msg::Point p;
    p.x = lanelet_point.x();
    p.y = lanelet_point.y();
    p.z = lanelet_point.z();
    points.emplace_back(p);
  }
  return points;
}

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto stop_line = Memory::getInstance().lanelet_map_ptr_->lineStringLayer.get(lanelet_id);
  for (const auto & point : stop_line) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.emplace_back(p);
  }
  return points;
}

auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose, const bool fill_pitch)
  -> geometry_msgs::msg::PoseStamped
{
  if (
    const auto pose = std::get<std::optional<traffic_simulator_msgs::msg::LaneletPose>>(
      canonicalizeLaneletPose(lanelet_pose))) {
    geometry_msgs::msg::PoseStamped ret;
    ret.header.frame_id = "map";
    const auto spline = getCenterPointsSpline(pose->lanelet_id);
    ret.pose = spline->getPose(pose->s);
    const auto normal_vec = spline->getNormalVector(pose->s);
    const auto diff = math::geometry::normalize(normal_vec) * pose->offset;
    ret.pose.position = ret.pose.position + diff;
    const auto tangent_vec = spline->getTangentVector(pose->s);
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = fill_pitch ? std::atan2(-tangent_vec.z, std::hypot(tangent_vec.x, tangent_vec.y)) : 0.0;
    rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
    ret.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy) *
                           quaternion_operation::convertEulerAngleToQuaternion(pose->rpy);
    return ret;
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z, ") is invalid, please check lanelet length and connection.");
  }
}

auto toPolygon(const lanelet::ConstLineString3d & line_string)
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (const auto & p : line_string) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    ret.emplace_back(point);
  }
  return ret;
}

auto getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction,
  const std::uint8_t shift) -> std::optional<lanelet::Id>
{
  if (shift == 0) {
    return getLaneChangeableLaneletId(
      lanelet_id, traffic_simulator::lane_change::Direction::STRAIGHT);
  } else {
    auto reference_id = lanelet_id;
    for (uint8_t i = 0; i < shift; i++) {
      auto id = getLaneChangeableLaneletId(reference_id, direction);
      if (!id) {
        return std::nullopt;
      } else {
        reference_id = id.value();
      }
      if (i == (shift - 1)) {
        return reference_id;
      }
    }
  }
  return std::nullopt;
}

auto getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction)
  -> std::optional<lanelet::Id>
{
  const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  std::optional<lanelet::Id> target = std::nullopt;
  switch (direction) {
    case traffic_simulator::lane_change::Direction::STRAIGHT:
      target = lanelet.id();
      break;
    case traffic_simulator::lane_change::Direction::LEFT:
      if (Memory::getInstance().vehicle_routing_graph_ptr_->left(lanelet)) {
        target = Memory::getInstance().vehicle_routing_graph_ptr_->left(lanelet)->id();
      }
      break;
    case traffic_simulator::lane_change::Direction::RIGHT:
      if (Memory::getInstance().vehicle_routing_graph_ptr_->right(lanelet)) {
        target = Memory::getInstance().vehicle_routing_graph_ptr_->right(lanelet)->id();
      }
      break;
  }
  return target;
}

auto generateMarker() -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray markers;
  lanelet::ConstLanelets all_lanelets =
    lanelet::utils::query::laneletLayer(Memory::getInstance().lanelet_map_ptr_);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  lanelet::ConstLineStrings3d stop_lines = lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(Memory::getInstance().lanelet_map_ptr_);
  lanelet::ConstPolygons3d parking_lots =
    lanelet::utils::query::getAllParkingLots(Memory::getInstance().lanelet_map_ptr_);

  auto cl_ll_borders = color_utils::fromRgba(1.0, 1.0, 1.0, 0.999);
  auto cl_road = color_utils::fromRgba(0.2, 0.7, 0.7, 0.3);
  auto cl_cross = color_utils::fromRgba(0.2, 0.7, 0.2, 0.3);
  auto cl_stoplines = color_utils::fromRgba(1.0, 0.0, 0.0, 0.5);
  auto cl_trafficlights = color_utils::fromRgba(0.7, 0.7, 0.7, 0.8);
  auto cl_detection_areas = color_utils::fromRgba(0.7, 0.7, 0.7, 0.3);
  auto cl_parking_lots = color_utils::fromRgba(0.7, 0.7, 0.0, 0.3);
  auto cl_parking_spaces = color_utils::fromRgba(1.0, 0.647, 0.0, 0.6);
  auto cl_lanelet_id = color_utils::fromRgba(0.8, 0.2, 0.2, 0.999);

  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(road_lanelets, cl_ll_borders, true));
  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(markers, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    markers,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.1));
  insertMarkerArray(
    markers,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    markers, lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    markers, lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    markers, lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(crosswalk_lanelets, cl_lanelet_id));
  return markers;
}

auto insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2)
  -> void
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

////////////////////////////////////////
/////////////////////////////////////// bt
/////////////////////////////////////

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>
{
  if (waypoints.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  if (waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(waypoints);
  const auto stop_lines = getStopLinesOnPath({route_lanelets});
  for (const auto & stop_line : stop_lines) {
    std::vector<geometry_msgs::msg::Point> stop_line_points;
    for (const auto & point : stop_line) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      stop_line_points.emplace_back(p);
    }
    const auto collision_point = spline.getCollisionPointIn2D(stop_line_points);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>
{
  if (spline.getLength() <= 0) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  const auto stop_lines = getStopLinesOnPath({route_lanelets});
  for (const auto & stop_line : stop_lines) {
    std::vector<geometry_msgs::msg::Point> stop_line_points;
    for (const auto & point : stop_line) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      stop_line_points.emplace_back(p);
    }
    const auto collision_point = spline.getCollisionPointIn2D(stop_line_points);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getSpeedLimit(const lanelet::Ids & lanelet_ids) -> double
{
  std::vector<double> limits;
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  }
  for (auto itr = lanelet_ids.begin(); itr != lanelet_ids.end(); itr++) {
    const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(*itr);
    const auto limit = Memory::getInstance().traffic_rules_vehicle_ptr_->speedLimit(lanelet);
    limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
  }
  return *std::min_element(limits.begin(), limits.end());
}

auto canChangeLane(const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id) -> bool
{
  const auto from_lanelet =
    Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(to_lanelet_id);
  return Memory::getInstance().traffic_rules_vehicle_ptr_->canChangeLane(from_lanelet, to_lanelet);
}

auto getAlongLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose, const double along)
  -> traffic_simulator_msgs::msg::LaneletPose
{
  traffic_simulator_msgs::msg::LaneletPose along_pose = from_pose;
  along_pose.s = along_pose.s + along;
  if (along_pose.s >= 0) {
    while (along_pose.s >= getLaneletLength(along_pose.lanelet_id)) {
      auto next_ids = getNextLaneletIds(along_pose.lanelet_id, "straight");
      if (next_ids.empty()) {
        next_ids = getNextLaneletIds(along_pose.lanelet_id);
        if (next_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + along, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s - getLaneletLength(along_pose.lanelet_id);
      along_pose.lanelet_id = next_ids[0];
    }
  } else {
    while (along_pose.s < 0) {
      auto previous_ids = getPreviousLaneletIds(along_pose.lanelet_id, "straight");
      if (previous_ids.empty()) {
        previous_ids = getPreviousLaneletIds(along_pose.lanelet_id);
        if (previous_ids.empty()) {
          THROW_SEMANTIC_ERROR(
            "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
            from_pose.s + along, "), next lanelet of id = ", along_pose.lanelet_id, "is empty.");
        }
      }
      along_pose.s = along_pose.s + getLaneletLength(previous_ids[0]);
      along_pose.lanelet_id = previous_ids[0];
    }
  }
  return along_pose;
}

auto getStopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d
{
  lanelet::ConstLineStrings3d ret;
  for (const auto & traffic_sign : getTrafficSignRegulatoryElementsOnPath(lanelet_ids)) {
    if (traffic_sign->type() == "stop_sign") {
      for (const auto & stop_line : traffic_sign->refLines()) {
        ret.emplace_back(stop_line);
      }
    }
  }
  return ret;
}

auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.emplace_back(traffic_sign);
    }
  }
  return ret;
}

auto getPreviousLanelets(const lanelet::Id lanelet_id, const double distance) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  ret.push_back(lanelet_id);
  while (total_distance < distance) {
    auto ids = getPreviousLaneletIds(lanelet_id, "straight");
    if (ids.size() != 0) {
      total_distance = total_distance + getLaneletLength(ids[0]);
      ret.push_back(ids[0]);
      continue;
    } else {
      auto else_ids = getPreviousLaneletIds(lanelet_id);
      if (else_ids.size() != 0) {
        total_distance = total_distance + getLaneletLength(else_ids[0]);
        ret.push_back(else_ids[0]);
        continue;
      } else {
        break;
      }
    }
  }
  return ret;
}

auto getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>
{
  std::unordered_map<lanelet::Id, lanelet::Ids> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    ret.emplace(lanelet_id, getRightOfWayLaneletIds(lanelet_id));
  }
  return ret;
}

auto getRightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & right_of_way : Memory::getInstance()
                                     .lanelet_map_ptr_->laneletLayer.get(lanelet_id)
                                     .regulatoryElementsAs<lanelet::RightOfWay>()) {
    for (const auto & ll : right_of_way->rightOfWayLanelets()) {
      if (lanelet_id != ll.id()) {
        ids.push_back(ll.id());
      }
    }
  }
  return ids;
}

auto getLaneChangeTrajectory(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double longitudinal_distance =
    traffic_simulator::lane_change::Parameter::default_lanechange_distance;
  switch (lane_change_parameter.constraint.type) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      longitudinal_distance = lane_change_parameter.constraint.value;
      break;
    case traffic_simulator::lane_change::Constraint::Type::TIME:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
  }
  const auto along_pose = getAlongLaneletPose(from_pose, longitudinal_distance);
  // clang-format off
  const auto left_point =
    toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset + 5.0)).pose.position;
  const auto right_point =
    toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset - 5.0)).pose.position;
  // clang-format on
  const auto collision_point = getCenterPointsSpline(lane_change_parameter.target.lanelet_id)
                                 ->getCollisionPointIn2D(left_point, right_point);
  if (!collision_point) {
    return std::nullopt;
  }
  const auto to_pose = traffic_simulator::helper::constructLaneletPose(
    lane_change_parameter.target.lanelet_id, collision_point.value(),
    lane_change_parameter.target.offset);
  const auto goal_pose_in_map = toMapPose(to_pose).pose;
  const auto from_pose_in_map = toMapPose(from_pose).pose;
  double start_to_goal_distance = std::sqrt(
    std::pow(from_pose_in_map.position.x - goal_pose_in_map.position.x, 2) +
    std::pow(from_pose_in_map.position.y - goal_pose_in_map.position.y, 2) +
    std::pow(from_pose_in_map.position.z - goal_pose_in_map.position.z, 2));

  auto traj = getLaneChangeTrajectory(
    toMapPose(from_pose).pose, to_pose, lane_change_parameter.trajectory_shape,
    start_to_goal_distance * 0.5);
  return std::make_pair(traj, collision_point.value());
}

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double to_length = getLaneletLength(lane_change_parameter.target.lanelet_id);
  std::vector<double> evaluation, target_s;
  std::vector<math::geometry::HermiteCurve> curves;

  for (double to_s = 0; to_s < to_length; to_s = to_s + 1.0) {
    auto goal_pose = toMapPose(traffic_simulator::helper::constructLaneletPose(
      lane_change_parameter.target.lanelet_id, to_s));
    if (
      math::geometry::getRelativePose(from_pose, goal_pose.pose).position.x <=
      forward_distance_threshold) {
      continue;
    }
    double start_to_goal_distance = std::sqrt(
      std::pow(from_pose.position.x - goal_pose.pose.position.x, 2) +
      std::pow(from_pose.position.y - goal_pose.pose.position.y, 2) +
      std::pow(from_pose.position.z - goal_pose.pose.position.z, 2));
    traffic_simulator_msgs::msg::LaneletPose to_pose;
    to_pose.lanelet_id = lane_change_parameter.target.lanelet_id;
    to_pose.s = to_s;
    auto traj = getLaneChangeTrajectory(
      from_pose, to_pose, lane_change_parameter.trajectory_shape, start_to_goal_distance * 0.5);
    if (traj.getMaximum2DCurvature() < maximum_curvature_threshold) {
      double eval = std::fabs(target_trajectory_length - traj.getLength());
      evaluation.push_back(eval);
      curves.push_back(traj);
      target_s.push_back(to_s);
    }
  }
  if (evaluation.empty()) {
    return std::nullopt;
  }
  std::vector<double>::iterator min_itr = std::min_element(evaluation.begin(), evaluation.end());
  size_t min_index = std::distance(evaluation.begin(), min_itr);
  return std::make_pair(curves[min_index], target_s[min_index]);
}

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const double tangent_vector_size) -> math::geometry::HermiteCurve
{
  geometry_msgs::msg::Vector3 start_vec;
  geometry_msgs::msg::Vector3 to_vec;
  geometry_msgs::msg::Pose goal_pose = toMapPose(to_pose).pose;
  double tangent_vector_size_in_curve = 0.0;
  switch (trajectory_shape) {
    case traffic_simulator::lane_change::TrajectoryShape::CUBIC:
      start_vec = getVectorFromPose(from_pose, tangent_vector_size);
      if (getTangentVector(to_pose.lanelet_id, to_pose.s)) {
        to_vec = getTangentVector(to_pose.lanelet_id, to_pose.s).value();
      } else {
        THROW_SIMULATION_ERROR(
          "Failed to calculate tangent vector at lanelet_id : ", to_pose.lanelet_id,
          " s : ", to_pose.s);
      }
      tangent_vector_size_in_curve = tangent_vector_size;
      break;
    case traffic_simulator::lane_change::TrajectoryShape::LINEAR:
      start_vec.x = (goal_pose.position.x - from_pose.position.x);
      start_vec.y = (goal_pose.position.y - from_pose.position.y);
      start_vec.z = (goal_pose.position.z - from_pose.position.z);
      to_vec = start_vec;
      tangent_vector_size_in_curve = 1;
      break;
  }
  return math::geometry::HermiteCurve(
    from_pose, goal_pose, start_vec,
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(to_vec.x * tangent_vector_size_in_curve)
      .y(to_vec.y * tangent_vector_size_in_curve)
      .z(to_vec.z * tangent_vector_size_in_curve));
}

auto getTangentVector(const lanelet::Id lanelet_id, const double s)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  return getCenterPointsSpline(lanelet_id)->getTangentVector(s);
}

auto getVectorFromPose(const geometry_msgs::msg::Pose & pose, const double magnitude)
  -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 dir =
    quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
  geometry_msgs::msg::Vector3 vector;
  vector.x = magnitude * std::cos(dir.z);
  vector.y = magnitude * std::sin(dir.z);
  vector.z = 0;
  return vector;
}

auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : getTrafficLightRegulatoryElementsOnPath(route_lanelets)) {
    for (auto light_string : traffic_light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        if (auto id = light_string.attribute("traffic_light_id").asId(); id) {
          ids.push_back(id.value());
        }
      }
    }
  }
  return ids;
}

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>
{
  auto traffic_light_ids = getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(waypoints, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>
{
  auto traffic_light_ids = getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(spline, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToTrafficLightStopLine(
  const std::vector<geometry_msgs::msg::Point> & waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>
{
  if (waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(waypoints);
  const auto stop_lines = getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

auto getDistanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>
{
  if (spline.getLength() <= 0) {
    return std::nullopt;
  }
  const auto stop_lines = getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

auto getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::vector<lanelet::routing::RoutingGraphConstPtr> graphs;
  graphs.emplace_back(Memory::getInstance().vehicle_routing_graph_ptr_);
  graphs.emplace_back(Memory::getInstance().pedestrian_routing_graph_ptr_);
  lanelet::routing::RoutingGraphContainer container(graphs);
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    double height_clearance = 4;
    size_t routing_graph_id = 1;
    const auto conflicting_crosswalks =
      container.conflictingInGraph(lanelet, routing_graph_id, height_clearance);
    for (const auto & crosswalk : conflicting_crosswalks) {
      ids.emplace_back(crosswalk.id());
    }
  }
  return ids;
}

auto getConflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto conflicting_lanelets = lanelet::utils::getConflictingLanelets(
      Memory::getInstance().vehicle_routing_graph_ptr_, lanelet);
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      ids.emplace_back(conflicting_lanelet.id());
    }
  }
  return ids;
}

auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<geometry_msgs::msg::Point>>
{
  std::vector<std::vector<geometry_msgs::msg::Point>> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    ret.emplace_back(std::vector<geometry_msgs::msg::Point>{});
    const auto stop_line = traffic_light->stopLine();
    if (stop_line) {
      auto & current_stop_line = ret.back();
      for (const auto & point : stop_line.value()) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        current_stop_line.emplace_back(p);
      }
    }
  }
  return ret;
}

auto getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
{
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::getInstance().lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_lights =
      lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      ret.emplace_back(traffic_light);
    }
  }
  return ret;
}

auto getTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> ret;
  lanelet::ConstLanelets all_lanelets =
    lanelet::utils::query::laneletLayer(Memory::getInstance().lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto & light : autoware_traffic_lights) {
    for (auto light_string : light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id == traffic_light_id) {
          ret.emplace_back(light);
        }
      }
    }
  }
  if (ret.empty()) {
    THROW_SEMANTIC_ERROR("traffic_light_id does not match. ID : ", traffic_light_id);
  }
  return ret;
}
}  // namespace lanelet2
}  // namespace traffic_simulator