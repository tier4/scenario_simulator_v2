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

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
auto LaneletWrapper::activate(const std::string & lanelet_map_path) -> void
{
  lanelet_map_path_ = lanelet_map_path;
  if (instance) {
    std::lock_guard<std::mutex> lock(mutex_);
    instance.reset();
  }
}

LaneletWrapper & LaneletWrapper::getInstance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!instance) {
    if (!lanelet_map_path_.empty()) {
      instance.reset(new LaneletWrapper(lanelet_map_path_));
    } else {
      THROW_SIMULATION_ERROR(
        "lanelet_map_path is empty! Please call lanelet_map::activate() first.");
    }
  }
  return *instance;
}

auto LaneletWrapper::routeCache() -> RouteCache & { return getInstance().route_cache_; }

auto LaneletWrapper::centerPointsCache() -> CenterPointsCache &
{
  return getInstance().center_points_cache_;
}

auto LaneletWrapper::laneletLengthCache() -> LaneletLengthCache &
{
  return getInstance().lanelet_length_cache_;
}

auto LaneletWrapper::map() -> const lanelet::LaneletMapPtr &
{
  return getInstance().lanelet_map_ptr_;
}

auto LaneletWrapper::shoulderLanelets() -> const lanelet::ConstLanelets &
{
  return getInstance().shoulder_lanelets_;
}
auto LaneletWrapper::vehicleRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &
{
  return getInstance().vehicle_routing_graph_ptr_;
}
auto LaneletWrapper::pedestrianRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &
{
  return getInstance().pedestrian_routing_graph_ptr_;
}
auto LaneletWrapper::trafficRulesVehicle() -> const lanelet::traffic_rules::TrafficRulesPtr &
{
  return getInstance().traffic_rules_vehicle_ptr_;
}
auto LaneletWrapper::trafficRulesPedestrian() -> const lanelet::traffic_rules::TrafficRulesPtr &
{
  return getInstance().traffic_rules_pedestrian_ptr_;
}

LaneletWrapper::LaneletWrapper(const std::filesystem::path & lanelet_map_path)
{
  lanelet::projection::MGRSProjector mgrs_projector;
  lanelet::ErrorMessages lanelet_errors;
  lanelet_map_ptr_ = lanelet::load(lanelet_map_path.string(), mgrs_projector, &lanelet_errors);
  if (!lanelet_errors.empty()) {
    std::stringstream ss;
    ss << "Failed to load lanelet map, errors:\n";
    for (const auto & error : lanelet_errors) {
      ss << " - " << error << "\n";
    }
    THROW_SIMULATION_ERROR(ss.str());
  }

  if (lanelet_map_ptr_->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("Lanelet layer is empty!");
  }

  overwriteLaneletsCenterline();
  shoulder_lanelets_ =
    lanelet::utils::query::shoulderLanelets(lanelet::utils::query::laneletLayer(lanelet_map_ptr_));
  // Vehicle
  traffic_rules_vehicle_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle_routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_vehicle_ptr_);
  // Pedestrian
  traffic_rules_pedestrian_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  pedestrian_routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_pedestrian_ptr_);
}

auto LaneletWrapper::calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
  -> std::vector<double>
{
  auto calculateSegmentDistances =
    [](const lanelet::ConstLineString3d & line_string) -> std::vector<double> {
    std::vector<double> segment_distances;
    segment_distances.reserve(line_string.size() - 1);
    for (size_t i = 1; i < line_string.size(); ++i) {
      segment_distances.push_back(lanelet::geometry::distance(line_string[i], line_string[i - 1]));
    }
    return segment_distances;
  };

  const auto segment_distances = calculateSegmentDistances(line_string);
  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));
  return accumulated_lengths;
}

auto LaneletWrapper::resamplePoints(
  const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
  -> lanelet::BasicPoints3d
{
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  auto findNearestIndexPair =
    [&](const double target_length) -> std::pair<std::size_t, std::size_t> {
    const auto N = accumulated_lengths.size();
    if (target_length < accumulated_lengths.at(1)) {  // Front
      return std::make_pair(0, 1);
    } else if (target_length > accumulated_lengths.at(N - 2)) {  // Back
      return std::make_pair(N - 2, N - 1);
    } else  // Middle
    {
      for (size_t i = 1; i < N; ++i) {
        if (
          accumulated_lengths.at(i - 1) <= target_length &&
          target_length <= accumulated_lengths.at(i)) {
          return std::make_pair(i - 1, i);
        }
      }
    }
    THROW_SEMANTIC_ERROR("findNearestIndexPair(): No nearest point found.");
  };

  // Create each segment
  lanelet::BasicPoints3d resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const double target_length = (static_cast<double>(i) / num_segments) *
                                 static_cast<double>(lanelet::geometry::length(line_string));
    const auto index_pair = findNearestIndexPair(target_length);

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

auto LaneletWrapper::overwriteLaneletsCenterline() -> void
{
  constexpr double resolution{2.0};

  auto generateFineCenterline =
    [&](const lanelet::ConstLanelet & lanelet_obj) -> lanelet::LineString3d {
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
  };

  for (auto & lanelet_obj : lanelet_map_ptr_->laneletLayer) {
    if (!lanelet_obj.hasCustomCenterline()) {
      lanelet_obj.setCenterline(generateFineCenterline(lanelet_obj));
    }
  }
}
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
