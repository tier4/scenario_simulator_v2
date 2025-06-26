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

#include <lanelet2_core/geometry/Lanelet.h>

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_loader.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
auto LaneletLoader::load(const std::filesystem::path & lanelet_map_path) -> lanelet::LaneletMapPtr
{
  lanelet::projection::MGRSProjector mgrs_projector;
  lanelet::ErrorMessages lanelet_errors;
  lanelet::LaneletMapPtr lanelet_map_ptr =
    lanelet::load(lanelet_map_path.string(), mgrs_projector, &lanelet_errors);

  if (!lanelet_errors.empty()) {
    std::stringstream ss;
    ss << "Failed to load lanelet map, errors:\n";
    for (const auto & error : lanelet_errors) {
      ss << " - " << error << "\n";
    }
    THROW_SIMULATION_ERROR(ss.str());
  }

  if (lanelet_map_ptr->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("Lanelet layer is empty!");
  }

  overwriteLaneletsCenterline(lanelet_map_ptr);
  return lanelet_map_ptr;
}

auto LaneletLoader::overwriteLaneletsCenterline(lanelet::LaneletMapPtr lanelet_map_ptr) -> void
{
  constexpr double resolution{2.0};

  auto generateFineCenterline =
    [&](const lanelet::ConstLanelet & lanelet_obj) -> lanelet::LineString3d {
    /// @note Get length of longer border
    const auto left_length =
      static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
    const auto right_length =
      static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
    const auto longer_distance = (left_length > right_length) ? left_length : right_length;
    const auto num_segments = std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

    /// @note Resample points
    const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
    const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

    /// @note Create centerline
    lanelet::LineString3d centerline(lanelet::utils::getId());
    for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++) {
      /// @note Add ID for the average point of left and right
      const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
      const lanelet::Point3d center_point(
        lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
        center_basic_point.z());
      centerline.push_back(center_point);
    }
    return centerline;
  };
  for (auto & lanelet_obj : lanelet_map_ptr->laneletLayer) {
    if (lanelet_obj.hasCustomCenterline()) {
      THROW_SIMULATION_ERROR("Detected custom centerline in lanelet map");
    }
    lanelet_obj.setCenterline(generateFineCenterline(lanelet_obj));
  }
}

auto LaneletLoader::resamplePoints(
  const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
  -> lanelet::BasicPoints3d
{
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  auto findNearestIndexPair =
    [&](const double target_length) -> std::pair<std::size_t, std::size_t> {
    const auto N = accumulated_lengths.size();
    if (target_length < accumulated_lengths.at(1)) {
      return std::make_pair(0, 1);
    } else if (target_length > accumulated_lengths.at(N - 2)) {
      return std::make_pair(N - 2, N - 1);
    } else {
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

  /// @note Create each segment
  lanelet::BasicPoints3d resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    /// @note Find two nearest points
    const double target_length = (static_cast<double>(i) / num_segments) *
                                 static_cast<double>(lanelet::geometry::length(line_string));
    const auto [first_index, second_index] = findNearestIndexPair(target_length);

    /// @note Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[first_index];
    const lanelet::BasicPoint3d front_point = line_string[second_index];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(first_index);
    const auto front_length = accumulated_lengths.at(second_index);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    /// @note Add to list
    resampled_points.emplace_back(target_point);
  }
  return resampled_points;
}

auto LaneletLoader::calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
  -> std::vector<double>
{
  auto calculateSegmentDistances =
    [](const lanelet::ConstLineString3d & line) -> std::vector<double> {
    std::vector<double> segment_distances;
    segment_distances.reserve(line.size() - 1);
    for (size_t i = 1; i < line.size(); ++i) {
      segment_distances.push_back(lanelet::geometry::distance(line[i], line[i - 1]));
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
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
