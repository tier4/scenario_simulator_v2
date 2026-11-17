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
#ifndef TRAFFIC_SIMULATOR__LANELET_LOADER_HPP_
#define TRAFFIC_SIMULATOR__LANELET_LOADER_HPP_

#include <lanelet2_io/Io.h>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <filesystem>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
class LaneletLoader
{
public:
  static auto load(const std::filesystem::path & lanelet_map_path) -> lanelet::LaneletMapPtr;
  static auto convertMapToBin(const lanelet::LaneletMapPtr lanelet_map_ptr)
    -> autoware_map_msgs::msg::LaneletMapBin;

private:
  static auto overwriteLaneletsCenterline(lanelet::LaneletMapPtr) -> void;
  static auto resamplePoints(
    const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
    -> lanelet::BasicPoints3d;
  static auto calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
    -> std::vector<double>;
};
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_LOADER_HPP_
