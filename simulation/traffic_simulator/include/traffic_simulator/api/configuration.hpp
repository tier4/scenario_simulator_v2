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

#ifndef TRAFFIC_SIMULATOR__API__CONFIGURATION_HPP_
#define TRAFFIC_SIMULATOR__API__CONFIGURATION_HPP_

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/range/iterator_range.hpp>
#include <iomanip>
#include <scenario_simulator_exception/exception.hpp>
#include <string>

namespace traffic_simulator
{
struct Configuration
{
  using Filename = std::string;

  using Pathname = boost::filesystem::path;

  bool auto_sink = true;

  bool verbose = false;

  bool standalone_mode = false;

  std::string simulator_host = "localhost";

  double conventional_traffic_light_publish_rate = 30.0;

  double v2i_traffic_light_publish_rate = 10.0;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  This setting comes from the argument of the same name (= `map_path`) in
   *  the launch file of ArchitectureProposal. In general, Autoware expects
   *  this argument to be given the path to the directory containing the two HD
   *  maps, lanelet_map.osm and pointcloud_map.pcd.
   *
   *  Depending on your map file, you may need to include additional files in
   *  this directory. For example, Autoware.Auto (at the time this comment was
   *  written) should be given the map origin coordinates in a separate yaml
   *  file.
   *
   * ------------------------------------------------------------------------ */
  const Pathname map_path;

  Filename lanelet2_map_file;

  Filename pointcloud_map_file;

  Pathname scenario_path = "";

  explicit Configuration(const Pathname & map_path)  //
  : map_path(assertMapPath(map_path)),
    lanelet2_map_file(findLexicographicallyFirstFilenameOf(map_path, ".osm")),
    pointcloud_map_file(findLexicographicallyFirstFilenameOf(map_path, ".pcd"))
  {
  }

  auto assertMapPath(const Pathname & map_path) const -> const Pathname &
  {
    if (map_path.empty()) {
      throw common::SimulationError("No map path is given");
    } else if (not boost::filesystem::is_directory(map_path)) {
      throw common::SimulationError(
        "The map_path must be a directory (given an ", std::quoted(map_path.string()), ")");
    } else if (not contains(map_path, ".osm")) {
      throw common::SimulationError("The map_path must contain at least one *.osm file");
    } else if (not contains(map_path, ".pcd")) {
      throw common::SimulationError("The map_path must contain at least one *.pcd file");
    } else {
      return map_path;
    }
  }

  template <typename... Ts>
  auto contains(Ts &&... xs) const -> bool
  {
    return not findLexicographicallyFirstFilenameOf(std::forward<decltype(xs)>(xs)...).empty();
  }

  auto findLexicographicallyFirstFilenameOf(
    const Pathname & pathname, const std::string & extension) const -> Filename
  {
    Filename result;

    for (const auto & each :
         boost::make_iterator_range(boost::filesystem::directory_iterator(pathname), {})) {
      const auto filename = each.path().filename().string();
      if (
        each.path().extension() == extension and
        std::lexicographical_compare(
          std::cbegin(result), std::cend(result), std::cbegin(filename), std::cend(filename))) {
        result = filename;
      }
    }

    return result;
  }

  auto getLanelet2MapFile() const -> const auto &
  {
    if (not lanelet2_map_file.empty()) {
      return lanelet2_map_file;
    } else {
      throw common::SimulationError("The map_path must contain at least one *.osm file");
    }
  }

  auto getPointCloudMapFile() const -> const auto &
  {
    if (not pointcloud_map_file.empty()) {
      return pointcloud_map_file;
    } else {
      throw common::SimulationError("The map_path must contain at least one *.pcd file");
    }
  }

  auto lanelet2_map_path() const { return map_path / lanelet2_map_file; }

  auto pointcloud_map_path() const { return map_path / pointcloud_map_file; }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__CONFIGURATION_HPP_
