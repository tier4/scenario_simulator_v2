// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
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

  double initialize_duration = 0;

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
  Pathname map_path = "";

  Filename lanelet2_map_file =
    "lanelet2_map.osm";  // TODO (yamacir-kit): Use `static inline` (if C++17)

  Filename pointcloud_map_file =
    "pointcloud_map.pcd";  // TODO (yamacir-kit): Use `static inline` (if C++17)

  Pathname scenario_path = "";

  Pathname metrics_log_path = "/tmp/metrics.json";

  Pathname rviz_config_path =  //
    ament_index_cpp::get_package_share_directory("scenario_test_runner") +
    "/config/scenario_simulator_v2.rviz";

  auto lanelet2_map_path() const { return map_path / lanelet2_map_file; }

  auto pointcloud_map_path() const { return map_path / pointcloud_map_file; }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__CONFIGURATION_HPP_
