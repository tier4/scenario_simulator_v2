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

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_generator.hpp>

namespace simple_sensor_simulator
{
OccupancyGridGenerator::OccupancyGridGenerator(
  const simulation_api_schema::OccupancyGridSensorConfiguration & configuration)
: configuration(configuration),
  grid_(configuration.resolution(), configuration.height(), configuration.width())
{
}

nav_msgs::msg::OccupancyGrid OccupancyGridGenerator::generate(
  const geometry_msgs::msg::Pose & ego_pose, const rclcpp::Time & stamp) const
{
  grid_.updateOrigin(ego_pose);
  for (const auto & primitive : primitive_ptrs_) {
    grid_.addPrimitive(primitive.second);
  }

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = stamp;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.data = grid_.getData();
  occupancy_grid.info.height = configuration.height();
  occupancy_grid.info.width = configuration.width();
  occupancy_grid.info.map_load_time = stamp;
  occupancy_grid.info.resolution = configuration.resolution();
  occupancy_grid.info.origin = ego_pose;
  occupancy_grid.info.origin.position.x = occupancy_grid.info.origin.position.x -
                                          0.5 * configuration.height() * configuration.resolution();
  occupancy_grid.info.origin.position.y = occupancy_grid.info.origin.position.y -
                                          0.5 * configuration.width() * configuration.resolution();
  return occupancy_grid;
}
}  // namespace simple_sensor_simulator
