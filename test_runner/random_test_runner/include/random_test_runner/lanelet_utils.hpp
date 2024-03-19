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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__LANELET_UTILS_HPP
#define RANDOM_TEST_RUNNER__LANELET_UTILS_HPP

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <boost/filesystem.hpp>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "random_test_runner/data_types.hpp"

namespace hdmap_utils
{
class HdMapUtils;
}

struct LaneletPart
{
  int64_t lanelet_id;
  double start_s;
  double end_s;
};

class LaneletUtils
{
public:
  LaneletUtils(const boost::filesystem::path & filename);

  LaneletUtils() = delete;
  LaneletUtils(const LaneletUtils &) = delete;
  LaneletUtils(LaneletUtils &&) = delete;
  LaneletUtils & operator=(const LaneletUtils &) = delete;
  LaneletUtils && operator=(LaneletUtils &&) = delete;

  double computeDistance(
    const traffic_simulator_msgs::msg::LaneletPose & p1,
    const traffic_simulator_msgs::msg::LaneletPose & p2);
  std::optional<traffic_simulator_msgs::msg::LaneletPose> getOppositeLaneLet(
    const traffic_simulator_msgs::msg::LaneletPose & pose);
  std::vector<LaneletPart> getLanesWithinDistance(
    const traffic_simulator_msgs::msg::LaneletPose & pose, double min_distance,
    double max_distance);

  std::vector<int64_t> getLaneletIds();
  geometry_msgs::msg::PoseStamped toMapPose(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose, const bool fill_pitch);
  std::vector<int64_t> getRoute(int64_t from_lanelet_id, int64_t to_lanelet_id);
  double getLaneletLength(int64_t lanelet_id);
  bool isInLanelet(int64_t lanelet_id, double s);

private:
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
};

#endif  // RANDOM_TEST_RUNNER__LANELET_UTILS_HPP
