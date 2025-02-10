// Copyright 2022 TIER IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <context_gamma_scenarios/catalogs.hpp>
#include <context_gamma_scenarios/context_gamma_scenario_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/api.hpp>

// headers in STL
#include <memory>
#include <string>
#include <vector>

class PedestrianFollowLane : public context_gamma_scenarios::ContextGammaScenarioNode
{
public:
  explicit PedestrianFollowLane(const rclcpp::NodeOptions & option)
  : context_gamma_scenarios::ContextGammaScenarioNode(
      "pedestrian_follow_lane",
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map", "lanelet2_map.osm",
      __FILE__, false, option)
  {
    start();
  }

private:
  int step_ = 0;
  const std::vector<int64_t> checkpoint_ids_ = {35016, 35036, 34981};
  void onUpdate() override
  {
    const auto t = api_.getCurrentTime();
    if (
      static_cast<traffic_simulator::LaneletPose>(api_.getEntityStatus("ego").getLaneletPose())
        .lanelet_id == checkpoint_ids_.at(step_)) {
      step_++;
    }
    if (step_ == static_cast<int>(checkpoint_ids_.size())) {
      stop(context_gamma_scenarios::Result::SUCCESS);
    }

    // LCOV_EXCL_STOP
    if (t >= 100) {
      stop(context_gamma_scenarios::Result::FAILURE);
    }
  }

  void onInitialize() override
  {
    api_.spawn(
      "ego",
      traffic_simulator::helper::constructCanonicalizedLaneletPose(
        34426, 10.0, 0, api_.getHdmapUtils()),
      getPedestrianParameters(), traffic_simulator::PedestrianBehavior::contextGamma());
    api_.requestSpeedChange(
      "ego", 4.0, traffic_simulator::speed_change::Transition::LINEAR,
      traffic_simulator::speed_change::Constraint(
        traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 1.0),
      true);
    api_.requestAssignRoute(
      "ego", std::vector<traffic_simulator::CanonicalizedLaneletPose>{
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 35016, 0.0, 0, api_.getHdmapUtils()),
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 35026, 0.0, 0, api_.getHdmapUtils()),
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 35036, 0.0, 0, api_.getHdmapUtils()),
               traffic_simulator::helper::constructCanonicalizedLaneletPose(
                 34981, 1.0, 0, api_.getHdmapUtils())});
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<PedestrianFollowLane>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
