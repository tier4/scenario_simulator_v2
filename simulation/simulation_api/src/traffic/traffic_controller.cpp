/**
 * @file traffic_controller.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implementation for the traffic controller
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) Tier IV.Inc {2015-2021}
 *
 */

#include <simulation_api/traffic/traffic_controller.hpp>
#include <simulation_api/traffic/traffic_sink.hpp>

namespace simulation_api
{
namespace traffic
{
TrafficController::TrafficController(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  const std::function<std::vector<std::string>(void)> & get_entity_names_function,
  const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
  const std::function<void(std::string)> & despawn_function,
  bool auto_sink)
: hdmap_utils_(hdmap_utils),
  auto_sink(auto_sink)
{
  if (auto_sink) {
    autoSink();
  }
}

void TrafficController::autoSink()
{
  for (const auto & lanelet_id : hdmap_utils_->getLaneletIds()) {
    if (hdmap_utils_->getNextLaneletIds(lanelet_id).empty()) {
      openscenario_msgs::msg::LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lanelet_id;
      lanelet_pose.s = hdmap_utils_->getLaneletLength(lanelet_id);
      const auto pose = hdmap_utils_->toMapPose(lanelet_pose);
      // addModule<simulation_api::traffic::TrafficSink>(5, pose.pose.position);
    }
  }
}

void TrafficController::execute()
{
  for (const auto & module : modules_) {
    module->execute();
  }
}
}  // namespace traffic
}  // namespace simulation_api
