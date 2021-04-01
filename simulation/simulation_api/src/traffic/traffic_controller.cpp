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

namespace simulation_api
{
namespace traffic
{
TrafficController::TrafficController(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
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

}

void TrafficController::execute()
{
  for (const auto & module : modules_) {
    module->execute();
  }
}
}  // namespace traffic
}  // namespace simulation_api
