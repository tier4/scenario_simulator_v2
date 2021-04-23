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

#ifndef TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_

#include <algorithm>
#include <awapi_accessor/autoware.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <cstdlib>
#include <future>
#include <memory>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <string>
#include <thread>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/vehicle_model/sim_model_time_delay.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#define DEBUG_VALUE(...) \
  std::cout << "\x1b[32m" #__VA_ARGS__ " = " << (__VA_ARGS__) << "\x1b[0m" << std::endl

#define DEBUG_LINE() \
  std::cout << "\x1b[32m" << __FILE__ << ":" << __LINE__ << "\x1b[0m" << std::endl

namespace traffic_simulator
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
  // NOTE: One day we will have to do simultaneous simulations of multiple Autowares.
  static std::unordered_map<std::string, awapi::Autoware> autowares;

  decltype(fork()) autoware_process_id = 0;

  bool autoware_initialized = false;

public:
  EgoEntity() = delete;

  EgoEntity(const EgoEntity &) = delete;

  EgoEntity & operator=(const EgoEntity &) = delete;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  This constructor builds an Ego-type entity with an ambiguous initial
   *  state. In this case, the values for status_ and current_kinematic_state_
   *  are boost::none, respectively.
   *
   *  This constructor is used for the purpose of delaying the transmission of
   *  the initial position from the entity's spawn. If you build an ego-type
   *  entity with this constructor, you must explicitly call setStatus at least
   *  once before the first onUpdate call to establish location and kinematic
   *  state.
   *
   *  For OpenSCENARIO, setStatus before the onUpdate call is called by
   *  TeleportAction in the Storyboard.Init section.
   *
   * ------------------------------------------------------------------------ */
  explicit EgoEntity(
    const std::string & name, const boost::filesystem::path & lanelet2_map_osm,
    const double step_time, const openscenario_msgs::msg::VehicleParameters & parameters);

  ~EgoEntity() override;

  auto getEntityTypename() const -> const std::string & override
  {
    static const std::string result = "EgoEntity";
    return result;
  }

  void plan(const std::vector<geometry_msgs::msg::PoseStamped> & route)
  {
    if (not std::exchange(autoware_initialized, true)) {
      autowares.at(name).initialize(getStatus().pose);
    }

    autowares.at(name).plan(route);
    autowares.at(name).engage();
  }

  void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose) override
  {
    plan({(*hdmap_utils_ptr_).toMapPose(lanelet_pose)});
  }

  void requestAssignRoute(
    const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints) override;

  void requestLaneChange(const std::int64_t to_lanelet_id);

  void setTargetSpeed(double value, bool) override
  {
    Eigen::VectorXd v(5);
    {
      v << 0, 0, 0, value, 0;
    }

    (*vehicle_model_ptr_).setState(v);
  }

  const std::string getCurrentAction() const override
  {
    return autowares.at(name).getAutowareStatus().autoware_state;
  }

  void onUpdate(double current_time, double step_time);

  bool setStatus(const openscenario_msgs::msg::EntityStatus & status) override;

  const openscenario_msgs::msg::WaypointsArray getWaypoints() override;

private:
  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle() override { return boost::none; }

  const openscenario_msgs::msg::EntityStatus getEntityStatus(
    const double time, const double step_time) const;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<geometry_msgs::msg::Pose> initial_pose_;
  boost::optional<double> previous_linear_velocity_;
  boost::optional<double> previous_angular_velocity_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
