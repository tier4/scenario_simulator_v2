// Copyright 2021 Tier IV, Inc All rights reserved.
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

#include <context_gamma_planner/behavior/vehicle/follow_lane_action.hpp>
#include <context_gamma_planner/behavior/vehicle/follow_polyline_trajectory_action.hpp>
#include <context_gamma_planner/behavior/vehicle/lane_change_action.hpp>
#include <context_gamma_planner/vehicle_plugin.hpp>
#include <pugixml.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>

namespace context_gamma_planner
{
/**
 * @brief Configures the VehiclePlugin.
 * @details This function is responsible for configuring the VehiclePlugin.
 * @param logger The logger object used for logging.
 */
void VehiclePlugin::configure(const rclcpp::Logger & logger)
{
  std::cout << "vehicle plugin configure start" << std::endl;
  std::string path = ament_index_cpp::get_package_share_directory("context_gamma_planner") +
                     "/config/vehicle_behavior.xml";
  factory_.registerNodeType<context_gamma_planner::vehicle::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<context_gamma_planner::vehicle::LaneChangeAction>("LaneChange");
  factory_.registerNodeType<context_gamma_planner::vehicle::FollowPolylineTrajectoryAction>(
    "FollowPolylineTrajectory");
  tree_ = createBehaviorTree(path);
  logging_event_ptr_ =
    std::make_shared<context_gamma_planner::LoggingEvent>(tree_.rootNode(), logger);
  reset_request_event_ptr_ = std::make_shared<context_gamma_planner::ResetRequestEvent>(
    tree_.rootNode(), [&]() { return getRequest(); },
    [&](traffic_simulator::behavior::Request request) { return setRequest(request); });
  setRequest(traffic_simulator::behavior::Request::NONE);
  traffic_simulator_msgs::msg::Obstacle obstacle_msg;
  obstacle_msg.type = traffic_simulator_msgs::msg::Obstacle::ENTITY;
  setObstacle(std::nullopt);
  std::cout << "configure end" << std::endl;
}

/**
 * @brief Creates a behavior tree from an XML file.
 * @param format_path The path to the XML file.
 * @return The created behavior tree.
 */
auto VehiclePlugin::createBehaviorTree(const std::string & format_path) -> BT::Tree
{
  auto xml_doc = pugi::xml_document();
  xml_doc.load_file(format_path.c_str());

  class XMLTreeWalker : public pugi::xml_tree_walker
  {
  public:
    explicit XMLTreeWalker(const BT::TreeNodeManifest & manifest) : manifest_(manifest) {}

  private:
    bool for_each(pugi::xml_node & node) final
    {
      if (node.name() == manifest_.registration_ID) {
        for (const auto & [port, info] : manifest_.ports) {
          node.append_attribute(port.c_str()) = std::string("{" + port + "}").c_str();
        }
      }
      return true;
    }

    const BT::TreeNodeManifest & manifest_;
  };

  for (const auto & [id, manifest] : factory_.manifests()) {
    if (factory_.builtinNodes().count(id) == 0) {
      auto walker = XMLTreeWalker(manifest);
      xml_doc.traverse(walker);
    }
  }

  auto xml_str = std::stringstream();
  xml_doc.save(xml_str);
  return factory_.createTreeFromText(xml_str.str());
}

/**
 * @brief Get the current action of the vehicle.
 * @return const std::string& The current action.
 */
const std::string & VehiclePlugin::getCurrentAction() const
{
  return logging_event_ptr_->getCurrentAction();
}

/**
 * @brief Updates the vehicle plugin.
 * @param current_time The current time.
 * @param step_time The time step.
 */
void VehiclePlugin::update(double current_time, double step_time)
{
  tryInitializeConstraintActivator();
  tryInitializeEgoInRVO();

  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }

  if (getPlanningSpeed()) {
    rvo_ego_->setMaxSpeed(getPlanningSpeed().value());
  }

  rvo_simulator_.deleteObstacles();
  rvo_simulator_.addGlobalObstacles(activator_ptr_->calculateRVOObstacles());
  rvo_simulator_.processObstacles();

  auto ego_status =
    static_cast<traffic_simulator::EntityStatus>(*this->getCanonicalizedEntityStatus());
  reflectEgoInRVO(ego_status);
  reflectNonEgoEntitiesInRVO();
  rvo_simulator_.deleteUnavailableAgent();
  updateRVO(step_time);
  updateSimulatorStatus();
  visualize();

  /// @note cleanup constraints
  activator_ptr_->deactivateAllConstraints();
}

void VehiclePlugin::tryInitializeConstraintActivator()
{
  if (activator_ptr_ == nullptr) {
    activator_ptr_ =
      std::make_shared<context_gamma_planner::vehicle::constraints::ConstraintActivator>(
        getHdMapUtils(), getTrafficLights());
    setConstraintActivator(activator_ptr_);
  }
}

void VehiclePlugin::tryInitializeEgoInRVO()
{
  if (rvo_ego_ == nullptr) {
    std::cout << "set agent conf start" << std::endl;
    /// @todo This value should be set by entity parameters.
    const mock::Catalogs catalogs;
    RVO::AgentConfig vehicle_agent_config =
      rvo_simulator_.createVehicleConfig(catalogs.getVehicleParameters());
    rvo_ego_ = std::make_shared<RVO::Agent>(
      getCanonicalizedEntityStatus()->getName(), getCanonicalizedEntityStatus()->getMapPose(),
      vehicle_agent_config);
    rvo_simulator_.addAgent(rvo_ego_);
    std::cout << "set agent conf end" << std::endl;
  }
}

/**
 * @brief a single tick of the behavior tree.
 * @param current_time The current time.
 * @param step_time The time step.
 * @return The status of the behavior tree node.
 */
BT::NodeStatus VehiclePlugin::tickOnce(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  return tree_.rootNode()->executeTick();
}

void VehiclePlugin::reflectEgoInRVO(const traffic_simulator::EntityStatus & ego_status)
{
  rvo_ego_->is_available_ = true;
  auto ego_pos = ego_status.pose.position;
  rvo_ego_->setPosition(RVO::Vector2(ego_pos.x, ego_pos.y));
}

void VehiclePlugin::reflectNonEgoEntitiesInRVO()
{
  auto other_entity_status = getOtherEntityStatus();
  rvo_simulator_.markUnavailableAllAgent();
  rvo_ego_->is_available_ = true;
  for (auto entity_status : other_entity_status) {
    auto agent = rvo_simulator_.getAgent(
      static_cast<traffic_simulator::EntityStatus>(entity_status.second).name);
    if (agent) {
      updateNonEgoEntityInRVO(agent, entity_status.second);
    } else {
      createNonEgoEntityInRVO(entity_status.second);
      std::cout << "add new agent" << std::endl;
    }
  }
}

void VehiclePlugin::updateNonEgoEntityInRVO(
  std::shared_ptr<RVO::Agent> agent,
  const traffic_simulator::entity_status::CanonicalizedEntityStatus & canonicalized_entity_status)
{
  agent->is_available_ = true;
  agent->setPose(canonicalized_entity_status.getMapPose());
  agent->setVelocity(RVO::Vector2(
    canonicalized_entity_status.getTwist().linear.x,
    canonicalized_entity_status.getTwist().linear.y));
  // we don't know other agent's navigation planning here.
  // but, the agent velocity is reflecting its planner's decision in previous simulation step
  // so, we set a waypoint in the direction of travel
  agent->updateFrontWaypoint(agent->getPosition() + agent->getVelocity() * 10);
}

void VehiclePlugin::createNonEgoEntityInRVO(
  const traffic_simulator::entity_status::CanonicalizedEntityStatus & canonicalized_entity_status)
{
  auto entity_status = static_cast<traffic_simulator::EntityStatus>(canonicalized_entity_status);
  RVO::AgentConfig other_entity_config;
  const mock::Catalogs catalogs;
  /// @todo This value should be set by entity parameters.
  if (entity_status.type.type == traffic_simulator_msgs::msg::EntityType::VEHICLE) {
    other_entity_config = rvo_simulator_.createVehicleConfig(catalogs.getVehicleParameters());
  } else if (entity_status.type.type == traffic_simulator_msgs::msg::EntityType::PEDESTRIAN) {
    other_entity_config = rvo_simulator_.createPedestrianConfig(catalogs.getPedestrianParameters());
  }
  auto new_agent = std::make_shared<RVO::Agent>(
    entity_status.name, canonicalized_entity_status.getMapPose(), other_entity_config);
  rvo_simulator_.addAgent(new_agent);
}

void VehiclePlugin::updateRVO(double step_time)
{
  auto next_goal = getNextGoal();
  rvo_ego_->updateFrontWaypoint(RVO::Vector2(next_goal.x, next_goal.y));
  rvo_simulator_.setTimeStep(step_time);
  rvo_simulator_.update(rvo_ego_);
}

void VehiclePlugin::updateSimulatorStatus()
{
  auto ego_status =
    static_cast<traffic_simulator::EntityStatus>(*this->getCanonicalizedEntityStatus());
  ego_status.pose = rvo_ego_->getPose();
  ego_status.action_status.twist.linear.x = rvo_ego_->getVelocity().x();
  ego_status.action_status.twist.linear.y = rvo_ego_->getVelocity().y();

  const auto candidate_on_route =
    traffic_simulator::lanelet_wrapper::pose::toLaneletPose(ego_status.pose, getRouteLanelets(), 2.0);
  const auto candidate_not_on_route = traffic_simulator::lanelet_wrapper::pose::toLaneletPose(
    ego_status.pose, ego_status.bounding_box, false, 3.0, traffic_simulator::RoutingGraphType::VEHICLE);
  if (candidate_on_route) {
    ego_status.lanelet_pose_valid = true;
    ego_status.lanelet_pose = candidate_on_route.value();
  } else if (candidate_not_on_route) {
    ego_status.lanelet_pose_valid = true;
    ego_status.lanelet_pose = candidate_not_on_route.value();
  } else {
    ego_status.lanelet_pose_valid = false;
    ego_status.lanelet_pose = traffic_simulator::LaneletPose();
  }
  // std::cout << traffic_simulator_msgs::msg::to_yaml(ego_status) << std::endl;
  getCanonicalizedEntityStatus()->set(
    ego_status, getDefaultMatchingDistanceForLaneletPoseCalculation());
}

void VehiclePlugin::visualize()
{
  visualization_msgs::msg::MarkerArray marker_array;
  rvo_visualize_.visualizeAllMarkers(marker_array, rvo_simulator_);
  rvo_visualize_.visualizeNextGoal(marker_array, getNextGoal());
  for (auto && marker : marker_array.markers) {
    marker.ns += "_" + getCanonicalizedEntityStatus()->getName();
  }
  setDebugMarker(marker_array.markers);
}

}  // namespace context_gamma_planner

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(context_gamma_planner::VehiclePlugin, entity_behavior::BehaviorPluginBase)
