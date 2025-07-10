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

#include <context_gamma_planner/behavior/pedestrian/follow_lane_action.hpp>
#include <context_gamma_planner/behavior/pedestrian/follow_polyline_trajectory_action.hpp>
#include <context_gamma_planner/pedestrian_plugin.hpp>
#include <iostream>
#include <pugixml.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>

namespace context_gamma_planner
{
/**
 * @brief Configures the PedestrianPlugin.
 * @param logger The logger object used for logging.
 */
void PedestrianPlugin::configure(const rclcpp::Logger & logger)
{
  std::cout << "pedestrian plugin configure start" << std::endl;
  std::string path = ament_index_cpp::get_package_share_directory("context_gamma_planner") +
                     "/config/pedestrian_behavior.xml";
  factory_.registerNodeType<context_gamma_planner::pedestrian::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<context_gamma_planner::pedestrian::FollowPolylineTrajectoryAction>(
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
  setObstacle(obstacle_msg);
}

/**
 * @brief Creates a behavior tree from an XML file.
 * @param format_path The path to the XML file.
 * @return The created behavior tree.
 */
auto PedestrianPlugin::createBehaviorTree(const std::string & format_path) -> BT::Tree
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
 * @brief Get the current action.
 * @return const std::string& The current action.
 */
auto PedestrianPlugin::getCurrentAction() -> const std::string &
{
  return logging_event_ptr_->getCurrentAction();
}

/**
 * @brief Updates the pedestrian plugin.
 * @param current_time The current time.
 * @param step_time The time step.
 */
void PedestrianPlugin::update(double current_time, double step_time)
{
  using math::geometry::operator-;

  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
  if (getPlanningSpeed()) {
  }

  auto transformLocalToGlobalVelocity =
    [](const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Vector3 local_vel) {
      const auto yaw = math::geometry::convertQuaternionToEulerAngle(pose.orientation).z;
      local_vel.x = std::cos(yaw) * local_vel.x - std::sin(yaw) * local_vel.y;
      local_vel.y = std::sin(yaw) * local_vel.x + std::cos(yaw) * local_vel.y;
      return local_vel;
    };
  auto cast_to_vec = [](const geometry_msgs::msg::Point & p) {
    auto result = geometry_msgs::msg::Vector3();
    result.x = p.x;
    result.y = p.y;
    result.z = p.z;
    return result;
  };

  auto next_goal = getNextGoal();
  // std::cout << next_goal.x << ", " << next_goal.y << std::endl;
  const auto ego_entity = getCanonicalizedEntityStatus();
  const auto ego_pose = ego_entity->getMapPose();
  const auto ego_vel = transformLocalToGlobalVelocity(ego_pose, ego_entity->getTwist().linear);
  const auto ego_poly =
    rotate_polygon(polygon_from_bbox(ego_entity->getBoundingBox()), ego_pose.orientation);

  std::vector<line> orca_lines;
  const auto other_entity_status = getOtherEntityStatus();
  for (const auto & [other_name, other_entity] : other_entity_status) {
    const auto other_pose = other_entity.getMapPose();
    const auto other_vel =
      transformLocalToGlobalVelocity(other_pose, other_entity.getTwist().linear);
    const auto other_poly =
      rotate_polygon(polygon_from_bbox(other_entity.getBoundingBox()), other_pose.orientation);

    const auto relative_position = other_pose.position - ego_pose.position;
    const auto relative_velocity = ego_vel - other_vel;

    orca_lines.push_back(
      calculate_orca_line(ego_vel, relative_position, relative_velocity, ego_poly, other_poly));
  }

  const auto optimized_velocity = optimizeVelocityWithConstraints(
    orca_lines, 1.0, cast_to_vec(getNextGoal() - ego_pose.position), false);
  if (optimized_velocity) {
    updateEgoPose(optimized_velocity.value(), step_time);
  }
}

/**
 * @brief a single tick of the behavior tree.
 * @param current_time The current time.
 * @param step_time The time step.
 * @return The status of the behavior tree node.
 */
BT::NodeStatus PedestrianPlugin::tickOnce(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  return tree_.rootNode()->executeTick();
}

void PedestrianPlugin::updateEgoPose(
  const geometry_msgs::msg::Vector3 & velocity, const double step_time)
{
  const auto ego_entity = getCanonicalizedEntityStatus();
  auto map_pose = ego_entity->getMapPose();
  map_pose.position.x += velocity.x * step_time;
  map_pose.position.y += velocity.y * step_time;
  if (velocity.x != 0.0 || velocity.y != 0.0) {
    auto angular = geometry_msgs::msg::Vector3();
    angular.set__z(std::atan2(velocity.y, velocity.x));
    map_pose.orientation = math::geometry::convertEulerAngleToQuaternion(angular);
  }

  ego_entity->setMapPose(map_pose);
  ego_entity->setLinearVelocity(math::geometry::norm(velocity));
}

}  // namespace context_gamma_planner

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(context_gamma_planner::PedestrianPlugin, entity_behavior::BehaviorPluginBase)
