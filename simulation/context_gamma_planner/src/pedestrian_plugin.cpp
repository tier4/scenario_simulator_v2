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

#include "context_gamma_planner/pedestrian_plugin.hpp"

namespace context_gamma_planner
{

void PedestrianPlugin::configure(const rclcpp::Logger & logger)
{
  std::string path = ament_index_cpp::get_package_share_directory("context_gamma_planner") +
                     "/config/pedestrian_behavior.xml";
  factory_.registerNodeType<context_gamma_planner::pedestrian::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<context_gamma_planner::pedestrian::FollowPolylineTrajectoryAction>(
    "FollowPolylineTrajectory");
  tree_ = createBehaviorTree(path);
  logging_event_ptr_ =
    std::make_shared<context_gamma_planner::LoggingEvent>(tree_.rootNode(), logger);
  reset_request_event_ptr_ = std::make_shared<context_gamma_planner::ResetRequestEvent>(
    tree_.rootNode(), [this]() { return getRequest(); },
    [this](traffic_simulator::behavior::Request request) { return setRequest(request); });
  setRequest(traffic_simulator::behavior::Request::NONE);
  traffic_simulator_msgs::msg::Obstacle obstacle_msg;
  obstacle_msg.type = traffic_simulator_msgs::msg::Obstacle::ENTITY;
  setObstacle(obstacle_msg);
}

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

auto PedestrianPlugin::getCurrentAction() const -> const std::string &
{
  return logging_event_ptr_->getCurrentAction();
}

void PedestrianPlugin::update(double current_time, double step_time)
{
  using math::geometry::operator-;

  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }

  auto transformLocalToGlobalVelocity =
    [](const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & local_vel) {
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      tf2::Vector3 lv(local_vel.x, local_vel.y, local_vel.z);
      tf2::Vector3 gv = tf2::quatRotate(q, lv);
      return tf2::toMsg(gv);
    };

  const auto ego_entity = getCanonicalizedEntityStatus();
  auto ego_pose = ego_entity->getMapPose();
  const auto ego_bbox = ego_entity->getBoundingBox();
  const auto ego_local_linear_velocity = ego_entity->getTwist().linear;
  const auto ego_global_linear_velocity =
    transformLocalToGlobalVelocity(ego_pose, ego_local_linear_velocity);
  const auto ego_angle = math::geometry::convertQuaternionToEulerAngle(ego_pose.orientation).z;

  ego_pose.position.x += ego_bbox.center.x * cos(ego_angle) - ego_bbox.center.y * sin(ego_angle);
  ego_pose.position.y += ego_bbox.center.x * sin(ego_angle) + ego_bbox.center.y * cos(ego_angle);

  std::vector<line> orca_lines;
  const auto other_entity_status = getOtherEntityStatus();
  for (const auto & [other_name, other_entity] : other_entity_status) {
    if (other_name.find("__CONTEXT_GAMMA_IGNORE__") != std::string::npos) {
      continue;
    }

    const auto & other_bbox = other_entity.getBoundingBox();
    auto other_pose = other_entity.getMapPose();
    const auto other_local_linear_velocity = other_entity.getTwist().linear;
    const auto other_global_linear_velocity =
      transformLocalToGlobalVelocity(other_pose, other_local_linear_velocity);
    const auto other_angle =
      math::geometry::convertQuaternionToEulerAngle(other_pose.orientation).z;

    other_pose.position.x +=
      other_bbox.center.x * cos(other_angle) - other_bbox.center.y * sin(other_angle);
    other_pose.position.y +=
      other_bbox.center.x * sin(other_angle) + other_bbox.center.y * cos(other_angle);

    const auto relative_position = other_pose.position - ego_pose.position;
    const auto relative_velocity = ego_global_linear_velocity - other_global_linear_velocity;

    orca_lines.push_back(calculateOrcaLine(
      ego_global_linear_velocity, relative_position, relative_velocity, ego_bbox, ego_angle,
      other_bbox, other_angle, step_time));
  }
  const auto planning_speed = getPlanningSpeed() ? getPlanningSpeed().value() : 0.0;

  const auto optimized_velocity = optimizeVelocityWithConstraints(
    orca_lines, planning_speed, math::geometry::castToVec(getNextGoal() - ego_pose.position),
    false);
  if (optimized_velocity) {
    updateEgoPose(optimized_velocity.value(), step_time);
  }
}

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
