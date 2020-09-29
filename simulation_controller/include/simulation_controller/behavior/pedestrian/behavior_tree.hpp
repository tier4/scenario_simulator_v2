#ifndef ENTITY_BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_ACTION_HPP
#define ENTITY_BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_ACTION_HPP

#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/behavior/pedestrian/follow_lane_action.hpp>
#include <simulation_controller/behavior/pedestrian/acquire_position_action.hpp>
#include <simulation_controller/behavior/pedestrian/lane_change_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

#include <memory>
#include <functional>
#include <map>
#include <vector>

namespace entity_behavior
{
    namespace pedestrian
    {
        class BehaviorTree
        {
        public:
            BehaviorTree();
            BT::NodeStatus tick(double current_time, double step_time);
            std::string getCurrentAction() const
            {
                return current_action_;
            }
            template <typename T>
            void setValueToBlackBoard(std::string key,T value)
            {
                tree_.rootBlackboard()->set(key, value);
            }
            simulation_controller::entity::EntityStatus getUpdatedStatus()
            {
                simulation_controller::entity::EntityStatus status;
                tree_.rootBlackboard()->get("updated_status", status);
                return status;
            }
            std::vector<geometry_msgs::msg::Point> getTrajectory()
            {
                std::vector<geometry_msgs::msg::Point> ret;
                tree_.rootBlackboard()->get("trajectory", ret);
                return ret;
            }
            void setRequest(std::string request);
        private:
            std::string request_;
            BT::BehaviorTreeFactory factory_;
            BT::Tree tree_;
            std::shared_ptr<BT::StdCoutLogger> logger_cout_ptr_;
            void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status);
            void setupLogger();
            BT::TimestampType type_;
            BT::TimePoint first_timestamp_;
            std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
            std::string current_action_;
        };
    }  // namespace pedestrian
}  // namespace entity_behavior

#endif  // ENTITY_BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_ACTION_HPP