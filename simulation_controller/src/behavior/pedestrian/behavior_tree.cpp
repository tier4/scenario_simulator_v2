#include <simulation_controller/behavior/pedestrian/behavior_tree.hpp>

#include <ros/package.h>

#include <iostream>

namespace entity_behavior
{
    namespace pedestrian
    {
        BehaviorTree::BehaviorTree()
        {
            std::string path = ros::package::getPath("simulation_controller") + "/resource/pedestrian_entity_behavior.xml";
            factory_.registerNodeType<entity_behavior::pedestrian::FollowLaneAction>("FollowLane");
            factory_.registerNodeType<entity_behavior::pedestrian::AcquirePositionAction>("AcquirePosition");
            //factory_.registerNodeType<entity_behavior::pedestrian::LaneChangeAction>("LaneChange");
            tree_ = factory_.createTreeFromFile(path);
            current_action_ = "root";
            //logger_cout_ptr_ = std::make_shared<BT::StdCoutLogger>(tree_);
            setupLogger();
            setRequest("none");
        }

        void BehaviorTree::setRequest(std::string request)
        {
            request_ = request;
            setValueToBlackBoard("request", request);
        }

        void BehaviorTree::setupLogger()
        {
            first_timestamp_ = std::chrono::high_resolution_clock::now();
            auto subscribeCallback = [this](BT::TimePoint timestamp, const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status) 
            {
                if (status != BT::NodeStatus::IDLE)
                {
                    if (type_ == BT::TimestampType::ABSOLUTE)
                    {
                        this->callback(timestamp.time_since_epoch(), node, prev, status);
                    }
                    else
                    {
                        this->callback(timestamp - first_timestamp_, node, prev, status);
                    }
                }
            };
            auto visitor = [this, subscribeCallback](BT::TreeNode* node) {
                subscribers_.push_back(node->subscribeToStatusChange(std::move(subscribeCallback)));
            };
            applyRecursiveVisitor(tree_.rootNode(), visitor);
        }

        const BT::NodeStatus BehaviorTree::tick(double current_time, double step_time)
        {
            setValueToBlackBoard("current_time", current_time);
            setValueToBlackBoard("step_time", step_time);
            auto status = tree_.tickRoot();
            return status;
        }

        void BehaviorTree::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status)
        {
            current_action_ = node.name();
            if(status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
            {
                if(request_ == current_action_)
                {
                    setRequest("none");
                }
            }
        }
    }
}
