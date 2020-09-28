#ifndef ENTITY_BEHAVIOR__VEHICLE__ACQUIRE_POSITION_ACTION_HPP
#define ENTITY_BEHAVIOR__VEHICLE__ACQUIRE_POSITION_ACTION_HPP

#include <simulation_controller/entity/vehicle_parameter.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/behavior/action_node.hpp>

#include <geometry_msgs/msg/point.hpp>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <boost/optional.hpp>

namespace entity_behavior
{
    namespace vehicle
    {
        class AcquirePositionAction : public entity_behavior::ActionNode
        {
        public:
            AcquirePositionAction(const std::string& name, const BT::NodeConfiguration& config);
            BT::NodeStatus tick() override;
            static BT::PortsList providedPorts()
            {
                return 
                { 
                    BT::InputPort<std::string>("request"),
                    BT::InputPort<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils"),
                    BT::InputPort<simulation_controller::entity::EntityStatus>("entity_status"),
                    BT::InputPort<double>("current_time"),
                    BT::InputPort<double>("step_time"),
                    BT::InputPort<boost::optional<double>>("target_speed"),
                    BT::InputPort<std::shared_ptr<simulation_controller::entity::VehicleParameters>>("vehicle_parameters"),
                    BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("trajectory"),
                    BT::OutputPort<simulation_controller::entity::EntityStatus>("updated_status"),
                    BT::OutputPort<std::string>("request"),

                    BT::InputPort<simulation_controller::entity::EntityStatus>("target_status")
                };
            }
        private:
            boost::optional<simulation_controller::entity::EntityStatus> target_status_;
            std::vector<geometry_msgs::msg::Point> following_trajectory_;
            boost::optional<std::vector<int> > route_;
        };
    }  // namespace vehicle
}  // namespace entity_behavior

#endif  // ENTITY_BEHAVIOR__VEHICLE__ACQUIRE_POSITION_ACTION_HPP
