#ifndef ENTITY_BEHAVIOR__VEHICLE__FOLLOW_LANE_ACTION_HPP
#define ENTITY_BEHAVIOR__VEHICLE__FOLLOW_LANE_ACTION_HPP

#include <simulation_controller/entity/entity_base.hpp>
#include <simulation_controller/behavior/action_node.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/entity/vehicle_parameter.hpp>
#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace entity_behavior
{
    namespace vehicle
    {
        class FollowLaneAction : public entity_behavior::ActionNode
        {
        public:
            FollowLaneAction(const std::string& name, const BT::NodeConfiguration& config);
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
                    BT::InputPort<std::unordered_map<std::string,simulation_controller::entity::EntityStatus>>("other_entity_status"),
                    BT::InputPort<std::unordered_map<std::string,simulation_controller::entity::EntityType>>("entity_type_list")
                };
            }
        };
    }  // namespace vehicle
} // namespace entity_behavior

#endif  // ENTITY_BEHAVIOR__VEHICLE__FOLLOW_LANE_ACTION_HPP
