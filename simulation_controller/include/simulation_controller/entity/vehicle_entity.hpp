#ifndef SIMULATION_CONTROLLER___VEHICLE_ENTITY_HPP_
#define SIMULATION_CONTROLLER___VEHICLE_ENTITY_HPP_

#include <simulation_controller/entity/entity_base.hpp>
#include <simulation_controller/entity/vehicle_parameter.hpp>
#include <simulation_controller/color_utils/color_utils.hpp>

#include <simulation_controller/behavior/vehicle/lane_change_action.hpp>
#include <simulation_controller/behavior/vehicle/behavior_tree.hpp>

#include <rclcpp/rclcpp.hpp>

// headers in pugixml
#include "pugixml.hpp"

#include <boost/optional.hpp>
#include <memory>

namespace simulation_controller
{
    namespace entity
    {
        class VehicleEntity : public EntityBase
        {
        public:
            VehicleEntity(std::string name, const EntityStatus &initial_state, const pugi::xml_node & xml);
            VehicleEntity(std::string name, const EntityStatus &initial_state, VehicleParameters parameters);
            VehicleEntity(std::string name, const pugi::xml_node & xml);
            VehicleEntity(std::string name, VehicleParameters parameters);
            visualization_msgs::msg::MarkerArray generateMarker(rclcpp::Time stamp, 
                std_msgs::msg::ColorRGBA color = color_utils::makeColorMsg("forestgreen", 0.8)) const;
            const VehicleParameters parameters;
            void onUpdate(double current_time, double step_time) override;
            void requestAcquirePosition(int lanelet_id, double s, double offset);
            void requestLaneChange(int to_lanelet_id);
            void cancelRequest();
            void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr)
            {
                hdmap_utils_ptr_ = ptr;
                tree_ptr_->setValueToBlackBoard("hdmap_utils", hdmap_utils_ptr_);
            }
            void setTargetSpeed(double target_speed, bool continuous);
        private:
            std::shared_ptr<entity_behavior::vehicle::BehaviorTree> tree_ptr_;
            BT::NodeStatus action_status_;
            std::vector<geometry_msgs::msg::Point> following_trajectory_;
            entity_behavior::vehicle::LaneChangeParameter lane_change_params_;
            boost::optional<double> target_speed_;
        };
    }  // namespace entity
}  // namespace simulation_controller

#endif  // SIMULATION_CONTROLLER___VEHICLE_ENTITY_HPP_
