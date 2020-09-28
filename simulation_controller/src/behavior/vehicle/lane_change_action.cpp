#include <simulation_controller/behavior/vehicle/behavior_tree.hpp>
#include <simulation_controller/behavior/vehicle/lane_change_action.hpp>
#include <simulation_controller/entity/vehicle_parameter.hpp>

namespace entity_behavior
{
    namespace vehicle
    {
        LaneChangeAction::LaneChangeAction(const std::string& name,const BT::NodeConfiguration& config) 
            : entity_behavior::ActionNode(name, config)
        {
            
        }

        BT::NodeStatus LaneChangeAction::tick()
        {
            std::string request;
            if(!getInput("request", request))
            {
                throw BehaviorTreeRuntimeError("failed to get input request in LaneChangeAction");
            }
            if(request != "lane_change")
            {
                curve_ = boost::none;
                current_s_ = 0;
                return BT::NodeStatus::FAILURE;
            }

            LaneChangeParameter params;
            if(!getInput<LaneChangeParameter>("lane_change_params", params))
            {
                throw BehaviorTreeRuntimeError("failed to get input lane_change_params in LaneChangeAction");
            }

            std::shared_ptr<simulation_controller::entity::VehicleParameters> vehicle_params_ptr;
            if(!getInput<std::shared_ptr<simulation_controller::entity::VehicleParameters>>("vehicle_parameters", vehicle_params_ptr))
            {
                throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in LaneChangeAction");
            }

            double step_time,current_time;
            if(!getInput<double>("step_time", step_time))
            {
                throw BehaviorTreeRuntimeError("failed to get input step_time in LaneChangeAction");
            }
            if(!getInput<double>("current_time", current_time))
            {
                throw BehaviorTreeRuntimeError("failed to get input current_time in LaneChangeAction");
            }

            std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
            if(!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils_ptr))
            {
                throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in LaneChangeAction");
            }

            simulation_controller::entity::EntityStatus entity_status;
            if(!getInput<simulation_controller::entity::EntityStatus>("entity_status", entity_status))
            {
                throw BehaviorTreeRuntimeError("failed to get input entity_status in LaneChangeAction");
            }

            if(!curve_)
            {
                if(request == "lane_change")
                {
                    if(entity_status.coordinate == simulation_controller::entity::LANE)
                    {
                        if(!hdmap_utils_ptr->canChangeLane(entity_status.lanelet_id, params.to_lanelet_id))
                        {
                            return BT::NodeStatus::FAILURE;
                        }
                    }
                    auto from_pose = hdmap_utils_ptr->toMapPose(entity_status);
                    if(!from_pose)
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                    auto ret = hdmap_utils_ptr->getLaneChangeTrajectory(from_pose->pose, params.to_lanelet_id);
                    if(ret)
                    {
                        curve_ = ret->first;
                        target_s_ = ret->second;
                        setOutput("trajectory", curve_->getTrajectory());
                    }
                    else
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
            if(curve_)
            {
                double current_linear_vel = entity_status.twist.linear.x;
                current_s_ = current_s_ + current_linear_vel * step_time;
                if(current_s_ < curve_->getLength())
                {
                    geometry_msgs::Pose pose = curve_->getPose(current_s_, true);
                    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time, pose, entity_status.twist, entity_status.accel);
                    setOutput("updated_status", entity_status_updated);
                    return BT::NodeStatus::RUNNING;
                }
                else
                {
                    double s = (current_s_ - curve_->getLength()) + target_s_;
                    curve_ = boost::none;
                    current_s_ = 0;
                    geometry_msgs::Vector3 rpy;
                    rpy.x = 0;
                    rpy.y = 0;
                    rpy.z = 0;
                    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time, params.to_lanelet_id, s, 0, rpy, entity_status.twist, entity_status.accel);
                    setOutput("updated_status", entity_status_updated);
                    return BT::NodeStatus::SUCCESS;
                }
            }
            return BT::NodeStatus::FAILURE;
        }
    }  // namespace vehicle
}  // namespace entity_behavior
