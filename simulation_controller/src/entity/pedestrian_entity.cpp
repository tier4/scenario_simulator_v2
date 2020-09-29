#include <simulation_controller/entity/pedestrian_entity.hpp>
#include <simulation_controller/entity/exception.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>

namespace simulation_controller
{
    namespace entity
    {
        PedestrianEntity::PedestrianEntity(std::string name, const EntityStatus &initial_state, const pugi::xml_node & xml) 
            : EntityBase(xml.child("Pedestrian").attribute("name").as_string(), name, initial_state),
            parameters(xml)
        {
            tree_ptr_ = std::make_shared<entity_behavior::pedestrian::BehaviorTree>();
            tree_ptr_->setValueToBlackBoard("pedestrian_parameters",
                std::make_shared<simulation_controller::entity::PedestrianParameters>(parameters));
        }

        PedestrianEntity::PedestrianEntity(std::string name, const EntityStatus &initial_state, PedestrianParameters params)
            : EntityBase(params.name, name, initial_state),
            parameters(params)
        {
            tree_ptr_ = std::make_shared<entity_behavior::pedestrian::BehaviorTree>();
            tree_ptr_->setValueToBlackBoard("pedestrian_parameters",
                std::make_shared<simulation_controller::entity::PedestrianParameters>(parameters));
        }

        PedestrianEntity::PedestrianEntity(std::string name, const pugi::xml_node & xml) 
            : EntityBase(xml.child("Pedestrian").attribute("name").as_string(), name),
            parameters(xml)
        {
            tree_ptr_ = std::make_shared<entity_behavior::pedestrian::BehaviorTree>();
            tree_ptr_->setValueToBlackBoard("pedestrian_parameters",
                std::make_shared<simulation_controller::entity::PedestrianParameters>(parameters));
        }

        PedestrianEntity::PedestrianEntity(std::string name, PedestrianParameters params)
            : EntityBase(params.name, name),
            parameters(params)
        {
            tree_ptr_ = std::make_shared<entity_behavior::pedestrian::BehaviorTree>();
            tree_ptr_->setValueToBlackBoard("pedestrian_parameters",
                std::make_shared<simulation_controller::entity::PedestrianParameters>(parameters));
        }

        void PedestrianEntity::requestAcquirePosition(int lanelet_id, double s, double offset)
        {
            tree_ptr_->setRequest("acquire_position");
            geometry_msgs::msg::Vector3 rpy;
            geometry_msgs::msg::Twist twist;
            geometry_msgs::msg::Accel accel;
            auto target_status = simulation_controller::entity::EntityStatus(0, lanelet_id, s, offset, rpy, twist, accel);
            tree_ptr_->setValueToBlackBoard("target_status", target_status);
        }
        
        /*
        void PedestrianEntity::requestLaneChange(int to_lanelet_id)
        {
            tree_ptr_->setRequest("lane_change");
            lane_change_params_.to_lanelet_id = to_lanelet_id;
            tree_ptr_->setValueToBlackBoard("lane_change_params", lane_change_params_);
        }
        */

        void PedestrianEntity::cancelRequest()
        {
            tree_ptr_->setRequest("none");
        }

        void PedestrianEntity::setTargetSpeed(double target_speed, bool continuous)
        {
            target_speed_ = target_speed;
            tree_ptr_->setValueToBlackBoard("target_speed", target_speed_);
            if(continuous)
            {
                target_speed_ = boost::none;
            }
        }

        void PedestrianEntity::onUpdate(double current_time, double step_time)
        {
            if(!status_)
            {
                return;
            }
            following_trajectory_.clear();
            tree_ptr_->setValueToBlackBoard("entity_status", status_.get());
            action_status_ = tree_ptr_->tick(current_time, step_time);
            auto status_updated = tree_ptr_->getUpdatedStatus();
            if(target_speed_)
            {
                if(status_updated.twist.linear.x >= target_speed_.get())
                {
                    target_speed_ = boost::none;
                    tree_ptr_->setValueToBlackBoard("target_speed", target_speed_);
                }
            }
            setStatus(status_updated);
            following_trajectory_ = tree_ptr_->getTrajectory();
            updateStandStillDuration(step_time);
        }

        visualization_msgs::msg::MarkerArray PedestrianEntity::generateMarker(rclcpp::Time stamp, std_msgs::msg::ColorRGBA color) const
        {
            visualization_msgs::msg::MarkerArray ret;
            if(!status_)
            {
                return ret;
            }
            visualization_msgs::msg::Marker bbox;
            bbox.header.frame_id = name;
            bbox.header.stamp = stamp;
            bbox.ns = name;
            bbox.id = 0;
            bbox.action = bbox.ADD;
            bbox.pose.position.x = parameters.bounding_box.center.x;
            bbox.pose.position.y = parameters.bounding_box.center.y;
            bbox.pose.position.z = parameters.bounding_box.center.z;
            bbox.pose.orientation.x = 0.0;
            bbox.pose.orientation.y = 0.0;
            bbox.pose.orientation.z = 0.0;
            bbox.pose.orientation.w = 1.0;
            bbox.type = bbox.CUBE;
            bbox.lifetime = rclcpp::Duration(0.1);
            bbox.scale.x = parameters.bounding_box.dimensions.length;
            bbox.scale.y = parameters.bounding_box.dimensions.width;
            bbox.scale.z = parameters.bounding_box.dimensions.height;
            bbox.color = color;
            ret.markers.push_back(bbox);

            visualization_msgs::msg::Marker text;
            text.header.frame_id = name;
            text.header.stamp = stamp;
            text.ns = name;
            text.id = 1;
            text.action = text.ADD;
            text.pose.position.x = parameters.bounding_box.center.x;
            text.pose.position.y = parameters.bounding_box.center.y;
            text.pose.position.z = parameters.bounding_box.center.z + parameters.bounding_box.dimensions.height * 0.5 + 0.8;
            text.pose.orientation.x = 0.0;
            text.pose.orientation.y = 0.0;
            text.pose.orientation.z = 0.0;
            text.pose.orientation.w = 1.0;
            text.type = text.TEXT_VIEW_FACING;
            text.scale.x = 0.0;
            text.scale.y = 0.0;
            text.scale.z = 1.0;
            text.lifetime = rclcpp::Duration(0.1);
            text.text = name;
            text.color = color;
            ret.markers.push_back(text);

            visualization_msgs::msg::Marker text_velocity;
            text_velocity.header.frame_id = name;
            text_velocity.header.stamp = stamp;
            text_velocity.ns = name;
            text_velocity.id = 2;
            text_velocity.action = text_velocity.ADD;
            text_velocity.pose.position.x = parameters.bounding_box.center.x;
            text_velocity.pose.position.y = parameters.bounding_box.center.y;
            text_velocity.pose.position.z = parameters.bounding_box.center.z + parameters.bounding_box.dimensions.height * 0.5;
            text_velocity.pose.orientation.x = 0.0;
            text_velocity.pose.orientation.y = 0.0;
            text_velocity.pose.orientation.z = 0.0;
            text_velocity.pose.orientation.w = 1.0;
            text_velocity.type = text_velocity.TEXT_VIEW_FACING;
            text_velocity.scale.x = 0.0;
            text_velocity.scale.y = 0.0;
            text_velocity.scale.z = 0.5;
            text_velocity.lifetime = rclcpp::Duration(0.1);
            if(target_speed_)
            {
                std::string current_vel = std::to_string(status_.get().twist.linear.x).substr(0,5);
                std::string target_vel = std::to_string(target_speed_.get()).substr(0,5);
                text_velocity.text = "speed: " + current_vel + "=>" + target_vel + " [m/sec]";
            }
            else
            {
                std::string current_vel = std::to_string(status_.get().twist.linear.x).substr(0,5);
                text_velocity.text = "speed: " + current_vel + " [m/sec]";
            }
            text_velocity.color = color_utils::makeColorMsg("lightgray", 1.0);
            ret.markers.push_back(text_velocity);

            visualization_msgs::msg::Marker text_action;
            text_action.header.frame_id = name;
            text_action.header.stamp = stamp;
            text_action.ns = name;
            text_action.id = 3;
            text_action.action = text_action.ADD;
            text_action.pose.position.x = parameters.bounding_box.center.x;
            text_action.pose.position.y = parameters.bounding_box.center.y;
            text_action.pose.position.z = parameters.bounding_box.center.z + parameters.bounding_box.dimensions.height * 0.5 - 0.5;
            text_action.pose.orientation.x = 0.0;
            text_action.pose.orientation.y = 0.0;
            text_action.pose.orientation.z = 0.0;
            text_action.pose.orientation.w = 1.0;
            text_action.type = text_action.TEXT_VIEW_FACING;
            text_action.scale.x = 0.0;
            text_action.scale.y = 0.0;
            text_action.scale.z = 0.5;
            text_action.lifetime = rclcpp::Duration(0.1);
            text_action.text = "behavior: " + tree_ptr_->getCurrentAction();
            text_action.color = color_utils::makeColorMsg("lightgray", 1.0);
            ret.markers.push_back(text_action);

            if(verbose_)
            {
                visualization_msgs::msg::Marker trajectory_marker;
                trajectory_marker.header.frame_id = "map";
                trajectory_marker.ns = name;
                trajectory_marker.id = 4;
                trajectory_marker.action = trajectory_marker.ADD;
                trajectory_marker.type = trajectory_marker.LINE_STRIP;
                trajectory_marker.scale.x = 0.3;
                trajectory_marker.scale.y = 0.3;
                trajectory_marker.scale.z = 0.3;
                trajectory_marker.color = color;
                trajectory_marker.points = following_trajectory_;
                trajectory_marker.lifetime = rclcpp::Duration(0.1);
                for(auto itr=trajectory_marker.points.begin(); itr!=trajectory_marker.points.end(); itr++)
                {
                    trajectory_marker.colors.push_back(color);
                }
                ret.markers.push_back(trajectory_marker);

                visualization_msgs::msg::Marker text_position;
                text_position.header.frame_id = name;
                text_position.header.stamp = stamp;
                text_position.ns = name;
                text_position.id = 5;
                text_position.action = text_position.ADD;
                text_position.pose.position.x = parameters.bounding_box.center.x;
                text_position.pose.position.y = parameters.bounding_box.center.y;
                text_position.pose.position.z = parameters.bounding_box.center.z - parameters.bounding_box.dimensions.height * 0.5;
                text_position.pose.orientation.x = 0.0;
                text_position.pose.orientation.y = 0.0;
                text_position.pose.orientation.z = 0.0;
                text_position.pose.orientation.w = 1.0;
                text_position.type = text_position.TEXT_VIEW_FACING;
                text_position.scale.x = 0.0;
                text_position.scale.y = 0.0;
                text_position.scale.z = 0.5;
                std::string s_str = std::to_string(status_->s).substr(0,5);
                if(status_->coordinate == CoordinateFrameTypes::LANE)
                {
                    text_position.text = "lane: " + std::to_string(status_->lanelet_id) + "\ns:  " + s_str;
                }
                else
                {
                    text_position.text = "lane: none\n s:  none";
                }
                
                text_position.color = color_utils::makeColorMsg("lightgray", 1.0);
                text_position.lifetime = rclcpp::Duration(0.1);
                ret.markers.push_back(text_position);
            }
            return ret;
        }
    }  // namespace entity
} // namespace simulation_controller
