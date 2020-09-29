#ifndef  SIMULATOR_CONTROLLER__ENTITY_MANAGER_HPP_
#define  SIMULATOR_CONTROLLER__ENTITY_MANAGER_HPP_

#include <simulation_controller/entity/ego_entity.hpp>
#include <simulation_controller/entity/vehicle_entity.hpp>
#include <simulation_controller/entity/pedestrian_entity.hpp>
#include <simulation_controller/entity/exception.hpp>

#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <type_traits>
#include <typeinfo>
#include <map>
#include <memory>

namespace simulation_controller
{
    namespace entity
    {
        class EntityManager
        {
        private:
            std::map<std::string, boost::any> entities_;
            std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
            rclcpp::Clock::SharedPtr clock_ptr_;
        public:
            template<class NodeT>
            EntityManager(NodeT && node): broadcaster_(node)
            {
                clock_ptr_ = node->get_clock();
                hdmap_utils_ptr_ = std::make_shared<hdmap_utils::HdMapUtils>("/map/vector_map");
            }
            void setVerbose(bool verbose)
            {
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    if(it->second.type() == typeid(VehicleEntity))
                    {
                        boost::any_cast<VehicleEntity&>(it->second).setVerbose(verbose);
                    }
                    if(it->second.type() == typeid(EgoEntity))
                    {
                        boost::any_cast<EgoEntity&>(it->second).setVerbose(verbose);
                    }
                    if(it->second.type() == typeid(PedestrianEntity))
                    {
                        boost::any_cast<PedestrianEntity&>(it->second).setVerbose(verbose);
                    }
                }
            }
            void requestAcquirePosition(std::string name, int lanelet_id, double s, double offset)
            {
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    boost::any_cast<VehicleEntity&>(it->second).requestAcquirePosition(lanelet_id, s, offset);
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    boost::any_cast<EgoEntity&>(it->second).requestAcquirePosition(lanelet_id, s, offset);
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    boost::any_cast<PedestrianEntity&>(it->second).requestAcquirePosition(lanelet_id, s, offset);
                }
            }
            void requestLaneChange(std::string name, int to_lanelet_id)
            {
                if(getEntityStatusCoordinate(name) == CoordinateFrameTypes::WORLD)
                {
                    return;
                }
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    boost::any_cast<VehicleEntity&>(it->second).requestLaneChange(to_lanelet_id);
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    boost::any_cast<EgoEntity&>(it->second).requestLaneChange(to_lanelet_id);
                }
            }
            void requestLaneChange(std::string name, Direction direction)
            {
                if(getEntityStatusCoordinate(name) == CoordinateFrameTypes::WORLD)
                {
                    return;
                }
                auto status = getEntityStatus(name, CoordinateFrameTypes::LANE);
                if(!status)
                {
                    return;
                }
                if(direction == Direction::LEFT)
                {
                    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_id, "left");
                    if(target)
                    {
                        requestLaneChange(name, target.get());
                        return;
                    }
                    return;
                }
                if(direction == Direction::RIGHT)
                {
                    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_id, "right");
                    if(target)
                    {
                        requestLaneChange(name, target.get());
                        return;
                    }
                    return;
                }
                return;
            }
            boost::optional<double> getLongitudinalDistance(std::string from, std::string to, double max_distance = 100)
            {
                if(!entityStatusSetted(from) || !entityStatusSetted(to))
                {
                    return boost::none;
                }
                if(getEntityStatusCoordinate(from) == CoordinateFrameTypes::LANE && getEntityStatusCoordinate(to) == CoordinateFrameTypes::LANE)
                {
                    auto from_status = getEntityStatus(from, CoordinateFrameTypes::LANE);
                    auto to_status = getEntityStatus(to, CoordinateFrameTypes::LANE);
                    if(from_status &&  to_status)
                    {
                        auto dist =  hdmap_utils_ptr_->getLongitudinalDistance(from_status->lanelet_id, from_status->s, 
                            to_status->lanelet_id, to_status->s);
                        if(!dist)
                        {
                            return boost::none;
                        }
                        else
                        {
                            if(dist <= max_distance)
                            {
                                return dist.get();
                            }
                            return boost::none;
                        }
                    }
                }
                return boost::none;
            }
            geometry_msgs::msg::Pose getRelativePose(std::string from, std::string to)
            {
                auto from_status = getEntityStatus(from);
                auto to_status =  getEntityStatus(to);
                if(!from_status)
                {
                    throw simulation_controller::SimulationRuntimeError("failed to get status of " + from + " entity in getRelativePose");
                }
                if(!to_status)
                {
                    throw simulation_controller::SimulationRuntimeError("failed to get status of " + to + " entity in getRelativePose");
                }
                auto from_pose = from_status->pose;
                auto to_pose = to_status->pose;
                return getRelativePose(from_pose, to_pose);
            }
            geometry_msgs::msg::Pose getRelativePose(std::string from, geometry_msgs::msg::Pose to)
            {
                auto from_status = getEntityStatus(from);
                if(!from_status)
                {
                    throw simulation_controller::SimulationRuntimeError("failed to get status of " + from + " entity in getRelativePose");
                }
                return getRelativePose(from_status->pose, to);
            }
            geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, std::string to)
            {
                auto to_status = getEntityStatus(to);
                if(!to_status)
                {
                    throw simulation_controller::SimulationRuntimeError("failed to get status of " + to + " entity in getRelativePose");
                }
                return getRelativePose(from, to_status->pose);
            }
            geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, geometry_msgs::msg::Pose to) const
            {
                geometry_msgs::msg::Transform from_translation;
                from_translation.translation.x = from.position.x;
                from_translation.translation.y = from.position.y;
                from_translation.translation.z = from.position.z;
                from_translation.rotation = from.orientation;
                tf2::Transform from_tf;
                tf2::fromMsg(from_translation, from_tf);
                geometry_msgs::msg::Transform to_translation;
                to_translation.translation.x = to.position.x;
                to_translation.translation.y = to.position.y;
                to_translation.translation.z = to.position.z;
                to_translation.rotation = to.orientation;
                tf2::Transform to_tf;
                tf2::fromMsg(to_translation, to_tf);
                tf2::Transform tf_delta;
                tf_delta = from_tf.inverse() * to_tf;
                geometry_msgs::msg::Pose ret;
                tf2::toMsg(tf_delta, ret);
                return ret;
            }
            boost::optional<VehicleParameters> getVehicleParameters(std::string name)
            {
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return boost::none;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<const VehicleEntity&>(it->second).parameters;
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<const EgoEntity&>(it->second).parameters;
                }
                return boost::none;
            }
            std::vector<std::string> getEntityNames()
            {
                std::vector<std::string> ret;
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    ret.push_back(it->first);
                }
                return ret;
            }
            visualization_msgs::msg::MarkerArray generateMarker()
            {
                visualization_msgs::msg::MarkerArray ret;
                rclcpp::Time now = clock_ptr_->now();
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    if(it->second.type() == typeid(VehicleEntity))
                    {
                        const auto marker = boost::any_cast<const VehicleEntity&>(it->second).generateMarker(now, color_utils::makeColorMsg("steelblue", 0.8));
                        ret.markers.insert(ret.markers.end(), marker.markers.begin(), marker.markers.end());
                    }
                    if(it->second.type() == typeid(EgoEntity))
                    {
                        const auto marker = boost::any_cast<const EgoEntity&>(it->second).generateMarker(now, color_utils::makeColorMsg("forestgreen", 0.8));
                        ret.markers.insert(ret.markers.end(), marker.markers.begin(), marker.markers.end());
                    }
                    if(it->second.type() == typeid(PedestrianEntity))
                    {
                        const auto marker = boost::any_cast<const PedestrianEntity&>(it->second).generateMarker(now, color_utils::makeColorMsg("orange", 0.8));
                        ret.markers.insert(ret.markers.end(), marker.markers.begin(), marker.markers.end());
                    }
                }
                return ret;
            }
            bool setEntityStatus(std::string name, EntityStatus status)
            {
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return false;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<VehicleEntity&>(it->second).setStatus(status);
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<EgoEntity&>(it->second).setStatus(status);
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    return boost::any_cast<PedestrianEntity&>(it->second).setStatus(status);
                }
                return false;
            }
            CoordinateFrameTypes getEntityStatusCoordinate(std::string name)
            {
                auto it = entities_.find(name);
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<const VehicleEntity&>(it->second).getStatusCoordinateFrameType();
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<const EgoEntity&>(it->second).getStatusCoordinateFrameType();
                }
                throw simulation_controller::SimulationRuntimeError("failed to get entity status coordinate, entity name does not match, entity_name : " + name);
            }
            boost::optional<EntityStatus> getEntityStatus(std::string name, CoordinateFrameTypes coordinate = CoordinateFrameTypes::WORLD)
            {
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return boost::none;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<const VehicleEntity&>(it->second).getStatus(coordinate);
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<const EgoEntity&>(it->second).getStatus(coordinate);
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    return boost::any_cast<const PedestrianEntity&>(it->second).getStatus(coordinate);
                }
                return boost::none;
            }
            bool entityStatusSetted(std::string name)
            {
                auto it = entities_.find(name);
                if (it == entities_.end())
                {
                    return false;
                }
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<VehicleEntity&>(it->second).statusSetted();
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<EgoEntity&>(it->second).statusSetted();
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    return boost::any_cast<PedestrianEntity&>(it->second).statusSetted();
                }
                return false;
            }
            void setTargetSpeed(std::string name, double target_speed, bool continuous)
            {
                auto it = entities_.find(name);
                if(it->second.type() == typeid(VehicleEntity))
                {
                    boost::any_cast<VehicleEntity&>(it->second).setTargetSpeed(target_speed, continuous);
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    boost::any_cast<EgoEntity&>(it->second).setTargetSpeed(target_speed, continuous);
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    boost::any_cast<PedestrianEntity&>(it->second).setTargetSpeed(target_speed, continuous);
                }
            }
            void update(double current_time, double step_time)
            {
                auto type_list = getEntityTypeList();
                std::unordered_map<std::string,EntityStatus> all_status;
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    if(it->second.type() == typeid(VehicleEntity))
                    {
                        boost::any_cast<VehicleEntity&>(it->second).setEntityTypeList(type_list);
                        boost::any_cast<VehicleEntity&>(it->second).onUpdate(current_time, step_time);
                        if(boost::any_cast<VehicleEntity&>(it->second).statusSetted())
                        {
                            auto status = boost::any_cast<VehicleEntity&>(it->second).getStatus(
                                boost::any_cast<VehicleEntity&>(it->second).getStatusCoordinateFrameType());
                            all_status[boost::any_cast<VehicleEntity&>(it->second).name] = status;
                        }
                    }
                    if(it->second.type() == typeid(EgoEntity))
                    {
                        boost::any_cast<EgoEntity&>(it->second).setEntityTypeList(type_list);
                        boost::any_cast<EgoEntity&>(it->second).onUpdate(current_time, step_time);
                        if(boost::any_cast<EgoEntity&>(it->second).statusSetted())
                        {
                            auto status = boost::any_cast<EgoEntity&>(it->second).getStatus(
                                boost::any_cast<EgoEntity&>(it->second).getStatusCoordinateFrameType());
                            all_status[boost::any_cast<EgoEntity&>(it->second).name] = status;
                        }
                    }
                    if(it->second.type() == typeid(PedestrianEntity))
                    {
                        boost::any_cast<PedestrianEntity&>(it->second).setEntityTypeList(type_list);
                        boost::any_cast<PedestrianEntity&>(it->second).onUpdate(current_time, step_time);
                        if(boost::any_cast<PedestrianEntity&>(it->second).statusSetted())
                        {
                            auto status = boost::any_cast<PedestrianEntity&>(it->second).getStatus(
                                boost::any_cast<PedestrianEntity&>(it->second).getStatusCoordinateFrameType());
                            all_status[boost::any_cast<PedestrianEntity&>(it->second).name] = status;
                        }
                    }
                }
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    if(it->second.type() == typeid(VehicleEntity))
                    {
                        boost::any_cast<VehicleEntity&>(it->second).setOtherStatus(all_status);
                    }
                    if(it->second.type() == typeid(EgoEntity))
                    {
                        boost::any_cast<EgoEntity&>(it->second).setOtherStatus(all_status);
                    }
                    if(it->second.type() == typeid(PedestrianEntity))
                    {
                        boost::any_cast<PedestrianEntity&>(it->second).setOtherStatus(all_status);
                    }
                }
            }
            void broadcastTransform(geometry_msgs::msg::PoseStamped pose)
            {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = pose.header.stamp;
                transform_stamped.header.frame_id = "map";
                transform_stamped.child_frame_id = pose.header.frame_id;
                transform_stamped.transform.translation.x = pose.pose.position.x;
                transform_stamped.transform.translation.y = pose.pose.position.y;
                transform_stamped.transform.translation.z = pose.pose.position.z;
                transform_stamped.transform.rotation = pose.pose.orientation;
                broadcaster_.sendTransform(transform_stamped);
            }
            bool reachPosition(std::string name, geometry_msgs::msg::Pose target_pose, double tolerance)
            {
                auto status = getEntityStatus(name);
                if(!status)
                {
                    throw simulation_controller::SimulationRuntimeError("error occurs while getting entity stauts, target entity : " + name);
                }
                auto pose = status->pose;
                double dist = std::sqrt(std::pow(pose.position.x-target_pose.position.x,2) + std::pow(pose.position.y-target_pose.position.y,2)
                    + std::pow(pose.position.z-target_pose.position.z,2));
                if(dist < tolerance)
                {
                    return true;
                }
                return false;
            }
            bool reachPosition(std::string name, int lanelet_id, double s, double offset, double tolerance)
            {
                geometry_msgs::msg::Vector3 rpy;
                geometry_msgs::msg::Twist twist;
                geometry_msgs::msg::Accel accel;
                auto target_status = simulation_controller::entity::EntityStatus(0, lanelet_id, s, offset, rpy, twist, accel);
                auto target_pose = hdmap_utils_ptr_->toMapPose(target_status);
                if(!target_pose)
                {
                    throw simulation_controller::SimulationRuntimeError("failed to transform into map frame");
                }
                return reachPosition(name, target_pose->pose, tolerance);
            }
            void broadcastEntityTransform()
            {
                std::vector<std::string> names = getEntityNames();
                for(auto it = names.begin(); it != names.end(); it++)
                {
                    if(entityStatusSetted(*it))
                    {
                        auto status = getEntityStatus(*it);
                        if(status)
                        {
                            auto pose = hdmap_utils_ptr_->toMapPose(status.get());
                            if(pose)
                            {
                                pose->header.stamp = clock_ptr_->now();
                                pose->header.frame_id = *it;
                                broadcastTransform(pose.get());
                            }
                        }
                    }
                }
            }
            boost::optional<double> getStandStillDuration(std::string name)
            {
                auto it = entities_.find(name);
                if(it->second.type() == typeid(VehicleEntity))
                {
                    return boost::any_cast<VehicleEntity&>(it->second).getStandStillDuration();
                }
                if(it->second.type() == typeid(EgoEntity))
                {
                    return boost::any_cast<EgoEntity&>(it->second).getStandStillDuration();
                }
                if(it->second.type() == typeid(PedestrianEntity))
                {
                    return boost::any_cast<PedestrianEntity&>(it->second).getStandStillDuration();
                }
                throw simulation_controller::SimulationRuntimeError("entity " + name + "does not exist");
            }
            const std::unordered_map<std::string,EntityType> getEntityTypeList() const
            {
                std::unordered_map<std::string,EntityType> ret;
                for(auto it = entities_.begin(); it != entities_.end(); it++)
                {
                    if(it->second.type() == typeid(VehicleEntity))
                    {
                        ret[it->first] = EntityType::VEHICLE;
                    }
                    if(it->second.type() == typeid(EgoEntity))
                    {
                        ret[it->first] = EntityType::EGO;
                    }
                    if(it->second.type() == typeid(PedestrianEntity))
                    {
                        ret[it->first] = EntityType::PEDESTRIAN;
                    }
                }
                return ret;
            }
            tf2_ros::TransformBroadcaster broadcaster_;
            template<typename T>
            bool spawnEntity(T& entity)
            {                
                if(entities_.count(entity.name) != 0)
                {
                    throw simulation_controller::SimulationRuntimeError("entity " + entity.name + " already exist.");
                }
                if(std::is_base_of<EntityBase, T>::value == false)
                {
                    return false;
                }
                entity.setHdMapUtils(hdmap_utils_ptr_);
                entities_.insert(std::make_pair(entity.name,entity));
                return true;
            }
            bool despawnEntity(std::string name)
            {
                if(entities_.count(name) == 0)
                {
                    return false;
                }
                entities_.erase(name);
                return true;
            }
        };
    }  // namespace entity
}  // namespace simulation_controller

#endif   // SIMULATOR_CONTROLLER__ENTITY_MANAGER_HPP_
