#ifndef  SIMULATOR_CONTROLLER__ENTITY_MANAGER_HPP_
#define  SIMULATOR_CONTROLLER__ENTITY_MANAGER_HPP_

#include <simulation_controller/entity/ego_entity.hpp>
#include <simulation_controller/entity/vehicle_entity.hpp>
#include <simulation_controller/entity/pedestrian_entity.hpp>
#include <simulation_controller/entity/exception.hpp>

#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
            bool without_lanelet_;
        public:
            void setVerbose(bool verbose);
            void requestAcquirePosition(std::string name, int lanelet_id, double s, double offset);
            void requestLaneChange(std::string name, int to_lanelet_id);
            void requestLaneChange(std::string name, Direction direction);
            boost::optional<double> getLongitudinalDistance(std::string from, std::string to, double max_distance = 100);
            geometry_msgs::Pose getRelativePose(std::string from, std::string to);
            geometry_msgs::Pose getRelativePose(std::string from, geometry_msgs::Pose to);
            geometry_msgs::Pose getRelativePose(geometry_msgs::Pose from, std::string to);
            geometry_msgs::Pose getRelativePose(geometry_msgs::Pose from, geometry_msgs::Pose to) const;
            explicit EntityManager(bool without_lanelet=false);
            boost::optional<VehicleParameters> getVehicleParameters(std::string name);
            std::vector<std::string> getEntityNames();
            std::vector<std::string> getVehicleEntityNames();
            std::vector<std::string> getEgoEntityNames();
            visualization_msgs::MarkerArray generateMarker();
            bool setEntityStatus(std::string name, EntityStatus status);
            CoordinateFrameTypes getEntityStatusCoordinate(std::string name);
            boost::optional<EntityStatus> getEntityStatus(std::string name, CoordinateFrameTypes coordinate = CoordinateFrameTypes::WORLD);
            bool entityStatusSetted(std::string name);
            void setTargetSpeed(std::string name, double target_speed, bool continuous);
            void update(double current_time, double step_time);
            void broadcastTransform(geometry_msgs::PoseStamped pose);
            bool reachPosition(std::string name, geometry_msgs::Pose target_pose, double tolerance);
            bool reachPosition(std::string name, int lanelet_id, double s, double offset, double tolerance);
            void broadcastEntityTransform();
            boost::optional<double> getStandStillDuration(std::string name);
            const std::unordered_map<std::string,EntityType> getEntityTypeList() const;
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
                if(!without_lanelet_)
                {
                    entity.setHdMapUtils(hdmap_utils_ptr_);
                }
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
