#ifndef SIMULATION_CONTROLLER__XMLRPC_WRAPPER__ENTITY_API_IMPL_HPP_
#define SIMULATION_CONTROLLER__XMLRPC_WRAPPER__ENTITY_API_IMPL_HPP_

#include <simulation_controller/api/api_impl_base.hpp>
#include <simulation_controller/entity/vehicle_entity.hpp>
#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/entity/vehicle_parameter.hpp>
#include <simulation_controller/entity/entity_manager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <memory>
#include <boost/optional.hpp>

namespace scenario_simulator
{
    class EntityAPIImpl : ApiImplBase
    {
    public:
        EntityAPIImpl(std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr, std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr)
        : ApiImplBase(client_ptr, entity_manager_ptr) {};
        bool spawn(bool is_ego, std::string name, 
            std::string catalog_xml,
            simulation_controller::entity::EntityStatus status);
        bool spawn(bool is_ego, std::string name,
            simulation_controller::entity::VehicleParameters params,
            simulation_controller::entity::EntityStatus status);
        bool spawn(bool is_ego, std::string name,
            simulation_controller::entity::PedestrianParameters params,
            simulation_controller::entity::EntityStatus status);
        bool spawn(bool is_ego, std::string name, 
            std::string catalog_xml);
        bool spawn(bool is_ego, std::string name,
            simulation_controller::entity::VehicleParameters params);
        bool spawn(bool is_ego, std::string name,
            simulation_controller::entity::PedestrianParameters params);
        simulation_controller::entity::EntityStatus getEntityStatus(std::string name,
            simulation_controller::entity::CoordinateFrameTypes corrdinate = simulation_controller::entity::CoordinateFrameTypes::LANE);
        bool setEntityStatus(std::string name, const simulation_controller::entity::EntityStatus & status);
        boost::optional<double> getLongitudinalDistance(std::string from, std::string to);
        boost::optional<double> getTimeHeadway(std::string from, std::string to);
        void requestAcquirePosition(std::string name, int lanelet_id, double s, double offset);
        void requestLaneChange(std::string name, int to_lanelet_id);
        void requestLaneChange(std::string name, simulation_controller::entity::Direction direction);
        bool isInLanelet(std::string name, int lanelet_id);
        void setTargetSpeed(std::string name, double target_speed, bool continuous);
        geometry_msgs::msg::Pose getRelativePose(std::string from, std::string to);
        geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, std::string to);
        geometry_msgs::msg::Pose getRelativePose(std::string from, geometry_msgs::msg::Pose to);
        geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, geometry_msgs::msg::Pose to);
        bool reachPosition(std::string name, geometry_msgs::msg::Pose target_pose, double tolerance);
        bool reachPosition(std::string name, int lanelet_id, double s, double offset, double tolerance);
        void setVerbose(bool verbose);
        boost::optional<double> getStandStillDuration(std::string name) const;
    private:
        simulation_controller::entity::EntityStatus toStatus(XmlRpc::XmlRpcValue param);
        XmlRpc::XmlRpcValue toValue(std::string name, simulation_controller::entity::EntityStatus status);
    };
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__ENTITY_API_IMPL_HPP_
