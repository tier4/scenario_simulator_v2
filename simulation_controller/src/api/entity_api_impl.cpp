#include <simulation_controller/api/entity_api_impl.hpp>
#include <simulation_controller/entity/exception.hpp>

namespace scenario_simulator
{
    bool EntityAPIImpl::spawn(bool is_ego, std::string name, std::string catalog_xml, simulation_controller::entity::EntityStatus status)
    {
        XmlRpc::XmlRpcValue value,status_value;
        status_value = toValue(name, status);
        value[0][0]["methodName"] = "spawn_entity";
        value[0][0]["params"] = status_value;
        value[0][0]["params"]["entity/is_ego"] = is_ego;
        value[0][0]["params"]["entity/catalog_xml"] = catalog_xml;

        pugi::xml_document catalog_xml_doc;
        catalog_xml_doc.load_string(catalog_xml.c_str());
        pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
        //catalog_xml_doc.has("Vehicle");
        if(vehicle_node !=NULL)
        {
            if(is_ego)
            {
                simulation_controller::entity::EgoEntity ego(name, status, catalog_xml_doc);
                if(!entity_manager_ptr_->spawnEntity(ego))
                {
                    return false;
                }
            }
            else
            {
                simulation_controller::entity::VehicleEntity npc(name, status, catalog_xml_doc);
                if(!entity_manager_ptr_->spawnEntity(npc))
                {
                    return false;
                }
            }
        }
        pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
        if(pedestrian_node !=NULL)
        {
            simulation_controller::entity::PedestrianEntity pedestrian(name, status, catalog_xml_doc);
            if(!entity_manager_ptr_->spawnEntity(pedestrian))
            {
                return false;
            }
        }

        XmlRpc::XmlRpcValue result;
        try
        {
            client_ptr_->execute("system.multicall", value, result);
        }
        catch(XmlRpc::XmlRpcException e)
        {
            throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
        }
        return result[0][0]["success"];
    }

    bool EntityAPIImpl::spawn(bool is_ego, std::string name,
        simulation_controller::entity::VehicleParameters params,
        simulation_controller::entity::EntityStatus status)
    {
        return spawn(is_ego, name, params.toXml(), status);
    }

    boost::optional<double> EntityAPIImpl::getStandStillDuration(std::string name) const
    {
        return entity_manager_ptr_->getStandStillDuration(name);
    }

    bool EntityAPIImpl::spawn(bool is_ego, std::string name,
        simulation_controller::entity::PedestrianParameters params,
        simulation_controller::entity::EntityStatus status)
    {
        return spawn(is_ego, name, params.toXml(), status);
    }

    bool EntityAPIImpl::spawn(bool is_ego, std::string name, std::string catalog_xml)
    {
        XmlRpc::XmlRpcValue value;
        value[0][0]["methodName"] = "spawn_entity";
        value[0][0]["params"]["entity/is_ego"] = is_ego;
        value[0][0]["params"]["entity/catalog_xml"] = catalog_xml;

        pugi::xml_document catalog_xml_doc;
        catalog_xml_doc.load_string(catalog_xml.c_str());
        pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
        //catalog_xml_doc.has("Vehicle");
        if(vehicle_node !=NULL)
        {
            if(is_ego)
            {
                simulation_controller::entity::EgoEntity ego(name, catalog_xml_doc);
                if(!entity_manager_ptr_->spawnEntity(ego))
                {
                    return false;
                }
            }
            else
            {
                simulation_controller::entity::VehicleEntity npc(name, catalog_xml_doc);
                if(!entity_manager_ptr_->spawnEntity(npc))
                {
                    return false;
                }
            }
        }
        pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
        if(pedestrian_node !=NULL)
        {
            simulation_controller::entity::PedestrianEntity pedestrian(name, catalog_xml_doc);
            if(!entity_manager_ptr_->spawnEntity(pedestrian))
            {
                return false;
            }
        }

        XmlRpc::XmlRpcValue result;
        try
        {
            client_ptr_->execute("system.multicall", value, result);
        }
        catch(XmlRpc::XmlRpcException e)
        {
            throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
        }
        return result[0][0]["success"];
    }

    bool EntityAPIImpl::spawn(bool is_ego, std::string name,
        simulation_controller::entity::VehicleParameters params)
    {
        return spawn(is_ego, name, params.toXml());
    }

    bool EntityAPIImpl::spawn(bool is_ego, std::string name,
        simulation_controller::entity::PedestrianParameters params)
    {
        return spawn(is_ego, name, params.toXml());
    }

    boost::optional<double> EntityAPIImpl::getTimeHeadway(std::string from, std::string to)
    {
        if(!entity_manager_ptr_->entityStatusSetted(from) || !entity_manager_ptr_->entityStatusSetted(to))
        {
            return boost::none;
        }
        geometry_msgs::Pose pose = getRelativePose(from, to);
        if(pose.position.x > 0)
        {
            return boost::none;
        }
        simulation_controller::entity::EntityStatus to_status = getEntityStatus(to);
        double ret = (pose.position.x * -1) / (to_status.twist.linear.x);
        if(std::isnan(ret))
        {
            return  std::numeric_limits<double>::infinity();
        }
        return ret;
    }

    void EntityAPIImpl::setVerbose(bool verbose)
    {
        entity_manager_ptr_->setVerbose(verbose);
    }

    bool EntityAPIImpl::setEntityStatus(std::string name, const simulation_controller::entity::EntityStatus & status)
    {
        return entity_manager_ptr_->setEntityStatus(name, status);
    }

    geometry_msgs::Pose EntityAPIImpl::getRelativePose(std::string from, std::string to)
    {
        return entity_manager_ptr_->getRelativePose(from, to);
    }

    geometry_msgs::Pose EntityAPIImpl::getRelativePose(geometry_msgs::Pose from, std::string to)
    {
        return entity_manager_ptr_->getRelativePose(from, to);
    }

    geometry_msgs::Pose EntityAPIImpl::getRelativePose(std::string from, geometry_msgs::Pose to)
    {
        return entity_manager_ptr_->getRelativePose(from, to);
    }

    geometry_msgs::Pose EntityAPIImpl::getRelativePose(geometry_msgs::Pose from, geometry_msgs::Pose to)
    {
        return entity_manager_ptr_->getRelativePose(from, to);
    }

    void EntityAPIImpl::setTargetSpeed(std::string name, double target_speed, bool continuous)
    {
        entity_manager_ptr_->setTargetSpeed(name, target_speed, continuous);
        return;
    }

    void EntityAPIImpl::requestLaneChange(std::string name, int to_lanelet_id)
    {
        entity_manager_ptr_->requestLaneChange(name, to_lanelet_id);
        return;
    }

    void EntityAPIImpl::requestLaneChange(std::string name, simulation_controller::entity::Direction direction)
    {
        entity_manager_ptr_->requestLaneChange(name, direction);
        return;
    }

    void EntityAPIImpl::requestAcquirePosition(std::string name, int lanelet_id, double s, double offset)
    {
        entity_manager_ptr_->requestAcquirePosition(name, lanelet_id, s, offset);
        return;
    }

    bool EntityAPIImpl::reachPosition(std::string name, geometry_msgs::Pose target_pose, double tolerance)
    {
        if(!entity_manager_ptr_->entityStatusSetted(name))
        {
            return false;
        }
        return entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
    }

    bool EntityAPIImpl::reachPosition(std::string name, int lanelet_id, double s, double offset, double tolerance)
    {
        if(!entity_manager_ptr_->entityStatusSetted(name))
        {
            return false;
        }
        return entity_manager_ptr_->reachPosition(name, lanelet_id, s, offset, tolerance);
    }

    bool EntityAPIImpl::isInLanelet(std::string name, int lanelet_id)
    {
        if(!entity_manager_ptr_->entityStatusSetted(name))
        {
            return false;
        }
        try
        {
            simulation_controller::entity::EntityStatus status = getEntityStatus(name,simulation_controller::entity::CoordinateFrameTypes::LANE);
            if(status.lanelet_id == lanelet_id)
            {
                return true;
            }
        }
        catch(simulation_controller::SimulationRuntimeError)
        {
            return false;
        }
        return false;
    }

    simulation_controller::entity::EntityStatus EntityAPIImpl::getEntityStatus(std::string name,
        simulation_controller::entity::CoordinateFrameTypes corrdinate)
    {
        auto status = entity_manager_ptr_->getEntityStatus(name, corrdinate);
        if(!status)
        {
            throw simulation_controller::SimulationRuntimeError("error occurs while getting entity stauts, target entity : " + name);
        }
        return status.get();
    }

    boost::optional<double> EntityAPIImpl::getLongitudinalDistance(std::string from, std::string to)
    {
        return entity_manager_ptr_->getLongitudinalDistance(from, to);
    }

    XmlRpc::XmlRpcValue EntityAPIImpl::toValue(std::string name, simulation_controller::entity::EntityStatus status)
    {
        if(status.coordinate == simulation_controller::entity::CoordinateFrameTypes::WORLD)
        {
            XmlRpc::XmlRpcValue param;
            param["entity/name"] = name;
            param["coordinate"] = "world";
            param["pose/position/x"] = status.pose.position.x;
            param["pose/position/y"] = status.pose.position.y;
            param["pose/position/z"] = status.pose.position.z;
            param["pose/orientation/x"] = status.pose.orientation.x;
            param["pose/orientation/y"] = status.pose.orientation.y;
            param["pose/orientation/z"] = status.pose.orientation.z;
            param["pose/orientation/w"] = status.pose.orientation.w;
            param["twist/linear/x"] = status.twist.linear.x;
            param["twist/linear/y"] = status.twist.linear.y;
            param["twist/linear/z"] = status.twist.linear.z;
            param["twist/angular/x"] = status.twist.angular.x;
            param["twist/angular/y"] = status.twist.angular.y;
            param["twist/angular/z"] = status.twist.angular.z;
            param["accel/linear/x"] = status.accel.linear.x;
            param["accel/linear/y"] = status.accel.linear.y;
            param["accel/linear/z"] = status.accel.linear.z;
            param["accel/angular/x"] = status.accel.angular.x;
            param["accel/angular/y"] = status.accel.angular.y;
            param["accel/angular/z"] = status.accel.angular.z;
            param["time"] = status.time;
            return param;
        }
        if(status.coordinate == simulation_controller::entity::CoordinateFrameTypes::LANE)
        {
            XmlRpc::XmlRpcValue param;
            param["entity/name"] = name;
            param["coordinate"] = "lane";
            param["lanelet_id"] = status.lanelet_id;
            param["s"] = status.s;
            param["offset"] = status.offset;
            param["roll"] = status.rpy.x;
            param["pitch"] = status.rpy.y;
            param["yaw"] = status.rpy.z;
            param["twist/linear/x"] = status.twist.linear.x;
            param["twist/linear/y"] = status.twist.linear.y;
            param["twist/linear/z"] = status.twist.linear.z;
            param["twist/angular/x"] = status.twist.angular.x;
            param["twist/angular/y"] = status.twist.angular.y;
            param["twist/angular/z"] = status.twist.angular.z;
            param["accel/linear/x"] = status.accel.linear.x;
            param["accel/linear/y"] = status.accel.linear.y;
            param["accel/linear/z"] = status.accel.linear.z;
            param["accel/angular/x"] = status.accel.angular.x;
            param["accel/angular/y"] = status.accel.angular.y;
            param["accel/angular/z"] = status.accel.angular.z;
            param["time"] = status.time;
            return param;
        }
    }

    simulation_controller::entity::EntityStatus EntityAPIImpl::toStatus(XmlRpc::XmlRpcValue param)
    {
        std::string coordinate = param["coordinate"];
        std::string name = param["entity/name"];
        geometry_msgs::Pose pose;
        if(coordinate == "lane")
        {
            int lanelet_id = param["lanelet_id"];
            double s = param["s"];
            double offset = param["offset"];
            geometry_msgs::Vector3 rpy;
            rpy.x = param["roll"];
            rpy.y = param["pitch"];
            rpy.z = param["yaw"];
            geometry_msgs::Twist twist;
            twist.linear.x = param["twist/linear/x"];
            twist.linear.y = param["twist/linear/y"];
            twist.linear.z = param["twist/linear/z"];
            twist.angular.x = param["twist/angular/x"];
            twist.angular.y = param["twist/angular/y"];
            twist.angular.z = param["twist/angular/z"];
            geometry_msgs::Accel accel;
            accel.linear.x = param["accel/linear/x"];
            accel.linear.y = param["accel/linear/y"];
            accel.linear.z = param["accel/linear/z"];
            accel.angular.x = param["accel/angular/x"];
            accel.angular.y = param["accel/angular/y"];
            accel.angular.z = param["accel/angular/z"];
            double time = param["time"];
            simulation_controller::entity::EntityStatus status(time, lanelet_id, s, offset, rpy, twist, accel);
            return status;
        }
        if(coordinate == "world")
        {
            pose.position.x = param["pose/position/x"];
            pose.position.y = param["pose/position/y"];
            pose.position.z = param["pose/position/z"];
            if(param.hasMember("pose/orientation/x") ||
                param.hasMember("pose/orientation/y") ||
                param.hasMember("pose/orientation/z") ||
                param.hasMember("pose/orientation/w"))
            {
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;
            }
            else
            {
                pose.orientation.x = param["pose/orientation/x"];
                pose.orientation.y = param["pose/orientation/y"];
                pose.orientation.z = param["pose/orientation/z"];
                pose.orientation.w = param["pose/orientation/w"];
            }
            geometry_msgs::Twist twist;
            twist.linear.x = param["twist/linear/x"];
            twist.linear.y = param["twist/linear/y"];
            twist.linear.z = param["twist/linear/z"];
            twist.angular.x = param["twist/angular/x"];
            twist.angular.y = param["twist/angular/y"];
            twist.angular.z = param["twist/angular/z"];
            geometry_msgs::Accel accel;
            accel.linear.x = param["accel/linear/x"];
            accel.linear.y = param["accel/linear/y"];
            accel.linear.z = param["accel/linear/z"];
            accel.angular.x = param["accel/angular/x"];
            accel.angular.y = param["accel/angular/y"];
            accel.angular.z = param["accel/angular/z"];
            double time = param["time"];
            simulation_controller::entity::EntityStatus status(time, pose, twist, accel);
            return status;
        }
    }
}
