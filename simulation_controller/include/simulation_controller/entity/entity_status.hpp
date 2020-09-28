#ifndef SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_
#define SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

namespace simulation_controller
{
    namespace entity
    {
        enum CoordinateFrameTypes
        {
            WORLD = 0,
            LANE = 1,
        };

        class EntityStatus
        {
        public:
            EntityStatus() = default;
            EntityStatus(double t, 
                geometry_msgs::Pose pose, 
                geometry_msgs::Twist twist, 
                geometry_msgs::Accel accel) : 
                    time(t), 
                    pose(pose), 
                    twist(twist), 
                    accel(accel)
                    {
                        coordinate = WORLD;
                    };
            EntityStatus(double t, 
                int lanelet_id, double s, double offset,
                geometry_msgs::Vector3 rpy,
                geometry_msgs::Twist twist, 
                geometry_msgs::Accel accel) : 
                    time(t), 
                    twist(twist),
                    lanelet_id(lanelet_id),
                    offset(offset),
                    rpy(rpy),
                    s(s),
                    accel(accel)
                    {
                        coordinate = LANE;
                    };
            EntityStatus& operator =(const EntityStatus& obj)
            {
                this->time = obj.time;
                this->coordinate = obj.coordinate;
                this->twist = obj.twist;
                this->accel = obj.accel;
                this->lanelet_id = obj.lanelet_id;
                this->offset = obj.offset;
                this->s = obj.s;
                this->rpy = obj.rpy;
                this->pose = obj.pose;
                return *this;
            }
            double time;
            CoordinateFrameTypes coordinate;
            geometry_msgs::Twist twist;
            geometry_msgs::Accel accel;
            /* Field for Lene Pose */
            int lanelet_id;
            double offset;
            double s;
            geometry_msgs::Vector3 rpy;
            /* Field for world pose */
            geometry_msgs::Pose pose;
        };
    }
}

#endif  // SIMULATION_CONTROLLER___ENTITY_STATUS_HPP_