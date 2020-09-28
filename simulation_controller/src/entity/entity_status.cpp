#include <simulation_controller/entity/entity_status.hpp>

namespace simulation_controller
{
    namespace entity
    {
        EntityStatus::EntityStatus(double t, 
            geometry_msgs::msg::Pose pose, 
            geometry_msgs::msg::Twist twist, 
            geometry_msgs::msg::Accel accel) : 
                time(t), 
                twist(twist), 
                accel(accel),
                pose(pose)
                {
                    coordinate = WORLD;
                }
        EntityStatus::EntityStatus(double t, 
            int lanelet_id, double s, double offset,
            geometry_msgs::msg::Vector3 rpy,
            geometry_msgs::msg::Twist twist, 
            geometry_msgs::msg::Accel accel) : 
                time(t), 
                twist(twist),
                accel(accel),
                lanelet_id(lanelet_id),
                offset(offset),
                s(s),
                rpy(rpy)
                {
                    coordinate = LANE;
                }
    }  // namespace entity
}  // namespace simulation_controller
