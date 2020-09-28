#ifndef SIMULATION_CONTROLLER__MATH__HERMITE_CURVE_HPP
#define SIMULATION_CONTROLLER__MATH__HERMITE_CURVE_HPP

#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <vector>

namespace simulation_controller
{
    namespace math
    {
        class HermiteCurve
        {
        private:
            double ax_,bx_,cx_,dx_;
            double ay_,by_,cy_,dy_;
            double az_,bz_,cz_,dz_;
        public:
            HermiteCurve(geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose goal_pose,
                geometry_msgs::msg::Vector3 start_vec, geometry_msgs::msg::Vector3 goal_vec);
            HermiteCurve(double ax, double bx, double cx, double dx, 
                double ay, double by, double cy, double dy, 
                double az, double bz, double cz, double dz);
            std::vector<geometry_msgs::msg::Point> getTrajectory();
            geometry_msgs::msg::Pose getPose(double s,bool autoscale=false);
            geometry_msgs::msg::Point getPoint(double s,bool autoscale=false);
            geometry_msgs::msg::Vector3 getTangentVector(double s,bool autoscale=false);
            double get2DCurvature(double s,bool autoscale=false);
            double getMaximu2DCurvature();
            double getLength();
        };
    }
}

#endif  // SIMULATION_CONTROLLER__MATH__HERMITE_CURVE_HPP