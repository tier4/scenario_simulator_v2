#ifndef HDMAP_UTILS__HDMAP_UTILS_HPP
#define HDMAP_UTILS__HDMAP_UTILS_HPP

#include <simulation_controller/entity/entity_status.hpp>
#include <simulation_controller/math/hermite_curve.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <geometry_msgs/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/optional.hpp>
#include <map>

namespace hdmap_utils
{
    class HdMapError : public std::runtime_error
    {
    public:
        HdMapError(const char *message, int res=0) : error_info_(res), runtime_error(message) {};
    private:
        int error_info_;
    };

    class HdMapUtils
    {
    public:
        HdMapUtils(std::string lanelet_topic);
        std::vector<geometry_msgs::Point> toMapPoints(int lanelet_id, std::vector<double> s);
        boost::optional<geometry_msgs::PoseStamped> toMapPose(int lanelet_id, double s, double offset, geometry_msgs::Quaternion quat);
        boost::optional<geometry_msgs::PoseStamped> toMapPose(int lanelet_id, double s, double offset, geometry_msgs::Vector3 rpy);
        boost::optional<geometry_msgs::PoseStamped> toMapPose(int lanelet_id, double s, double offset);
        boost::optional<geometry_msgs::PoseStamped> toMapPose(simulation_controller::entity::EntityStatus status);
        std::vector<int> getNextLaneletIds(int lanelet_id, std::string turn_direction);
        std::vector<int> getNextLaneletIds(int lanelet_id) const;
        std::vector<int> getPreviousLaneletIds(int lanelet_id) const;
        boost::optional<int> getLaneChangeableLenletId(int lanlet_id, std::string direction);
        double getLaneletLength(int lanelet_id) const;
        bool isInLanelet(int lanelet_id, double s);
        boost::optional<double> getLongitudinalDistance(int from_lanelet_id, double from_s, int to_lanelet_id, double to_s);
        double getSpeedLimit(std::vector<int> lanelet_ids);
        std::vector<int> getFollowingLanelets(int lanelet_id, double distance = 100);
        std::vector<geometry_msgs::Point> getCenterPoints(int lanelet_id);
        std::vector<geometry_msgs::Point> clipTrajectoryFromLaneletIds(int lanelet_id, double s, 
            std::vector<int> lanelet_ids, double foward_distance=20);
        bool canChangeLane(int from_lanlet_id, int to_lanelet_id);
        boost::optional<std::pair<simulation_controller::math::HermiteCurve,double>> getLaneChangeTrajectory(geometry_msgs::Pose from_pose, int to_lanelet_id);
        boost::optional<simulation_controller::math::HermiteCurve> getLaneChangeTrajectory(geometry_msgs::Pose from_pose, 
            int to_lanelet_id, double to_s, double tangent_vector_size=100);
        boost::optional<geometry_msgs::Vector3> getTangentVector(int lanelet_id, double s);
        std::vector<int> getRoute(int from_lanelet_id, int to_lanelet_id);
        std::vector<int> getConflictingCrosswalkIds(std::vector<int> lanlet_ids) const;
        boost::optional<double> getCollisionPointInLaneCoordinate(int lanelet_id, int crossing_lanelet_id);
    private:
        geometry_msgs::Vector3 getVectorFromPose(geometry_msgs::Pose pose, double magnitude);
        void mapCallback(const autoware_lanelet2_msgs::MapBin & msg);
        lanelet::LaneletMapPtr lanelet_map_ptr_;
        lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
        lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
        lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
        lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
        lanelet::routing::RoutingGraphContainerUPtr overall_graphs_ptr_;
        double getTrajectoryLength(std::vector<geometry_msgs::Point> trajectory);
        std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z);
    };
}

#endif  // HDMAP_UTILS__HDMAP_UTILS_HPP
