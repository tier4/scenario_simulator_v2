#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/dynamic_constraints.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>
#include <traffic_simulator_msgs/msg/vertex.hpp>
#include <traffic_simulator_msgs/msg/polyline.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <traffic_simulator/behavior/follow_trajectory.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <geometry/quaternion/direction_to_quaternion.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

class TrajectoryStatusDemoNode : public rclcpp::Node
{
public:
    TrajectoryStatusDemoNode() : Node("trajectory_status_demo_node")
    {
        RCLCPP_INFO(this->get_logger(), "TrajectoryStatusDemo node starting...");

        declareParameters();

        // Get time configuration and setup timer
        auto timer_interval_ms = this->get_parameter("time.timer_interval_ms").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms),
            std::bind(&TrajectoryStatusDemoNode::runDemo, this)
        );

        simulation_time_ = this->get_parameter("time.initial_time").as_double();
        traffic_simulator::lanelet_map::activate("/home/dmoszynski/robotecai/autoware/sources/autoware_maps/prd_jt_cicd_virtual_A’_dev/lanelet2_map.osm");

        // Get package source directory and create CSV file in src folder
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("trajectory_status_demo");
        std::string package_src_dir = package_share_dir + "/../../../../src/simulator/scenario_simulator/simulation/trajectory_status_demo/src";
        std::string csv_path = package_src_dir + "/trajectory_data.csv";

        RCLCPP_INFO(this->get_logger(), "Creating CSV file at: %s", csv_path.c_str());
        csv_file_.open(csv_path);
        csv_file_ << "time,pos_x,pos_y,pos_z,velocity,acceleration,remaining_distance\n";
    }

private:

    std::ofstream csv_file_;
    traffic_simulator_msgs::msg::EntityStatus status_;
    traffic_simulator_msgs::msg::PolylineTrajectory polyline_trajectory_;
    bool once = true;

    void runDemo()
    {
        try {
          // Get simulation parameters
          auto step_time = 1.0/30.0;
          auto matching_distance = this->get_parameter("matching_distance").as_double();
        //   auto target_speed = this->get_parameter("target_speed").as_double();

          if (once) {
            // Create EntityStatus from configuration
            status_ = createEntityStatusFromConfig();

            // Create PolylineTrajectory from configuration
            polyline_trajectory_ = createPolylineTrajectoryFromConfig();
            once = false;
          }

            // Create BehaviorParameter from configuration
            auto behavior_parameter = createBehaviorParameterFromConfig();


            // Call the makeUpdatedStatus API
            auto updated_status = traffic_simulator::follow_trajectory::makeUpdatedStatus(
                status_,
                polyline_trajectory_,
                behavior_parameter,
                step_time,
                matching_distance,
                status_.action_status.twist.linear.x
            );

            if (updated_status.has_value()) {
              const auto canonicalized_lanelet_pose =
                traffic_simulator::pose::toCanonicalizedLaneletPose(
                  updated_status->pose, updated_status->bounding_box, {}, false, matching_distance);
              updated_status = static_cast<traffic_simulator_msgs::msg::EntityStatus>(
                traffic_simulator::entity_status::CanonicalizedEntityStatus(
                  updated_status.value(), canonicalized_lanelet_pose));

              // Calculate remaining distance to waypoint
              double remaining_distance = 0.0;
              if (!polyline_trajectory_.shape.vertices.empty()) {
                const auto & waypoint =
                  polyline_trajectory_.shape.vertices.front().position.position;
                double dx = waypoint.x - updated_status->pose.position.x;
                double dy = waypoint.y - updated_status->pose.position.y;
                double dz = waypoint.z - updated_status->pose.position.z;
                remaining_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
              }

              csv_file_ << updated_status->time << "," << updated_status->pose.position.x << ","
                        << updated_status->pose.position.y << "," << updated_status->pose.position.z
                        << "," << updated_status->action_status.twist.linear.x << ","
                        << updated_status->action_status.accel.linear.x << "," << remaining_distance
                        << "\n";
              csv_file_.flush();

              // Update simulation time for next iteration
              status_ = *updated_status;
              simulation_time_ = updated_status->time;

            } else {
              RCLCPP_WARN(this->get_logger(), "makeUpdatedStatus returned no value");
              rclcpp::shutdown();
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in runDemo: %s", e.what());
            timer_->cancel();
        }
    }

    void declareParameters()
    {
        // Simulation parameters
        this->declare_parameter<double>("matching_distance", 1.0);
        this->declare_parameter<double>("target_speed", 15.0);

        // Time configuration
        this->declare_parameter<double>("time.initial_time", 0.0);
        this->declare_parameter<int>("time.timer_interval_ms", 33);

        // Entity parameters
        this->declare_parameter<std::string>("entity.name", "demo_vehicle");
        this->declare_parameter<int>("entity.type", 1);
        this->declare_parameter<int>("entity.subtype", 0);

        // Entity pose
        this->declare_parameter<double>("entity.pose.position.x", 0.0);
        this->declare_parameter<double>("entity.pose.position.y", 0.0);
        this->declare_parameter<double>("entity.pose.position.z", 0.0);
        this->declare_parameter<double>("entity.pose.orientation.x", 0.0);
        this->declare_parameter<double>("entity.pose.orientation.y", 0.0);
        this->declare_parameter<double>("entity.pose.orientation.z", 0.0);
        this->declare_parameter<double>("entity.pose.orientation.w", 1.0);

        // Entity bounding box
        this->declare_parameter<double>("entity.bounding_box.center.x", 0.0);
        this->declare_parameter<double>("entity.bounding_box.center.y", 0.0);
        this->declare_parameter<double>("entity.bounding_box.center.z", 0.0);
        this->declare_parameter<double>("entity.bounding_box.dimensions.x", 4.5);
        this->declare_parameter<double>("entity.bounding_box.dimensions.y", 2.0);
        this->declare_parameter<double>("entity.bounding_box.dimensions.z", 1.5);

        // Entity action status
        this->declare_parameter<double>("entity.action_status.twist.linear.x", 10.0);
        this->declare_parameter<double>("entity.action_status.twist.linear.y", 0.0);
        this->declare_parameter<double>("entity.action_status.twist.linear.z", 0.0);
        this->declare_parameter<double>("entity.action_status.twist.angular.x", 0.0);
        this->declare_parameter<double>("entity.action_status.twist.angular.y", 0.0);
        this->declare_parameter<double>("entity.action_status.twist.angular.z", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.linear.x", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.linear.y", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.linear.z", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.angular.x", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.angular.y", 0.0);
        this->declare_parameter<double>("entity.action_status.accel.angular.z", 0.0);

        // Entity lanelet pose
        this->declare_parameter<bool>("entity.lanelet_pose_valid", false);
        this->declare_parameter<int>("entity.lanelet_pose.lanelet_id", 0);
        this->declare_parameter<double>("entity.lanelet_pose.s", 0.0);
        this->declare_parameter<double>("entity.lanelet_pose.offset", 0.0);
        this->declare_parameter<double>("entity.lanelet_pose.rpy.x", 0.0);
        this->declare_parameter<double>("entity.lanelet_pose.rpy.y", 0.0);
        this->declare_parameter<double>("entity.lanelet_pose.rpy.z", 0.0);

        // Behavior parameters
        this->declare_parameter<double>("behavior.dynamic_constraints.max_speed", 30.0);
        this->declare_parameter<double>("behavior.dynamic_constraints.max_acceleration", 3.0);
        this->declare_parameter<double>("behavior.dynamic_constraints.max_acceleration_rate", 5.0);
        this->declare_parameter<double>("behavior.dynamic_constraints.max_deceleration", 5.0);
        this->declare_parameter<double>("behavior.dynamic_constraints.max_deceleration_rate", 8.0);
        this->declare_parameter<double>("behavior.lane_change_distance", 20.0);
        this->declare_parameter<double>("behavior.stop_margin", 3.0);
        this->declare_parameter<double>("behavior.follow_distance", 20.0);
        this->declare_parameter<bool>("behavior.see_around", true);

        // Trajectory parameters
        this->declare_parameter<double>("trajectory.initial_distance_offset", 0.0);
        this->declare_parameter<bool>("trajectory.dynamic_constraints_ignorable", true);
        this->declare_parameter<bool>("trajectory.closed", false);
        // Single waypoint parameters
        this->declare_parameter<double>("trajectory.waypoint_x", 100.0);
        this->declare_parameter<double>("trajectory.waypoint_y", 0.0);
        this->declare_parameter<double>("trajectory.waypoint_z", 0.0);
        this->declare_parameter<double>("trajectory.base_time", 0.0);
    }

    traffic_simulator_msgs::msg::EntityStatus createEntityStatusFromConfig()
    {
        traffic_simulator_msgs::msg::EntityStatus entity_status;

        // Set entity type and name
        entity_status.name = this->get_parameter("entity.name").as_string();
        entity_status.type.type = this->get_parameter("entity.type").as_int();
        entity_status.subtype.value = this->get_parameter("entity.subtype").as_int();

        // Set time
        entity_status.time = simulation_time_;

        // Set pose
        entity_status.pose.position.x = this->get_parameter("entity.pose.position.x").as_double();
        entity_status.pose.position.y = this->get_parameter("entity.pose.position.y").as_double();
        entity_status.pose.position.z = this->get_parameter("entity.pose.position.z").as_double();
        entity_status.pose.orientation.x = this->get_parameter("entity.pose.orientation.x").as_double();
        entity_status.pose.orientation.y = this->get_parameter("entity.pose.orientation.y").as_double();
        entity_status.pose.orientation.z = this->get_parameter("entity.pose.orientation.z").as_double();
        entity_status.pose.orientation.w = this->get_parameter("entity.pose.orientation.w").as_double();

        // Set bounding box
        entity_status.bounding_box.center.x = this->get_parameter("entity.bounding_box.center.x").as_double();
        entity_status.bounding_box.center.y = this->get_parameter("entity.bounding_box.center.y").as_double();
        entity_status.bounding_box.center.z = this->get_parameter("entity.bounding_box.center.z").as_double();
        entity_status.bounding_box.dimensions.x = this->get_parameter("entity.bounding_box.dimensions.x").as_double();
        entity_status.bounding_box.dimensions.y = this->get_parameter("entity.bounding_box.dimensions.y").as_double();
        entity_status.bounding_box.dimensions.z = this->get_parameter("entity.bounding_box.dimensions.z").as_double();

        // Set action status (velocity and acceleration)
        entity_status.action_status.twist.linear.x = this->get_parameter("entity.action_status.twist.linear.x").as_double();
        entity_status.action_status.twist.linear.y = this->get_parameter("entity.action_status.twist.linear.y").as_double();
        entity_status.action_status.twist.linear.z = this->get_parameter("entity.action_status.twist.linear.z").as_double();
        entity_status.action_status.twist.angular.x = this->get_parameter("entity.action_status.twist.angular.x").as_double();
        entity_status.action_status.twist.angular.y = this->get_parameter("entity.action_status.twist.angular.y").as_double();
        entity_status.action_status.twist.angular.z = this->get_parameter("entity.action_status.twist.angular.z").as_double();

        entity_status.action_status.accel.linear.x = this->get_parameter("entity.action_status.accel.linear.x").as_double();
        entity_status.action_status.accel.linear.y = this->get_parameter("entity.action_status.accel.linear.y").as_double();
        entity_status.action_status.accel.linear.z = this->get_parameter("entity.action_status.accel.linear.z").as_double();
        entity_status.action_status.accel.angular.x = this->get_parameter("entity.action_status.accel.angular.x").as_double();
        entity_status.action_status.accel.angular.y = this->get_parameter("entity.action_status.accel.angular.y").as_double();
        entity_status.action_status.accel.angular.z = this->get_parameter("entity.action_status.accel.angular.z").as_double();

        // Set lanelet pose
        entity_status.lanelet_pose_valid = this->get_parameter("entity.lanelet_pose_valid").as_bool();
        entity_status.lanelet_pose.lanelet_id = this->get_parameter("entity.lanelet_pose.lanelet_id").as_int();
        entity_status.lanelet_pose.s = this->get_parameter("entity.lanelet_pose.s").as_double();
        entity_status.lanelet_pose.offset = this->get_parameter("entity.lanelet_pose.offset").as_double();
        entity_status.lanelet_pose.rpy.x = this->get_parameter("entity.lanelet_pose.rpy.x").as_double();
        entity_status.lanelet_pose.rpy.y = this->get_parameter("entity.lanelet_pose.rpy.y").as_double();
        entity_status.lanelet_pose.rpy.z = this->get_parameter("entity.lanelet_pose.rpy.z").as_double();

        return entity_status;
    }

    traffic_simulator_msgs::msg::PolylineTrajectory createPolylineTrajectoryFromConfig()
    {
        traffic_simulator_msgs::msg::PolylineTrajectory trajectory;

        // Set trajectory parameters from config
        trajectory.initial_distance_offset = this->get_parameter("trajectory.initial_distance_offset").as_double();
        trajectory.dynamic_constraints_ignorable = this->get_parameter("trajectory.dynamic_constraints_ignorable").as_bool();
        trajectory.base_time = std::numeric_limits<double>::quiet_NaN(); // Absolute timing
        trajectory.closed = this->get_parameter("trajectory.closed").as_bool();

        // Create waypoints from config
        traffic_simulator_msgs::msg::Vertex waypoint;
        waypoint.position.position.x = this->get_parameter("trajectory.waypoint_x").as_double();
        waypoint.position.position.y = this->get_parameter("trajectory.waypoint_y").as_double();
        waypoint.position.position.z = this->get_parameter("trajectory.waypoint_z").as_double();
        waypoint.time = std::numeric_limits<double>::quiet_NaN(); // No specific arrival time

        trajectory.shape.vertices.push_back(waypoint);

        return trajectory;
    }

    traffic_simulator_msgs::msg::BehaviorParameter createBehaviorParameterFromConfig()
    {
        traffic_simulator_msgs::msg::BehaviorParameter behavior_param;

        // Set dynamic constraints from config
        behavior_param.dynamic_constraints.max_speed = this->get_parameter("behavior.dynamic_constraints.max_speed").as_double();
        behavior_param.dynamic_constraints.max_acceleration = this->get_parameter("behavior.dynamic_constraints.max_acceleration").as_double();
        behavior_param.dynamic_constraints.max_acceleration_rate = this->get_parameter("behavior.dynamic_constraints.max_acceleration_rate").as_double();
        behavior_param.dynamic_constraints.max_deceleration = this->get_parameter("behavior.dynamic_constraints.max_deceleration").as_double();
        behavior_param.dynamic_constraints.max_deceleration_rate = this->get_parameter("behavior.dynamic_constraints.max_deceleration_rate").as_double();

        // Set other behavior parameters from config
        behavior_param.lane_change_distance = this->get_parameter("behavior.lane_change_distance").as_double();
        behavior_param.stop_margin = this->get_parameter("behavior.stop_margin").as_double();
        behavior_param.follow_distance = this->get_parameter("behavior.follow_distance").as_double();
        behavior_param.see_around = this->get_parameter("behavior.see_around").as_bool();

        return behavior_param;
    }

    void updateEntityParameters(const traffic_simulator::EntityStatus& updated_status)
    {
        // Update entity position parameters for next iteration
        this->set_parameter(rclcpp::Parameter("entity.pose.position.x", updated_status.pose.position.x));
        this->set_parameter(rclcpp::Parameter("entity.pose.position.y", updated_status.pose.position.y));
        this->set_parameter(rclcpp::Parameter("entity.pose.position.z", updated_status.pose.position.z));
        this->set_parameter(rclcpp::Parameter("entity.pose.orientation.x", updated_status.pose.orientation.x));
        this->set_parameter(rclcpp::Parameter("entity.pose.orientation.y", updated_status.pose.orientation.y));
        this->set_parameter(rclcpp::Parameter("entity.pose.orientation.z", updated_status.pose.orientation.z));
        this->set_parameter(rclcpp::Parameter("entity.pose.orientation.w", updated_status.pose.orientation.w));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.linear.x", updated_status.action_status.twist.linear.x));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.linear.y", updated_status.action_status.twist.linear.y));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.linear.z", updated_status.action_status.twist.linear.z));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.angular.x", updated_status.action_status.twist.angular.x));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.angular.y", updated_status.action_status.twist.angular.y));
        this->set_parameter(rclcpp::Parameter("entity.action_status.twist.angular.z", updated_status.action_status.twist.angular.z));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    double simulation_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TrajectoryStatusDemoNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}