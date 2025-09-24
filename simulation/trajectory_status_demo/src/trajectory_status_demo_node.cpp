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

class TrajectoryStatusDemoNode : public rclcpp::Node
{
public:
    TrajectoryStatusDemoNode() : Node("trajectory_status_demo_node")
    {
        RCLCPP_INFO(this->get_logger(), "TrajectoryStatusDemo node starting...");

        // Parameters for initial conditions
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_z", 0.0);
        this->declare_parameter<double>("initial_speed", 10.0);
        this->declare_parameter<double>("step_time", 0.1);
        this->declare_parameter<double>("matching_distance", 1.0);
        this->declare_parameter<double>("target_speed", 15.0);
        this->declare_parameter<double>("waypoint_x", 100.0);
        this->declare_parameter<double>("waypoint_y", 0.0);
        this->declare_parameter<double>("waypoint_z", 0.0);

        // Timer to run the demo periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TrajectoryStatusDemoNode::runDemo, this)
        );

        simulation_time_ = 0.0;
    }

private:
    void runDemo()
    {
        try {
            // Get parameters
            auto initial_x = this->get_parameter("initial_x").as_double();
            auto initial_y = this->get_parameter("initial_y").as_double();
            auto initial_z = this->get_parameter("initial_z").as_double();
            auto initial_speed = this->get_parameter("initial_speed").as_double();
            auto step_time = this->get_parameter("step_time").as_double();
            auto matching_distance = this->get_parameter("matching_distance").as_double();
            auto target_speed = this->get_parameter("target_speed").as_double();
            auto waypoint_x = this->get_parameter("waypoint_x").as_double();
            auto waypoint_y = this->get_parameter("waypoint_y").as_double();
            auto waypoint_z = this->get_parameter("waypoint_z").as_double();

            // Create EntityStatus with initial conditions
            auto entity_status = createEntityStatus(
                initial_x, initial_y, initial_z, initial_speed, simulation_time_
            );

            // Create PolylineTrajectory with waypoint
            auto polyline_trajectory = createPolylineTrajectory(
                waypoint_x, waypoint_y, waypoint_z
            );

            // Create BehaviorParameter
            auto behavior_parameter = createBehaviorParameter();

            RCLCPP_INFO(this->get_logger(),
                "Calling makeUpdatedStatus with initial position: [%.2f, %.2f, %.2f], speed: %.2f m/s",
                initial_x, initial_y, initial_z, initial_speed);

            // Call the makeUpdatedStatus API
            auto updated_status = traffic_simulator::follow_trajectory::makeUpdatedStatus(
                entity_status,
                polyline_trajectory,
                behavior_parameter,
                step_time,
                matching_distance,
                target_speed
            );

            if (updated_status.has_value()) {
                RCLCPP_INFO(this->get_logger(),
                    "Updated status - Position: [%.2f, %.2f, %.2f], Speed: %.2f m/s, Time: %.2f s",
                    updated_status->pose.position.x,
                    updated_status->pose.position.y,
                    updated_status->pose.position.z,
                    updated_status->action_status.twist.linear.x,
                    updated_status->time);

                // Update simulation time for next iteration
                simulation_time_ = updated_status->time;

                // Update parameters with new position for next iteration
                this->set_parameter(rclcpp::Parameter("initial_x", updated_status->pose.position.x));
                this->set_parameter(rclcpp::Parameter("initial_y", updated_status->pose.position.y));
                this->set_parameter(rclcpp::Parameter("initial_z", updated_status->pose.position.z));
                this->set_parameter(rclcpp::Parameter("initial_speed", updated_status->action_status.twist.linear.x));

            } else {
                RCLCPP_WARN(this->get_logger(), "makeUpdatedStatus returned no value");
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in runDemo: %s", e.what());
        }
    }

    traffic_simulator_msgs::msg::EntityStatus createEntityStatus(
        double x, double y, double z, double speed, double time)
    {
        traffic_simulator_msgs::msg::EntityStatus entity_status;

        // Set entity type
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
        entity_status.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;

        // Set time and name
        entity_status.time = time;
        entity_status.name = "demo_vehicle";

        // Set pose
        entity_status.pose.position.x = x;
        entity_status.pose.position.y = y;
        entity_status.pose.position.z = z;
        entity_status.pose.orientation.w = 1.0; // Identity quaternion
        entity_status.pose.orientation.x = 0.0;
        entity_status.pose.orientation.y = 0.0;
        entity_status.pose.orientation.z = 0.0;

        // Set bounding box
        entity_status.bounding_box.center.x = 0.0;
        entity_status.bounding_box.center.y = 0.0;
        entity_status.bounding_box.center.z = 0.0;
        entity_status.bounding_box.dimensions.x = 4.5; // length
        entity_status.bounding_box.dimensions.y = 2.0; // width
        entity_status.bounding_box.dimensions.z = 1.5; // height

        // Set action status (velocity and acceleration)
        entity_status.action_status.twist.linear.x = speed;
        entity_status.action_status.twist.linear.y = 0.0;
        entity_status.action_status.twist.linear.z = 0.0;
        entity_status.action_status.twist.angular.x = 0.0;
        entity_status.action_status.twist.angular.y = 0.0;
        entity_status.action_status.twist.angular.z = 0.0;

        entity_status.action_status.accel.linear.x = 0.0;
        entity_status.action_status.accel.linear.y = 0.0;
        entity_status.action_status.accel.linear.z = 0.0;
        entity_status.action_status.accel.angular.x = 0.0;
        entity_status.action_status.accel.angular.y = 0.0;
        entity_status.action_status.accel.angular.z = 0.0;

        // Set lanelet pose as invalid for this demo
        entity_status.lanelet_pose_valid = false;

        return entity_status;
    }

    traffic_simulator_msgs::msg::PolylineTrajectory createPolylineTrajectory(
        double waypoint_x, double waypoint_y, double waypoint_z)
    {
        traffic_simulator_msgs::msg::PolylineTrajectory trajectory;

        // Set trajectory parameters
        trajectory.initial_distance_offset = 0.0;
        trajectory.dynamic_constraints_ignorable = true; // Use position following mode
        trajectory.base_time = std::numeric_limits<double>::quiet_NaN(); // Absolute timing
        trajectory.closed = false;

        // Create a single waypoint
        traffic_simulator_msgs::msg::Vertex waypoint;
        waypoint.position.position.x = waypoint_x;
        waypoint.position.position.y = waypoint_y;
        waypoint.position.position.z = waypoint_z;
        waypoint.time = std::numeric_limits<double>::quiet_NaN(); // No specific arrival time

        trajectory.shape.vertices.push_back(waypoint);

        return trajectory;
    }

    traffic_simulator_msgs::msg::BehaviorParameter createBehaviorParameter()
    {
        traffic_simulator_msgs::msg::BehaviorParameter behavior_param;

        // Set dynamic constraints
        behavior_param.dynamic_constraints.max_speed = 30.0; // m/s
        behavior_param.dynamic_constraints.max_acceleration = 3.0; // m/s^2
        behavior_param.dynamic_constraints.max_acceleration_rate = 5.0; // m/s^3
        behavior_param.dynamic_constraints.max_deceleration = 5.0; // m/s^2
        behavior_param.dynamic_constraints.max_deceleration_rate = 8.0; // m/s^3

        // Set other behavior parameters
        behavior_param.lane_change_distance = 20.0;
        behavior_param.stop_margin = 3.0;
        behavior_param.follow_distance = 20.0;
        behavior_param.see_around = true;

        return behavior_param;
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