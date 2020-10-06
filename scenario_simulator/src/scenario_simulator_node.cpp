#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator/scenario_simulator.hpp>

int main(int argc, char * argv[])
{
  // ros::init(argc, argv, "scenario_simulator_node");
  // scenario_simulator::ScenarioSimulator sim;
  // ros::spin();
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<scenario_simulator::ScenarioSimulator>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
