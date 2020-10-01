#include <ros/ros.h>
#include <scenario_simulator/scenario_simulator.hpp>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "scenario_simulator_node");
  scenario_simulator::ScenarioSimulator sim;
  ros::spin();
  return 0;
}
