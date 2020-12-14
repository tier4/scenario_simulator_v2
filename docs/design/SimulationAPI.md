# Simulation API

Simulation API provides C++ API to control NPC behavior in simulation.
Sample code is below.

## minimal example
```c++
#include <simulation_api/api/api.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

// headers in STL
#include <memory>
#include <vector>
#include <string>

// headers in pugixml
#include "pugixml.hpp"

class ScenarioRunnerMoc : public rclcpp::Node
{
public:
  explicit ScenarioRunnerMoc(const rclcpp::NodeOptions & option)
  : Node("scenario_runner", option),
    api_(this, ament_index_cpp::get_package_share_directory(
        "kashiwanoha_map") + "/map/lanelet2_map.osm")
  {
    // initialize simulator
    api_.initialize(1.0, 0.02);
    pugi::xml_document catalog_xml_doc;
    // set ego vehicle parameter by XML
    catalog_xml_doc.load_string(catalog_xml.c_str());
    simulation_api::entity::VehicleParameters params(catalog_xml_doc);
    // spawn ego vehicle
    api_.spawn(true, "ego", params);
    // set ego vehicle status (initial position/velocity/acceleration)
    api_.setEntityStatus("ego", getEgoInitialStatus());
    // setup timer for updating simulation frame
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(20ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    // call update frame API
    api_.updateFrame();
    current_time_ = current_time_ + 0.02;
  }
  double current_time_;
  int port_;
  scenario_simulator::API api_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // functions to get ego initial entity status.
  openscenario_msgs::msg::EntityStatus getEgoInitialStatus()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 10.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    geometry_msgs::msg::Accel accel;
    accel.linear.x = 0.0;
    accel.linear.y = 0.0;
    accel.linear.z = 0.0;
    accel.angular.x = 0.0;
    accel.angular.y = 0.0;
    accel.angular.z = 0.0;
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = 0.0;
    rpy.z = 0.0;
    /*
    simulation_api::entity::EntityStatus ret(
      api_.getCurrentTime(), 120545, 0.0, 0.0, rpy, twist, accel);
    */
    return ret;
  }

  // parameter definition of ego vehicle.
  std::string catalog_xml =
    R"(<Vehicle name= 'vehicle.volkswagen.t2' vehicleCategory='car'>
            <ParameterDeclarations/>
            <Performance maxSpeed='69.444' maxAcceleration='200' maxDeceleration='10.0'/>
            <BoundingBox>
                <Center x='1.5' y='0.0' z='0.9'/>
                <Dimensions width='2.1' length='4.5' height='1.8'/>
            </BoundingBox>
            <Axles>
                <FrontAxle maxSteering='0.5' wheelDiameter='0.6' trackWidth='1.8' positionX='3.1' positionZ='0.3'/>
                <RearAxle maxSteering='0.0' wheelDiameter='0.6' trackWidth='1.8' positionX='0.0' positionZ='0.3'/>
            </Axles>
            <Properties>
                <Property name='type' value='ego_vehicle'/>
            </Properties>
        </Vehicle>)";
};

// entry point
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ScenarioRunnerMoc>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
```

## API Documentation

See [package details](../package/About.md).
