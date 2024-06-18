# Traffic simulator

![Simple Demo](../image/simple_demo.png "traffic simulator")

The traffic simulator simulates a traffic flow in an urban area.
Each NPC has a behavior tree and follows the user's commands.

## C++ API

The traffic simulator provides C++ APIs to control the NPC behavior in simulation.
You can also see the detailed documentation of the API classes [here](https://tier4.github.io/scenario_simulator_v2-api-docs/classtraffic__simulator_1_1API.html).


<iframe 
  class="hatenablogcard" 
  style="width:100%;height:155px;max-width:450px;" 
  title="embree" 
  src="https://hatenablog-parts.com/embed?url=https://tier4.github.io/scenario_simulator_v2-api-docs/classtraffic__simulator_1_1API.html" 
  width="300" height="150" frameborder="0" scrolling="no">
</iframe>


### Minimal example

```c++
#include <traffic_simulator/api/api.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
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
    api_.setVerbose(true);
    api_.initialize(1.0, 0.02);
    pugi::xml_document catalog_xml_doc;
    catalog_xml_doc.load_string(catalog_xml.c_str());
    traffic_simulator::entity::VehicleParameters params(catalog_xml_doc);
    api_.spawn(false, "ego", params);
    api_.setEntityStatus("ego",
      traffic_simulator::helper::constructLaneletPose(120545, 0),
      traffic_simulator::helper::constructActionStatus(10));
    api_.setTargetSpeed("ego", 15, true);
    pugi::xml_document pedestrian_xml_doc;
    pedestrian_xml_doc.load_string(pedestrian_xml.c_str());
    traffic_simulator::entity::PedestrianParameters pedestrian_params(pedestrian_xml_doc);
    api_.spawn(false, "tom", pedestrian_params);
    api_.setEntityStatus("tom", "ego",
      traffic_simulator::helper::constructPose(10, 3, 0, 0, 0, 1.57),
      traffic_simulator::helper::constructActionStatus());
    api_.spawn(false, "bob", pedestrian_params,
      traffic_simulator::helper::constructLaneletPose(34378, 0.0),
      traffic_simulator::helper::constructActionStatus(1));
    api_.setTargetSpeed("bob", 1, true);
    api_.spawn(false, "npc1", params,
      traffic_simulator::helper::constructLaneletPose(34579, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc1", 5, true);
    lanechange_executed_ = false;
    api_.spawn(false, "npc2", params,
      traffic_simulator::helper::constructLaneletPose(34606, 20.0),
      traffic_simulator::helper::constructActionStatus(5));
    api_.setTargetSpeed("npc2", 0, true);
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(20ms, std::bind(&ScenarioRunnerMoc::update, this));
  }

private:
  void update()
  {
    if (api_.reachPosition("ego",
      traffic_simulator::helper::constructLaneletPose(34615, 10.0), 5))
    {
      api_.requestAcquirePosition("ego",
        traffic_simulator::helper::constructLaneletPose(35026, 0.0) );
      api_.setTargetSpeed("npc2", 13, true);
    }
    if (api_.reachPosition("ego",
      traffic_simulator::helper::constructLaneletPose(34579, 0.0), 5))
    {
      api_.setTargetSpeed("npc2", 3, true);
    }
    if (api_.checkCollision("ego", "npc1")) {
      std::cout << "npc1 collision!" << std::endl;
    }
    if (api_.checkCollision("ego", "npc2")) {
      std::cout << "npc2 collision!" << std::endl;
    }
    api_.updateFrame();
    current_time_ = current_time_ + 0.02;
  }
  bool lanechange_executed_;
  bool target_speed_set_;
  bool bob_spawned_;
  double current_time_;
  int port_;
  traffic_simulator::API api_;
  rclcpp::TimerBase::SharedPtr update_timer_;

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

  std::string pedestrian_xml =
    R"(
    <Pedestrian model='bob' mass='0.0' name='Bob' pedestrianCategory='pedestrian'>
            <BoundingBox>
                <Center x='0.0' y='0.0' z='0.5'/>
                <Dimensions width='1.0' length='1.0' height='2.0'/>
            </BoundingBox>
            <Properties/>
        </Pedestrian>)";
};

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
