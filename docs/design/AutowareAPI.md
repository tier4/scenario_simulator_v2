# Autoware API

Autoware API provides features to control autoware easilly via ROS2 API.  
AWAPI accessor is a C++ wrapper of the Autoware API, and it enables us to integrate with Autoware and other tools very easy!  

```plantuml source="docs/design/uml/autoware_api.pu"
```

## Detailed documentation

### AWAPI Accessor
#### minimal example
minimal example of AWAPI accessor is here.

```c++
#include <awapi_accessor/accessor.hpp>
#include <cstdlib>
#include <memory>

class Example
  : public rclcpp::Node, private autoware_api::Accessor
{
  std::shared_ptr<rclcpp::TimerBase> timer;
public:
  explicit Example(const rclcpp::NodeOptions & options)
  : rclcpp::Node("awapi_accessor_example", options),
    // pass the pointer of the class which inherits rclcpp::Node, rclcpp::LifeCycleNode etc..
    autoware_api::Accessor(this),
    timer(
      create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
          std::cout << ">>> TIMER CALLBACK!" << std::endl;
          {
            std_msgs::msg::Bool message {};
            message.data = true;
            setAutowareEngage(message);
          }
          {
            autoware_planning_msgs::msg::Route message {};
            setAutowareRoute(message);
          }
          {
            std_msgs::msg::Bool message {};
            setLaneChangeApproval(message);
          }
          {
            std_msgs::msg::Bool message {};
            setLaneChangeForce(message);
          }
          {
            autoware_perception_msgs::msg::TrafficLightStateArray message {};
            setTrafficLightStateArray(message);
          }
          {
            std_msgs::msg::Float32 message {};
            setVehicleVelocity(message);
          }
          {
            autoware_api_msgs::msg::AwapiAutowareStatus current {
              getAutowareStatus()
            };
          }
          {
            autoware_perception_msgs::msg::TrafficLightStateArray current {
              getTrafficLightStatus()
            };
          }
          {
            autoware_api_msgs::msg::AwapiVehicleStatus current {
              getVehicleStatus()
            };
          }
          {
            static auto value = 0;
            std_msgs::msg::String message {};
            message.data = "loop " + std::to_string(++value);
            setDebugString(message);
          }
          {
            std_msgs::msg::String current {
              getDebugString()
            };
            std::cout << current.data << std::endl;
          }
          std::cout << "<<< TIMER CALLBACK!" << std::endl;
        }))
  {}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor {};
  rclcpp::NodeOptions options {};
  auto example {
    std::make_shared<Example>(options)
  };
  executor.add_node(example);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

```

AWAPI Accessor has APIS below.  

```c++
setAutowareEngage(std_msgs::msg::Bool message);
```
engage autoware vehicle when the data of the message is true.


```c++
setAutowareRoute(autoware_planning_msgs::msg::Route message);
```
pass the route to the autoware.  

```c++
setLanChangeApproval(std_msgs::msg::Bool message);
```
approve lane change when the data of the message is true.  

```c++
setTrafficLightStateArray(autoware_perception_msgs::msg::TrafficLifhtStateArray message);
```
send traffic light state to the autoware.  

```c++
setVehicleVelocity(std_msgs::msg::Float32 message);
```
send vehicle velocity limit to the autoware.  

```c++
autoware_api_msgs::msg::AwapiAutowareStatus getAutowareStatus();
```
return autoware status.  

```c++
autoware_perception_msgs::msg::TrafficLifhtStateArray getTrafficLightStatus();
```
return current traffic right status.  

```c++
autoware_api_msgs::msg::AwapiVehicleStatus getVehicleStatus();
```
return current vehicle status.  

### AWAPI Adaptor

AWAPI adaptor collect datas from Autoware and send it to the AWAPI accessor.  
The interface of AWAPI adaptor is below.  
<font color="Red">Note! These APIs are not implemented yet for Autoware.Auto.  
They will be implemented after the integration branch will be merged.</font>  

#### /awapi/autoware/get/status

Current Autoware status.

Type: [autoware_api_msgs::msg::AwapiAutowareStatus](https://github.com/tier4/Pilot.Auto/blob/ros2/awapi/autoware_api_msgs/msg/AwapiAutowareStatus.msg)

Required for fault-injection scenarios (ODD2, See below) and failure cause analysis.
- UC-001-0028
- UC-001-0030 to UC-001-0036
- UC-001-0039
- UC-001-0044
- UC-016-0017

#### /awapi/autoware/put/engage

Send engage signal.

Type: std_msgs::msg::Bool

Required to put Autoware on standby until the simulation is ready to run.

#### /awapi/autoware/put/route

Send route to Autoware.

Type: [autoware_planning_msgs::msg::Route](https://github.com/tier4/Pilot.Auto/blob/ros2/common/msgs/autoware_planning_msgs/msg/Route.msg)

Required for OpenSCENARIO 1.0 `RoutingAction`.

#### /awapi/autoware/put/goal_pose

Send goal pose to Autoware.

Type: gometry_msgs::msg::PoseStamped

Required for OpenSCENARIO 1.0 `AcqirePositionAction`.

#### /awapi/autoware/put/initial_pose

Set initial pose to Autoware.

Type: geometry_msgs::msg::PoseStamped

Required for OpenSCENARIO 1.0 `TeleportAction`.
Although not mentioned in OpenSCENARIO, applying TeleportAction to ego cars outside the Init section should be an error.
Therefore, the intiial_pose should be sent only once at the start of the simulation.

#### /awapi/vehicle/get/status	

Current vehicle status (position, velocity, acceleration, steer angle) .

Type: [autoware_api_msgs::msg::AwapiVehicleStatus](https://github.com/tier4/Pilot.Auto/blob/ros2/awapi/autoware_api_msgs/msg/AwapiVehicleStatus.msg) 

Required for many OpenSCENARIO 1.0 EntityConditions.

#### /awapi/map/get/lanelet2_map

Get raw Lanelet2 map data.

Type: TBD

Required for NPC vehicle routing.

#### /awapi/map/put/lanelet2_map

Send path of lanelet2_map.osm to Lanelet2 Map Loader.

Type: std_msgs::msg::String

Required for OpenSCEANRIO 1.0 `LoadNetwork.LogicFile`.

#### /awapi/perception/put/detected_objects

Send NPC position and pose to Autoware directly as dummy object detection result.

Type: [autoware_perception_msgs::msg::DynamicObjectArray](https://github.com/tier4/Pilot.Auto/blob/ros2/common/msgs/autoware_perception_msgs/msg/object_recognition/DynamicObjectArray.msg)

Required for NPC vehicle simulation.

#### /awapi/perception/put/traffic_light

Directly change the signal recognition status of Autoware.

Type: [autoware_perception_msgs::msg::TrafficLightStateArray](https://github.com/tier4/Pilot.Auto/blob/ros2/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/TrafficLightStateArray.msg)

Required for OpenSCENARIO 1.0 `TrafficSignalAction`.

#### /awapi/vehicle/put/upper_bound_velocity

Set upper-bound velocity of Autoware directly.

Type: std_msgs::msg::Float32

Required for scenarios where Autoware runs at speeds below the legal speed embedded in the map.

#### /awapi/autoware/put/emergency

Forces Autoware into a state of emergency.

Type: TBD

Required for fault-injection scenarios (ODD2, See below).
- UC-001-0028
- UC-001-0030 to UC-001-0036
- UC-001-0039
- UC-001-0044
- UC-016-0017

#### /awapi/map/put/pointcloud_map

Send path of pointcloud_map.osm to Point Cloud Map Loader.

Type: std_msgs::msg::String

Required for OpenSCEANRIO 1.0 `LoadNetwork.SceneGraphFile`.

#### /awapi/map/get/pointcloud_map

Get raw Point Clound map data.

Type: TBD

Required for collision detection with obstacles (buildings, etc.) other than NPCs.

#### /awapi/perception/get/traffic_light

Curret Autoware's recognition result of traffic light.

Type: [autoware_perception_msgs::msg::TrafficLightStateArray](https://github.com/tier4/Pilot.Auto/blob/ros2/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/TrafficLightStateArray.msg)

Required for failure cause analysis.
In the case of Planning SImulation, it is exactly the same as the dummy information sent by put.

#### /awapi/perception/get/detected_objects

Current Autoware's detection result of obstacles.

Type: [autoware_perception_msgs::msg::DynamicObjectArray](https://github.com/tier4/Pilot.Auto/blob/ros2/common/msgs/autoware_perception_msgs/msg/object_recognition/DynamicObjectArray.msg)

Required for failure cause analysis.
In the case of Planning SImulation, it is exactly the same as the dummy information sent by put.

#### /awapi/sensing/put/no_ground_pointcloud

#### /awapi/sensing/get/no_ground_pointcloud

Type: TBD

Required for collision detection with obstacles (buildings, etc.) other than NPCs.
In the case of Planning SImulation, it is exactly the same as the dummy information sent by put.
