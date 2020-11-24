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
