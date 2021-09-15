# System Architecture

```plantuml source="docs/design/uml/whole_architecture.pu"

```

This tool consists of three components: test runner, openscenario_interpreter, and simulator.

Each component has the following features:

1. test_runner  
   The test_runner launches Autoware and other components in this tool.  
   The test_runner communicates with "openscenario_interpreter" with the ROS 2 lifecycle.
   (<https://design.ros2.org/articles/node_lifecycle.html>)

1. openscenario_interpreter  
   The test runner communicates with the openscenario_interpreter with the rclcpp lifecycle.  
   The openscenario_interpreter is an rclcpp lifecycle component.  
   ![lifecycle](https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png "lifecycle")
   When the openscenario_interpreter launched, the state of the openscenario_interpreter is "Unconfigured".  
   When the test runner launched, the test runner configures the openscenario_interpreter and the state of the openscenario_interpreter becomes "Inactive".  
   After that, the test runner activates the openscenario_interpreter and moves the state into "Active".
   When the exception is thrown in the openscenario interpreter, the openscenario interpreter moves into "Inactive" state.

1. simple_sensor_simulator
   The simple_sensor_simulator communicates with the openscenario_interpreter by using ZeroMQ.
   You can use any times of simulator by adapting this API.<!-- TODO: clarify the meaning -->

## Execution sequence of scenario testing

```plantuml source="docs/design/uml/sequence.pu"

```
