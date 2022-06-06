# System Architecture

```plantuml source="docs/design/uml/whole_architecture.pu"

```

The scenario_simulator_v2 consists of the three components: test runner, openscenario_interpreter, and simulator.

Each component has the following features:

1. test_runner  
   The test_runner launches Autoware and other components in this tool.  
   The test_runner communicates with "openscenario_interpreter" with the ROS 2 lifecycle.
   (<https://design.ros2.org/articles/node_lifecycle.html>)

1. openscenario_interpreter  
   The test runner communicates with the openscenario_interpreter with the rclcpp lifecycle.  
   The openscenario_interpreter is a rclcpp lifecycle component.  
   ![lifecycle](https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png "lifecycle")
   When the openscenario_interpreter launched, the state of the openscenario_interpreter is "Unconfigured".  
   When the test runner launched, the test runner configures the openscenario_interpreter and the state of the openscenario_interpreter becomes "Inactive".  
   After that, the test runner activates the openscenario_interpreter and moves the state into "Active".
   When the exception is thrown in the openscenario interpreter, the openscenario interpreter moves into "Inactive" state.

1. simulator  
   The simulator component communicates with the openscenario_interpreter by using ZeroMQ.
   You can use any simulators by adapting the ZeroMQ API.
   The [simple sensor simulator](SimpleSensorSimulator.md) is a reference implementation of the simulator component.

## Execution sequence of scenario testing

```plantuml source="docs/design/uml/sequence.pu"

```
