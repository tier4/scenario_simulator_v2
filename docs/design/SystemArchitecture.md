# System Architecture
```plantuml source="docs/design/uml/whole_architecture.pu"
```

This tool consists of 3 components, test runner, openscenario_interpreter and simulator.

Each component has these features.

1. test_runner  
"test_runner" launch Autoware and other components in this tool.
"test_runner" communicate with "openscenario_interpreter" with ros2 lifecycle.
(https://design.ros2.org/articles/node_lifecycle.html)

1. openscenario_interpreter  
test runner communicate with openscenario_interpreter with rclcpp lifecycle.  
openscenario_interpreter is a rclcpp lifecycle component.  
![lifecycle](https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png "lifecycle")
When the openscenario_interpreter launched, the state of openscenario_interpreter is "Unconfigured".  
When the test runner launched, ths test runner configures openscenario_interpreter and the state of the openscenario_interpreter becomes "Inactive".  
After that, the test runner activates openscenario_interpreter and move the state into "Active".
When the exception was thrown in openscenario interpreter, the openscenario interpreter moves into "Inactive" state.

1. simple_sensor_simulator
simple_sensor_simulator communicates with openscenario_interpreter by using ZeroMQ.
You can use any times of simulator by adapting this API.

# Execution sequence of scenario testing.

```plantuml source="docs/design/uml/sequence.pu"
```
