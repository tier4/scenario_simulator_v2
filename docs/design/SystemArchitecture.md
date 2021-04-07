# System Architecture
```plantuml source="docs/design/uml/whole_architecture.pu"
```

This tool consists of 3 components, test runner, oepnscenario_interpretor and simulator.

Each component has these features.

1. test_runner  
"test_runner" launch Autoware and other components in this tool.
"test_runner" communicate with "openscenario_interpretor" with ros2 lifecycle.
(https://design.ros2.org/articles/node_lifecycle.html)

1. openscenario_interpretor  
test runner communicate with openscenario_interpretor with rclcpp lifecycle.  
openscenario_interpretor is a rclcpp lifecycle component.  
![lifecycle](https://design.ros2.org/img/node_lifecycle/life_cycle_sm.png "lifecycle")
When the openscenario_interpretor launched, the state of openscenario_interpretor is "Unconfigured".  
When the test runner launched, ths test runner configures openscenario_interpretor and the state of the openscenario_interpretor becomes "Inactive".  
After the, the test runner activates openscenario_interpretor and move the state into "Active".
When the exception was thrown in openscenario interpretor, the openscenario interpretor moves into "Inactive" state.

1. simple_sensor_simulator
simple_sensor_simulator communicates with openscenario_interpretor by using XMLRPC API.
You can use any times of simulator by adapting this API.

# Execution sequence of scenario testing.

```plantuml source="docs/design/uml/sequence.pu"
```
