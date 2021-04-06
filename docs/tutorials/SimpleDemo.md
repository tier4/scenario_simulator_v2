# Simple demo

**Note! This demo does not use Autoware. Ego Vehicle is interpreted as a NPC**

Before you try to run this demo, you have to build and install scenario_simulator_v2 in you local computer.
Do you want to know how to do that, please refer [this documentation](BuildInstructions.md). 

## How to run simple demo

Only you have to do is type this command in your terminal.

```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share cenario_test_runner)/workflow_example.yaml'
```