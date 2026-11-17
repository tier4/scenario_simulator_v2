# Cpp scenario demo

!!!tip
    This demo does not use Autoware. Ego Vehicle is interpreted as an NPC

Before you try to run this demo, you have to build and install scenario_simulator_v2 in your local computer.
If you want to know how to do that, please refer to [this documentation](../BuildInstructions.md).

## How to run simple demo

Only you have to do is execute this command in your terminal.

```bash
source ~/scenario_simulator_ws/install/local_setup.bash
ros2 launch cpp_mock_scenarios mock_test.launch.py scenario:=traffic_simulation_demo launch_rviz:=true timeout:=60.0
```

You can see ego vehicle running in kashiwanoha.

![Cpp Scenario Demo](../../image/simple_demo.png "simple demo")

<iframe src="https://www.google.com/maps/embed?pb=!1m14!1m12!1m3!1d728.9291817914587!2d139.9333589791692!3d35.903161076557446!2m3!1f0!2f0!3f0!3m2!1i1024!2i768!4f13.1!5e1!3m2!1sja!2sjp!4v1617800059334!5m2!1sja!2sjp" width="600" height="450" style="border:0;" allowfullscreen="" loading="lazy"></iframe>
