# Random test runner

Random test runner allows running randomly generated scenarios to test Autoware autonomy implementation. For more information regarding Random test runner features and limitations please see [Usage](Usage.md#features).

## How to build

1. Clone the Autoware Core/Universe repository:
   ```bash
   git clone git@github.com:autowarefoundation/autoware.git
   ```
2. Navigate to the source directory:
   ```bash
   cd autoware 
   mkdir src 
   ```
3. Import Autoware and Simulator dependencies:
   ```bash
   vcs import src < autoware.repos  
   vcs import src < simulator.repos
   ```
4. Install dependencies for Autoware Core/Universe
   ```bash
   ./setup-dev-env.sh
   ``` 

5. Install dependent ROS packages.
   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install -iry --from-paths src --rosdistro $ROS_DISTRO
   ```
6. Build the workspace.
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## How to run

First complete build described in [How to build](#how-to-build) section.


Being in the main project directory run:

```shell
source install/setup.bash
ros2 launch random_test_runner random_test.launch.py
```

Which will run random tests with default parameters. Similarly to the image below you should see several npcs spawned in random locations around the ego vehicle, which will move on random path following the goal.

![Random test runner launched](img/random-test-runner-launched.png)

Detailed description of the possible parameters can be found under [Parameters](Usage.md#launch-arguments).

## 

After test is completed see `/tmp` directory. Among others, there will be two files:
1. `result.junit.xml` - test result file with information about encountered errors.
2. `result.yaml` - yaml file that can be used to replay tests.

For the more specified information about output files please see [Results](Usage.md#results).

##

It might happen that the random test runner will behave unexpectedly and the test will not launch correctly. For further details regarding known issues please see [Troubleshooting](Usage.md#troubleshooting).

## How to replay

Prerequisites:
1. Build as instructed in [How to build](#how-to-build)
2. Acquire `result.yaml` file:
   1. Either by running test as stated in [How to run](#how-to-run) part of instruction.
   2. Receiving it from someone who already ran it.
3. Place `result.yaml` in `<some_directory>` IMPORTANT: Do not change filename.
4. Execute:
 
```shell
ros2 launch random_test_runner random_test.launch.py input_dir:=<some_directory>
```

Random test runner will load `result.yaml` file and rerun test.

## Running with unity

TBD

[//]: # (Instruction is based on `kashiwanoha_map` Unity project but can be applied to any other projects supporting [`ZeroMQ` interface]&#40;https://tier4.github.io/scenario_simulator_v2-docs/design/ZeroMQ/&#41;. )

[//]: # ()
[//]: # (To run `random_test_runner` with Unity Kashiwanoha project: )

[//]: # (1. Clone and run [Kashiwanoha project]&#40;https://gitlab.com/robotec.ai/tieriv/kashiwanoha&#41;.)

[//]: # (2. Make sure that package name in `map_name` parameter is `kashiwanoha_map`. For projects other than Kashiwanoha, make sure to change it to correct package name.  )

[//]: # (3. Execute `random_test_runner` launch with `simulator_type` parameter:)

[//]: # (```shell)

[//]: # (ros2 launch random_test_runner random_test.launch.py simulator_type:="unity")

[//]: # (```)

[//]: # (|  NOTE: Since currently unity integration does not support ego vehicle, `random_test_runner` does not spawn it. |)

[//]: # (|----------------------------------------------------------------------------------------------------------------|)

[//]: # ()
[//]: # (|  NOTE: Kashiwanoha project is only supported on ROS 2 `galactic` but simulation interfaces are distribution-independent and random tests can be executed safely on `foxy` |)

[//]: # (|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|)

