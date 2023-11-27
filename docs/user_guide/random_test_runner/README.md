# Random test runner

Random test runner allows running randomly generated scenarios to test autoware autonomy implementation.

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

Which will run random tests with default parameters. You should see several npcs spawned in random locations around the ego vehicle, which will move on random path.
Detailed description of the possible parameters can be found under [Parameters](#launch-arguments).

## 

After test is completed see `/tmp` directory. Among others, there will be two files:
1. `result.junit.xml` - test result file with information about encountered errors.
2. `result.yaml` - yaml file that can be used to replay tests.

For the more specified information about output files please see [Results](#results).

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

## Launch arguments

This section describes arguments of the random test runner. All of them can be specified via command line, otherwise default value specified in the launch file is used. 

Parameters listed in [Node parameters](#node-parameters) section can be also specified in a yaml file which should be located in `<random_test_runner_directory>/param/`.

Parameters have the following source precedence priorities:
1. `*.yaml` file (applicable only for [Node parameters](#node-parameters))
2. Command line
3. Default values

### General arguments

| Parameter name               | Default value                 | Description                                                                                                                                                                                                                 |
|------------------------------|-------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `test_parameters_filename`   |  `""`                         | Yaml filename within `random_test_runner/param` directory containing test parameters. Only [Node parameters](#node-parameters) can be specified in this file.                                                               |
| `simulator_type`             |  `"simple_sensor_simulator"`  | Backend simulator. Currently supported value is `simple_sensor_simulator`. It's also accepted by the node but should be supplied as direct launch argument                                                              |
| `simulator_host`              | `"localhost"`                 | Simulation host. It can be either IP address or the host name that is resolvable in the environment (you can add a host by appending `"<SIMULATOR_IP> <SIMULATOR_NAME>"` line to the `/etc/hosts` file)                     |

### Autoware related arguments

Launch also accepts autoware parameters that control autoware related behavior. It can set which autoware architecture is in use, which vehicle
and sensor model is used in the simulation

| Parameter name      | Default value                 | Description                                                               |
|---------------------|-------------------------------|---------------------------------------------------------------------------|
| `architecture_type` | `"awf/universe"`              | Autoware architecture type. Supported values: `awf/universe` |
| `sensor_model`      | `"sample_sensor_kit"`                   | Ego sensor model                                                          |
| `vehicle_model`     | `"sample_vehicle"`                     | Ego vehicle model                                                         |


### Node parameters

Random testing supports several parameters to control test execution. They can be supplied either directly from command 
line or via `*.yaml` file inside `<random_test_runner_directory>/param/`.

#### Parameters reference

Random test runner parameters are split into three categories:

1. [Test control parameters](#test-control-parameters)
2. [Test suite parameters](#test-suite-parameters)
3. [Test case parameters](#test-case-parameters)

#### Test control parameters

High level parameters not directly related to the test itself

| Parameter name    | Default value                 | Description                                                                                                               |
|-------------------|-------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| `input_dir`       |  `""`                         |  Directory containing the result.yaml file to be replayed. If not empty, tests will be replayed from result.yaml          |
| `output_dir`      |  `"/tmp"`                     |  Directory to which result.yaml and result.junit.xml files will be placed                                                 |
| `test_count`      |  `5`                          |  Number of test cases to be performed in the test suite                                                                   |
| `simulator_type`  |  `"simple_sensor_simulator"`  |  Backend simulator. Currently supported value is `simple_sensor_simulator`. It should be set only via launch argument |
| `initialize_duration`         | `25`                          | How long test runner will wait for Autoware to initialize                                                                                                                                                                   |

#### Test suite parameters

Core test parameters. It sets map name, ego goal information and npc spawning parameters.

| Parameter name                            | Default value       | Description                                                                                                                                                               |
|-------------------------------------------|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `test_name`                               | `"random_test"`     | Test name. Used for descriptive purposes only                                                                                                                             |
| `map_name`                                | `"kashiwanoha_map"` | Package name containing map information (lanelet, point cloud, etc)                                                                                                       |
| `ego_goal_lanelet_id`                     | `-1`                | Goal lanelet's id. If `-1`, goal will be chosen randomly                                                                                                                  |
| `ego_goal_s`                              | `0.0`               | Goal lanelet's s (translation along the lanelet in meters). If `ego_goal_lanelet_id` equals `-1`, s will be chosen randomly                                               |
| `ego_goal_partial_randomization`          | `False`             | If `true`, goal will be randomized within distance set in `ego_goal_partial_randomization_distance` value. If `ego_goal_lanelet_id` is set to `-1`, this value is ignored |
| `ego_goal_partial_randomization_distance` | `30.0`              | Distance in meters from goal set by `ego_goal_lanelet_id` and `ego_goal_s`, within which goal pose will be randomized if `ego_goal_partial_randomization` is set to true  |
| `npc_count`                               | `10`                | Generated npc count                                                                                                                                                       |
| `npc_min_speed`                           | `0.5`               | Minimum speed of generated npcs                                                                                                                                           |
| `npc_max_speed`                           | `3.0`               | Maximum speed of generated npcs                                                                                                                                           |
| `npc_min_spawn_distance_from_ego`         | `10.0`              | Minimum distance of generated npcs from ego                                                                                                                               |
| `npc_max_spawn_distance_from_ego`         | `100.0`             | Maximum distance of generated npcs from ego                                                                                                                               |

#### Test case parameters

Test case parameters. Currently, only randomization seed.

| Parameter name  | Default value | Description                                                       |
|-----------------|---------------|-------------------------------------------------------------------|
| `seed`          |   `-1`        | Randomization seed. If `-1`, seed will be generated for each test |

## Results

After the testu suite execution two files can be found in the specified output folder.

### Result yaml file

Stores parameters used to generate the test suite. This file might be used to rerun the tests as described in [Replay](#how-to-replay).

#### Example `result.yaml`:
```yaml
random_test:
  name: video_test
  map_name: kashiwanoha_map
  ego_goal_s: 5.000000000000000000
  ego_goal_lanelet_id: -1
  ego_goal_partial_randomization: false
  ego_goal_partial_randomization_distance: 30
  npc_count: 10
  npc_min_speed: 0.500000000000000000
  npc_max_speed: 3.000000000000000000
  npc_min_spawn_distance_from_ego: 10.000000000000000000
  npc_max_spawn_distance_from_ego: 50.000000000000000000
  test_cases:
    - seed: 1281242544
    - seed: 3644198185
    - seed: 2673087374
    - seed: 1312244741
    - seed: 2518911638
```

### Result JUnit file

The file contains information about errors which occured during test cases which were executed by the random test runner.

If the execution had finished with an error the stored information contains type of the error and the message describing the error.

There are 3 types of error related directly to the simulation reported which includes:
1. `Stand still error` - reported when ego is found stuck in one place for too long.
2. `Timeout error` - reported when ego fails to reach the goal in a specified time.
3. `Collision error` - reported when collision between ego and npc appears.

Moreover if any `AutowareError`, `scenario_simulator_exception` or `std::runtime_error` occurs during the execution it will also be stored inside this file with a specific description.

If any other error occurs during the random test runner execution it will be stored with along with the information that the unknown error has occured.

#### Example `result.junit.xml`:
```xml
<?xml version="1.0"?>
<testsuites failures="0" errors="6" tests="6">
  <testsuite name="random_test" failures="0" errors="6" tests="6">
    <testcase name="4">
      <error type="stand still" message="Ego seems to be stuck" />
    </testcase>
    <testcase name="2">
      <error type="collision" message="npc2 and ego collided at 6.249999999999986s" />
      <error type="stand still" message="Ego seems to be stuck" />
    </testcase>
    <testcase name="3">
      <error type="collision" message="npc1 and ego collided at 17.25000000000011s" />
      <error type="timeout" message="Ego failed to reach goal within timeout" />
    </testcase>
    <testcase name="1">
      <error type="stand still" message="Ego seems to be stuck" />
    </testcase>
    <testcase name="0" />
  </testsuite>
</testsuites>
```