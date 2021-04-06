# ScenarioSimulator V2

Scenario Simulator V2 is a scenario testing framework for Autoware.

![Scenario Testing Framework](image/what_is_scenario_testing_framework.png "what is scenario testing framework")

It eables us to write scenario at once and run in various kinds of simulator.  

## Purpose of this package
Currently, various kinds of simulators and scenario formats are developed all over the world.
We need an open-source framework for integrating those testing tools with Autoware easily and quickly.
So, we developed this package.
<font color="Coral">__This package is designed to easily accommodate multiple simulators and scenario description formats.__</font>
This package provides under the Apache License, Version 2.0.
See also [LICENSE](LICENSE).

## How to use

### Running example scenario
``` bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

### Running with docker image
Download docker image tar file form
[Here](https://drive.google.com/drive/folders/1Ep_CAytXa-wmIBz-_oh7hrV9UzOQTe9r?ths=true).
``` bash
# loading docker
docker load -i scenario_simulator.tar

# create sharing directory
mkdir ${HOME}/scenarios

# running docker
docker run -it -p 6080:80 -v ${HOME}/scenarios:/home/ubuntu/Desktop/scenarios --shm-size=512m scenario_simulator .
```

When you see following message in the terminal,
``` bash
* enable custom user: ubuntu
useradd: user 'ubuntu' already exists
  set default password to "ubuntu"
```

Press ctrl+c once to start VNC server.
Then, open http://localhost:6080/ in your browser.

Launch lx terminal in VNC, and run an example below.
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp'
```

## Creating & Running Your Own Simulation Scenarios

1. Create a scenario file with the scenario editor. See [Scenario Editor](user_guide/scenario_editor/ScenarioEditorUserGuide)

2. After downloading the scenario file, open it in a text editor and modify map path to the path in your local environment
```
  RoadNetwork:
    LogicFile:
      filepath: /full/path/to/your/map_file.osm
```

3. Create workflow configuration file as shown below. You can also specify multiple scenario files and add optional configurations. See [here](./user_guide/scenario_test_runner/ScenarioTestRunner) for details.
```
Scenario:
  - { path: /full/path/to/your/scenario_file.yaml }
```

4. Run the scenario:
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='path/to/your/workflow_config.yaml' log_directory:='/tmp'
```

## Detailed Documentations
### How to use scenario editor
See [Scenario Editor](user_guide/scenario_editor/ScenarioEditorUserGuide)

### How to use scenario test runner
See [Scenario Test Runner](user_guide/scenario_test_runner/ScenarioTestRunner)

### Architecture documentation
See [Architecture Documentation](./design/SystemArchitecture.md)

## Contact Information
See [Contact Information](./etc/ContactUs.md)
