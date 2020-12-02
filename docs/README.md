# ScenarioSimulator

OpenSENARIO interpreter and simple simulator for Autoware

![rviz](image/rviz.png "rviz")

## Purpose of this package
Currently, various kinds of simulators and scenario formats are developed all over the world.
We need open-source framework for integrating those testing tools with Autoware easilly and quickly.
So, we developed this package.
<font color="Coral">__This package is designed to easily accommodate multiple simulators and scenario description formats.__</font>
This package provides under Apache License Version 2.0.
See also [LICENSE](LICENSE).

## How to use

### Running example scenario
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp' no_validation:=True
```

### Running with docker image
download docker image tar file form Google Drive. (https://drive.google.com/drive/folders/1Ep_CAytXa-wmIBz-_oh7hrV9UzOQTe9r?ths=true)
```bash
docker load -i scenario_simulator.tar
docker run -it -p 6080:80 --shm-size=512m scenario_simulator .
```

when you see following message in the terminal
```bash
* enable custom user: ubuntu
useradd: user 'ubuntu' already exists
  set default password to "ubuntu"
```

press ctrl+c once to start VNC server
then open http://localhost:6080/ in your browser.

launch lx terminal in VNC, and run an example by:

```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/workflow_example.yaml' log_directory:='/tmp' no_validation:=True
```

## Creating & Running Your Own Simulation Scenarios

1. Create scenario file with the scenario editor. See [Scenario Editor](doc/README.md)

2. After downloading the scenario file, open it in a text editor and modify map path to the path in your local environment
```
  RoadNetwork:
    LogicFile:
      filepath: /full/path/to/your/map_file.osm
```

3. Create workflow configuration file as shown below. You can also specify multiple scenario files and add optional configurations. See [here](docs/scenario_test_runner.md) for details.
```
Scenario:
  - { path: /full/path/to/your/scenario_file.yaml }
```

4. Run the scenario:
```
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='path/to/your/workflow_config.yaml' log_directory:='/tmp' no_validation:=True
```

## Detailed Documentations
### How to use scenario editor
See [Scenario Editor](user_guide/scenario_editor/ScenarioEditorUserGuide)

### How to use scenario test runner
See [Scenario Test Runner](user_guide/test_runner/ScenarioTestRunner)

### Architecture documentation
See [Architecture Documentation](design/SystemArchitecture)

## Contact Infomation
See [Contact Infomation](etc/ContactUs)
