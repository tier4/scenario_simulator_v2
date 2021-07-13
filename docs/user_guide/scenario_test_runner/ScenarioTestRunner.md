# Scenario Test Runner User Guide

Scenario Test Runner is being developed to assist in the definitive planning simulation using concept of OpenSCENARIO.
Simulations are described in a "YAML" based format called a "tier4 scenario format".
Then convert the scenario into an "XML" based format called an "OpenSCENARIO".
The format has been found at [OpenSCENARIO](http://www.openscenario.org/).

## How to use
```bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py workflow:='$(find-pkg-share scenario_test_runner)/config/workflow_example.yaml' log_directory:='/tmp'
```
The workflow file defines how to execute scenarios.
If you want to know how to write the workflow file, read [here.](HowToWriteWorkflowFile)

## Detailed Documentations

[How to write workflow file](./HowToWriteWorkflowFile.md)

[Scenario conversion](./ScenarioFormatConversion.md)

[Tips](./Tips.md)
