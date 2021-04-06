# Scenario simulator v2

Scenario simulator v2 is a scenario testing framework for Autoware.

![Scenario Testing Framework](image/what_is_scenario_testing_framework.png "what is scenario testing framework")

It enables Autoware developers to write a scenario at once and run in various kinds of simulator.  

## What can we do with this framework  
Currently, various kinds of simulators and scenario formats are developed all over the world.
We need an open-source framework for integrating those testing tools with Autoware easily and quickly.
So, we developed this package.
<font color="Coral">__This package is designed to easily accommodate multiple simulators and scenario description formats.__</font>
This package provides under the Apache License, Version 2.0.
See also [LICENSE](LICENSE).

## Why this framework is v2?
This package is re-designed [scenario runner](https://github.com/tier4/scenario_runner.iv.universe) developed by [Tier IV, Inc.](https://tier4.jp/en/), so we named this framework as "scenario_simulator_v2"

## Documentation Guide
### How to build scenario simulator
See [Build Instruction](tutorials/BuildInstructions.md)

### Run Simple Demo
See [Simple Demo](tutorials/SimpleDemo.md)

### How to use scenario editor
See [Scenario Editor](user_guide/scenario_editor/ScenarioEditorUserGuide.md)

### How to use scenario test runner
See [Scenario Test Runner](user_guide/scenario_test_runner/ScenarioTestRunner.md)

### Architecture documentation
See [Architecture Documentation](./design/SystemArchitecture.md)

## Contact Information
See [Contact Information](./etc/ContactUs.md)

