# Scenario simulator v2

[![ScenarioTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml)
[![Docker](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml)
[![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml)
[![BuildTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml)
[![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)

[![dockeri.co](https://dockeri.co/image/tier4/scenario_simulator_v2)](https://hub.docker.com/r/tier4/scenario_simulator_v2)

Scenario simulator v2 is a scenario testing framework for Autoware.

![Scenario Testing Framework](image/what_is_scenario_testing_framework.png "what is scenario testing framework")

It enables Autoware developers to write a scenario at once and run in various kinds of simulator.  

## What can we do with this framework  
Currently, various kinds of simulators and scenario formats are developed all over the world.
We need an open-source framework for integrating those testing tools with Autoware easily and quickly.
So, we developed this package.
<font color="#065479E">__This package is designed to easily accommodate multiple simulators and scenario description formats.__</font>
This package provides under the Apache License, Version 2.0.
See also [LICENSE](LICENSE).

## Why this framework is v2?
This package is re-designed [scenario runner](https://github.com/tier4/scenario_runner.iv.universe) developed by [Tier IV, Inc.](https://tier4.jp/en/), so we named this framework as "scenario_simulator_v2"

