# How to hack scenario_simulator_v2

## Note

This document is written for those who want to participate in the development
of `scenario_simulator_v2` or make their own modifications to
`scenario_simulator_v2`. Development requires general knowledge of ROS 2
software and robotic systems and a certain level of C++ skills, as well as
reading the ASAM OpenSCENARIO standard documentation appropriately.

## Prerequisite Knowledge

The `scenario_simulator_v2` consists of the following four main components.

### `scenario_test_runner`

The `scenario_test_runner` is responsible for managing the progress of the
entire process. It takes a single scenario file or a workflow file of multiple
scenarios as arguments, and passes each scenario to `openscenario_interpreter`
to evaluate each scenario. Another task of the `scenario_test_runner` is to
preprocess each scenario file before running the scenario.

TIER IV uses scenarios in YAML format with some extensions to ASAM OpenSCENARIO
1.2. The extended syntax mainly focuses on parameterization features that were
not standardized as of OpenSCENARIO 1.0. Processing of the extended syntax by
TIER IV is handled by `scenario_test_runner` and the preprocessed scenarios are
XML files that conform to the ASAM OpenSCENARIO 1.2 schema. Note that the
scenario passed to `openscenario_interpreter` is therefore in ASAM standard
format (`.xosc`), not the extended format by TIER IV (`.yaml`).

Both scenario files written in the TIER IV extended format (`.yaml`) and
standard ASAM OpenSCENARIO 1.2 scenario files (`.xosc`) can be given to
`scenario_test_runner`.

### `openscenario_interpreter`

The `openscenario_interpreter` is an interpreter for OpenSCENARIO 1.2 scenario
definition files in `.xosc` format, corresponding to the OpenSCENARIO Director
in The ASAM OpenSCENARIO architecture. It is a naive syntax tree interpreter
that parses a scenario definition file by recursive descent parsing, constructs
an abstract syntax tree, and evaluates the syntax tree directly. The
`openscenario_interpreter` is primarily responsible for interpreting ASAM
OpenSCENARIO, since the fundamental part of the simulation, such as coordinates
and time management, is managed by the `traffic_simulator`.

### `traffic_simulator`

This is a core library for traffic flow simulation, corresponding to the
Simulator Core in The ASAM OpenSCENARIO architecture. It is responsible for
computing the coordinates and behavior of each entity that appears in the
simulation, whereas `openscenario_interpreter` is responsible for parsing
scenario files and managing the progress of the scenario. It is clearly
differentiated from `openscenario_interpreter`, but `traffic_simulator` itself
is a library and is built into `openscenario_interpreter`, so it does not
appear as independent software while running the scenario.

A lot of processing, except for entity decision making during simulation are
pluggable and designed to allow switching the level of simulation detail and
computational complexity by connecting simulators with compatible APIs.
Currently only `simple_sensor_simulator` is compatible with the
`traffic_simulator` API, but work is underway to enable the connection of
AWSIM, a high-performance simulator being developed by TIER IV.

### `simple_sensor_simulator`

Reference implementation of a simulator conforming to the `traffic_simulator`
API. It provides a very simple environmental simulation with emphasis on
lightweight execution of simulations.

Based on the true values of the entities computed by `traffic_simulator`, it
computes the recognition information in addition to the LiDAR sensor values and
sends it to Autoware.

Historically, this module was created for sensor simulation, but now a lot of
processing beyond sensor simulation have been transferred from
`traffic_simulator`.

## Other Resources

### [ASAM OpenSCENARIO: User Guide](https://www.asam.net/index.php?eID=dumpFile&t=f&f=4908&token=ae9d9b44ab9257e817072a653b5d5e98ee0babf8)

This document describes the basic concepts of ASAM OpenSCENARIO 1.2. Note that
it provides a relatively detailed description of the coordinate system, but
lacks information on the details of the language's behavior.

### [XSD description](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/)

Documentation of the structure of each syntax element of ASAM OpenSCENARIO 1.2.
Refer to it appropriately when developing `openscenario_interpreter`. However,
there are many ambiguous descriptions and many details are left to the
simulator implementors. Thus, please refer to the code comments of the
implementation as well.

## Development Guide

Please follow the steps below to set up the environment and check the sample
scenario behavior.

- [Planning Evaluation Using Scenarios](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/others/planning-evaluation-using-scenarios/)

The sample scenario
([sample.yaml](test_runner/scenario_test_runner/scenario/sample.yaml)) is a
very simple one in which an Autoware-controlled vehicle is asked to drive a
certain distance along a straight road. Although the content of the scenario is
simple, it contains the minimum necessary elements to verify that
`scenario_simulator_v2` is able to properly activate Autoware and operate under
the scenario. Therefore, if this sample scenario works without any problems,
you can be sure that there are no problems in setting up your environment.

The most efficient way to understand the `scenario_simulator_v2` implementation
is to trace the process of running the sample scenario from top to bottom. In
other words, it's a good idea to start with scenario_test_runner.launch.py and
then read the code through `scenario_test_runner`, `openscenario_interpreter`,
`traffic_simulator`, and finally `simple_sensor_simulator`. In general, you
should be able to grasp the entire process flow of `scenario_simulator_v2` and
add or modify features in a day or two.

If you are unsure about any part of the code, please refer to the detailed
documentation of the relevant section accordingly, or contact the developers
via an issue in the `scenario_simulator_v2` GitHub repository.
