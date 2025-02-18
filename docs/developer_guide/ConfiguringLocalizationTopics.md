# Configuring Localization Topics

This section describes properties for configuring localization topics that
`scenario_simulator_v2` publishes to Autoware.

## Parameter file

Localization topics are configurable from the ROS 2 parameter file given to the
launch argument `parameter_file_path` of scenario_test_runner. The default
value of `parameter_file_path` is the path to [a sample parameter
file](https://github.com/tier4/scenario_simulator_v2/blob/f15bacd819abd2044c2d6c076530a2c4070ded3d/test_runner/scenario_test_runner/config/parameters.yaml).

All parameters that can be specified and their default values are shown in the
sample parameter file. In practice, it is not necessary to specify all
parameters except for some that are mandatory. In that case, the simulator will
behave as if similar default values had been specified.

Most parameters should have their uses understood just by looking at the sample
parameter file or by reading the comments in the file. Below we discuss some
parameters that require additional detailed explanation.

<!-- cspell: ignore YYYYMMDD -->

| Name                   | Value                                   | Default              | Description                                                                                                                                                          |
|------------------------|-----------------------------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `<topic-name>.version` | An `int` type value in YYYYMMDD format  | None (**mandatory**) | Suffix of `scenario_test_runner` launch argument `architecture_type`, used to maintain backward compatibility of the simulator when changing the Autoware interface. |
