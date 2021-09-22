# TierIV Scenario Format version 2.0

## What is TierIV Scenario Format version 2.0

The "TierIV Scenario format version 2.0" is a yaml-based scenario description format.

"TierIV Scenario format version 2.0" consists of two blocks below:

```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: <string>
      list: [<any>, <any> ..., <any>]
    - name: <string>
      start: <float or int>
      step: <float or int>
      stop: <float or int>
OpenSCENARIO:
  FileHeader: <string>
    revMajor: <string or int>
    revMinor: <string or int>
    date: <string or int>
    description: <string>
    author: <string>
  ParameterDeclarations:
  .
  .
```

### ScenarioModifiers Block

This block provides features to provide parameter distribution.

The parameter "name" expresses a variable. It is not case-sensitive, but its attributes must be the lower snake case and it is converted to a variable in its list during parameter distribution.
You can find more details in the test folder of the scenario_test_runner package.
Three parameters, "start", "stop" and "step" express theirs varied ranges.
Initial parameter distribution is from start to finish while increasing a value.

We can define parameter distributions like below:

```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: "distributions"
      start: 10
      stop: 20
      step: 3
```

If the step value is 1 then distributed parameter is [10]

If the step is value 2 distributed parameters are [10, 20]

If the step is value 3 distributed parameters are [10, 15, 20]

Attention:

- If the step is one, the only parameter of start is used
- If the step is zero, it returns error
- The number of simulations is factorial to number of steps

#### OpenSCENARIO Block

This block is a pure yaml conversion of the OpenSCENARIO xml file.

The OpenSCENARIO user guide is [here](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword).
