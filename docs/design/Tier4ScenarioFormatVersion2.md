# TierIV Scenario Format Version 2.0

### What is TierIV Scenario Format version 2.0
The "TierIV Scenario format version 2.0" is a yaml-based scenario description format.

"TierIV Scenario format version 2.0" consisting of two blocks below.  

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

#### ScenarioModifiers Block  
This block provides features to provide parameter distribution.

The parameter, "name" express a variable It is not case sensitive, but attributes must be a lower snake case and it converts to a variable in its list during parameter distribution.
See more details in test folder in scenario_test_runner package.
Three parameters, "start", "stop" and "step" express its varied range.
Initial parameter distribution is from start to finish while increasing a value.

We can define parameter distributions like below
```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: "distributions"
      start: 10
      stop: 20
      step: 3

```
if the step value is 1 then distributed parameter is [10]  

if the step is value 2 distributed parameters are [10, 20]

if the step is value 3 distributed parameters are [10, 15, 20]  

attention
- If the step is one, the only parameter of start is used
- If the step is zero, it returns error
- The number of simulations is factorial to number of steps.

#### OpenSCENARIO Block  
This block is a pure yaml conversion of the OpenSCENARIO xml file.

The OpenSCENARIO user guide is [here.](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword)

