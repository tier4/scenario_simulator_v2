# TierIV Scenario Format Ver 2.0

### What is TierIV Scenario Format ver 2.0
What is TierIV Scenario Format ver 2.0 is a yaml-based scenario description format.  
TierIV Scenario format are consistes of two blocks below.  

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
name express a variable It is not case sensitive, but attributes must be a lower snake case and it is converted to a variable in it's list during parameter distribution.
See more details in test folder in scenario_test_runner package.
start,step stop express it's variable range.
initial parameter distribution is from start to end while increasing a value.

We can define parameter destributions like below
```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: "destribution"
      start: 10
      stop: 20
      step: 3

```
if step value is 1 then distributed parameter is [10]  
if step is value 2 distributed parameter is [10, 20]  
if step is value 3 distributed parameter is [10, 15, 20]  

attension or int
- if step is one, only parameter of start is used
- if step is zero, it returns error
- the number of simulation is factorial to number of  steps

#### OpenSCENARIO Block  
This block is a pure yaml conversion of OpenSCENARIO xml file.
OpenSCENARIO user guide is [here.](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword)

