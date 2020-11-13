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

1. ScenarioModifiers Block  
This block provides features to provide parameter distribution.  

1. OpenSCENARIO Block  
This block is a pure yaml conversion of OpenSCENARIO xml file.
