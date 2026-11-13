# Tips about writing scenarios

### Required Scenario Tags

CatalogLocations inside OpenSCENARIO is defined structure below.
CatalogLocations in OpenSCENARIO specification must exist.

```yaml
CatalogLocations:
```

or

```yaml
CatalogLocations: {}
```

### For None Value Expression

#### OK

You can write the scenario as below if ScenarioModifiers is empty.

```yaml
ScenarioModifiers:
or
ScenarioModifiers: {}
```

#### Bad

You cannot write the scenario as below, because there is no necessary key
and `ScenarioModifier: []` is invalid syntax.

```yaml
ScenarioModifiers:
  ScenarioModifier:
    - name: <String>
```

or

```yaml
ScenarioModifiers:
  ScenarioModifier: []
```
