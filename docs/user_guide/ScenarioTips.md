# Scenario Example

Sample pieces of scenario

## Lane Change

with this piece of scenario, lane change will be executed as below.

- the shape of trajectory is smooth, and it is based on a cubic spline curve
- it takes 5 seconds to finish the lane change
-  the target lane is left (left is positive)

```yaml
Action:
  - name: lane_change
    PrivateAction:
      LateralAction:
        LaneChangeAction:
          LaneChangeActionDynamics:
            dynamicsDimension: time
            dynamicsShape: cubic
            value: 5
          LaneChangeTarget:
            RelativeTargetLane:
              entityRef: ego
              value: 1
```

![](../image/lane_change.gif)

