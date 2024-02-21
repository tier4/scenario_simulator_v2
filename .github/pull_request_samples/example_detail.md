# Description

## Abstract

Fixed bags below.

- Removed equal operators for geometry_msgs::msg::Point and geometry_msgs::msg::Vector3, because they were ambiguous.
- Fixed the bug which caused the intersection functions using vector to look past the last element of the vector and return wrong results.
- Fixed a bug where the LineSegment class could be constructed with a geometry_msgs::msg::Vector3 of size = 0. This lead to initialization of end_point with nan values.
- Fixed the getMinValue and getMaxValue bug where when an empty vector was passed the function tried to de-reference vector.end() and the whole program crashed.
- Fixed a getPolygon bug which caused the function to generate incorrect number of points.
- Added support for negative curvature values which were already supported by HermiteCurve. This incompatibility lead to errors.
- Fixed spline collision implementation error which caused spline to add normalized length to absolute lengths which is incorrect.
- Fixed CatmullRomSubspline collision detection by enabling the HermiteCurve and CatmullRomSpline to have multiple collisions detected and then choosing in the subspline the collisions that occur inside the subspline.

## Background

Fixed several residual problems in the geometric calculation library that were causing incorrect scenario execution results.

## Details

This PR fixes how the length of the curve is computed

| Before fix                                                                                                                                                                                                                                                                                                            | After fix                                                                                                                                                                                                                                                                                                                                                                                                     |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Lengths of the first two Curves (A and B) are calculated correctly (because these are full Curve lengths), but the distance in the third Curve (C) is calculated as half of the normalized length of the Curve (0.5). This value is added and the result and distance to the collision in the spline is equal to 20.5 | Lengths of the first two Curves (A and B) are calculated correctly (because these are full Curve lengths) and the distance in the third Curve (C) is also calculated correctly because the collision distance is being denormalized (multiplied by the full length of the Curve) which is equal to 5 (0.5 * 10).This value is added and the result and distance to the collision in the spline is equal to 25 |
| ![RJD-753_base](https://github.com/tier4/scenario_simulator_v2/assets/87643052/18b0f1a5-5370-4cf3-a60a-c1af05448d50)                                                                                                                                                                                                  | ![RJD-753_eval](https://github.com/tier4/scenario_simulator_v2/assets/87643052/87570089-bf77-4be8-b950-e3f1fb8499a9)                                                                                                                                                                                                                                                                                          |

## References

See also [this document.](https://tier4.github.io/scenario_simulator_v2-docs/)  
This link is an example and is not directly related to this sample.

# Destructive Changes

| traffic_simulator/OpenSCENARIO features | Severity | Fix | 
|----------------------------------------------------------|-------------|--------------|
| FollowFrontEntityAction | low - distance kept to the front entity might be a bit larger which might influence scenarios which are relying on specific behavior in this action but it does not seem to be a lot of scenarios | Conditions which check this distance should be increased accordingly - trial and error seem to be the best way to find how much change the condition without changing the essence of the scenario|
|  Computing the distance to stop_line or other "obstacles" | low - it will increase this distance making it a bit safer but if the scenario conditions rely on this value it might cause issues. No scenario, however, was noticed to be influenced by that to the extent that caused them to fail | Conditions that check this distance should be increased accordingly - trial and error seem to be the best way to find how much change the condition without changing the essence of the scenario |
| Distance-based lane change | Medium - regression is known which was caused by the fix. Cut-in scenarios which are written in a rather strict way might be influenced by this change | Decreasing the distance on which the lane change should occur or change the speed of the vehicles taking part in the scenario - example in the section below | 

This scenario usually passes without the fix but always fails after the fix is added. The scenario is based on [shinjuku_map](https://github.com/tier4/AWSIM/releases/download/v1.2.0/shinjuku_map.zip).

[scenario.yml.txt](https://github.com/tier4/scenario_simulator_v2/files/13707779/scenario.yml.txt)

An issue in this scenario is lane change action:

```
LaneChangeAction:
  LaneChangeActionDynamics:
    dynamicsDimension: distance
    value: 10 
    dynamicsShape: cubic
  LaneChangeTarget:
    AbsoluteTargetLane:
      value: '37'
```

To fix it:
- The speed of Ego vehicle can be decreased slightly
- The speed of NPC motorcycle can be increased slightly
- The lane change distance can be decreased slightly

The exact amount of the change of the values above is hard to estimate because it is very dependent on the specific scenario - like the vehicle position within the lanelet might influence how much the normalized lanelet length was different than not normalized length.

Here is the scenario that uses the third possible fix - decreasing the distance on which the lane change action takes place. One meter decrease makes the scenario pass again.

```
LaneChangeAction:
  LaneChangeActionDynamics:
    dynamicsDimension: distance
    value: 9 
    dynamicsShape: cubic
  LaneChangeTarget:
    AbsoluteTargetLane:
      value: '37'
```

Full fixed scenario
[scenario.yml.txt](https://github.com/tier4/scenario_simulator_v2/files/13707839/scenario.yml.txt)

# Known Limitations

- If the curvature is very large, the calculation may fail.
  - This link is an example and is not directly related to this sample.
