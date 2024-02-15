# Lane pose caluculation

The calculation method of the lane coordinate system differs from Entity to Entity, and the process of determining which lane to match is complex, including fallback.  
In this document, we will show how the posture calculation of the lane coordinate system is performed for each Entity.

There are two possible times when the lane coordinate system may be computed for all Entities.  
These are the timing immediately after the spawn of the Entity and the timing of the frame update.

## Lane pose caluculation when spawning

If you spawn by specifying the lane coordinate system, lane pose calucluation is skipped.  
Internally, spawn by specifying the lane coordinate system means specifying LaneltPose as the argument pose of the [API::spawn function](https://tier4.github.io/scenario_simulator_v2-api-docs/classtraffic__simulator_1_1API.html)  
In the OpenSCENARIO , it is to execute a TeleportAction by specifying the LanePosition. ([e.g](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/test_runner/scenario_test_runner/))

If you spawn by specifying the map coordinate system, [lane pose calucluation](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp#L521-L529) is executed.

There are several internal implementations of pose calculation in the lane coordinate system implemented in the [HDmapUtils class](https://tier4.github.io/scenario_simulator_v2-api-docs/classhdmap__utils_1_1HdMapUtils.html),
but the one used in this case is [here](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L602-L630).

In this function, the following procedure is used to calculate the pose in the lane coordinate system.

### search for matching lanes

This procedure falls back to 2 if 1. fails.
If 2 also fails, fallback is performed to 3.
If 3 also fails, fallback is performed to 4.

1. [Obtain a candidate lane for matching considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L607)
1. [Obtain lanes in the neighborhood of the Entity as matching candidates without considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L608-L610)
1. [Obtain matching candidates before and after the lane matched by considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L611-L628)
1. [Obtain matching candidates that exist in close proximity (0.1m) to the Entity.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L629C10-L629C23)

If all cases from 1~4 fail, the pose in the lane coordinate system will be a calculation failure.

### Calculate pose in lane coordinate system

After narrowing down the candidate lanes by the above procedure, the pose of the lane coordinate system is calculated from the coordinates of the intersection of the horizontal bar extended from the Entity and the Catmull-Rom Spline curve constructed from the center line of the lane for each candidate lane.

![Lane pose calculation](../image/lane_pose_calculation.png "Lane pose calculation.")

The length of the horizontal bar is defined by the type of the entity.

#### Ego Entity
Let $L_m$ be the length of the abscissa used in the lane coordinate system calculation and the tread of the front wheels be  $t_f$ and the tread of the rear wheels be $t_r$.

$$L_m = max(t_r, t_f) * 0.5 + 1.0$$

## Lane pose caluculation when updating frame.

### Ego Entity

### Non-Ego Entity

For non-EgoEntity, the process diverges depending on whether planning is done in the lane coordinate system inside BehaviorPlugin.  
If planning is done in the lane coordinate system, there is no need to convert from the map coordinate system to the lane coordinate system after onUpdate is executed.

#### VehicleEntity (with Behavior-Tree)

#### Vehicle Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated at spawn and is not recalculated thereafter.

#### Pedestrian Entity (with Behavior-Tree)

#### Pedestrian Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated at spawn and is not recalculated thereafter.