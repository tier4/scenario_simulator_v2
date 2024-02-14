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
but the one used in this case is [here](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L544-L554).

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
