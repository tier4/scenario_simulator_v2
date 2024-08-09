# Lane pose calculation when updating frames

If the entity's behavioral logic is planned in the lane coordinate system, skip this process.

## Ego Entity

Since EgoEntity is controlled in map coordinates by Autoware rather than in lane coordinates using a motion plugin, this process is performed at each frame update.  
This process is implemented [API](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/api/api.cpp#L302) -> [EgoEntity::setMapPose](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/entity/ego_entity.cpp#L294-L300) -> [EgoEntity::setStatus](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/entity/ego_entity.cpp#L309-L320).

### Search for matching lanes
This process branches off into 2.  

1. [Obtain a candidate lane for matching from Autoware planner output.](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/entity/ego_entity.cpp#L296)
2. [If Autoware has planner output, try matching considering the Autoware planner output.](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/utils/pose.cpp#L110-L111)
3. [If Autoware has no planner output or 2 fails, try obtaining candidate lanes for matching considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/utils/pose.cpp#L114-L115)

### Calculate pose in lane coordinate system

Let $L_m$ be the length of the horizontal bar used in the lane coordinate system calculation and the tread of the front wheels be $t_f$ and the tread of the rear wheels be $t_r$.
See also [here.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L278-L284)

$$L_m = max(t_r, t_f) + 2.0$$

![Lane pose calculation](../../image/lane_pose_calculation.png "Lane pose calculation.")

## Non-Ego Entity

For non-Ego Entities, the pose calculation process is carried out immediately after getting the updated EntityStatus from BehaviorTree.
If the planning is done in the lane coordinate system, the value of `lanelet_pose_valid` is equal to true, which causes [only position canonicalization](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/data_type/entity_status.cpp#L75-L76) to be performed.

### VehicleEntity (with Behavior-Tree)

Only during `follow_polyline_trajectory` execution is planning performed in map coordinate system, but [lane coordinate system calculations are not performed](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/behavior/follow_trajectory.cpp#L561)

This process is implemented [VehicleEntity::onUpdate](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/entity/vehicle_entity.cpp#L164-L165) -> [EntityBase::setStatus](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/entity/entity_base.cpp#L520-L523) -> [CanonicalizedEntityStatus::set](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/data_type/entity_status.cpp#L66-L84).

### Vehicle Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.

### Pedestrian Entity (with Behavior-Tree)

Planning is done in map coordinates in the `walk_straight` and `follow_polyline_trajectory` actions. 

In the `walk_straight` Action, [the pose in the lane coordinate system is calculated.](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L59-L62)  
The procedure for calculating the pose in the lane coordinate system at this time is as follows.  

If lane matching was successful in the previous frame (entity is currently on some unique lane), [do 1, otherwise do 2](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/traffic_simulator/src/utils/pose.cpp#L293-L297):

1.[Set the length of the horizontal bar (to the width of the bounding box + 1.0) and calculate the pose in the lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/utils/pose.cpp#L110-L111)  
2.[Calculate the pose in the lane coordinate system considering the size of the BoundingBox of the Entity](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/utils/pose.cpp#L114-L115)  

If calculation 1 or 2 fails,  
3.[Set the length of the horizontal bar to **2.0** and calculate the pose in the lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/traffic_simulator/src/utils/pose.cpp#L302-L303)  
If 1 or 2 are successful, then 3 is skipped.

If the pose could not be calculated in the lane coordinate system by considering up to the result of 3, [the pose calculation in the lane coordinate system is a failure](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/traffic_simulator/src/utils/pose.cpp#L335).

[Canonicalize pose in lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/traffic_simulator/src/utils/pose.cpp#L303) to determine the final pose in the lane coordinate system.
If final canonicalize failed, [set end of road lanelet pose](https://github.com/tier4/scenario_simulator_v2/blob/6f87603e2aaaddfd8ceb08645cb3933fe3b74515/simulation/traffic_simulator/src/utils/pose.cpp#L302-L332).

In the `follow_polyline_trajectory` Action, [lane coordinate system calculations are not performed](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/behavior/follow_trajectory.cpp#L546)

### Pedestrian Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.

### MiscObjectEntity

Misc object entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.
