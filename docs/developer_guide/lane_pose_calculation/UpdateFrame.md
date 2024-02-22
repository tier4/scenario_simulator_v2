# Lane pose calculation when updating frames

If the entity's behavioral logic is planned in the lane coordinate system, skip this process.

## Ego Entity

Since EgoEntity is controlled in map coordinates by Autoware rather than in lane coordinates using a motion plugin, this process is performed at each frame update.  
This process is implemented [here](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/api/api.cpp#L240C9-L240C19) and [here](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L276-L312).

### Search for matching lanes
This process branches off into 2 or 3 depending on the outcome of 1.  
If 3 is executed and fails, fallbacks to 4.

1. [Obtain a candidate lane for matching from Autoware planner output.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L276)
2. [If Autoware has no planner output, try obtaining candidate lanes for matching considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L286-L287)
3. [If Autoware has planner output, try matching considering the Autoware planner output.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L289-L290)
4. [If the step 3 failed, try obtaining candidate lanes for matching considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L292-L293)

### Calculate pose in lane coordinate system

Let $L_m$ be the length of the horizontal bar used in the lane coordinate system calculation and the tread of the front wheels be $t_f$ and the tread of the rear wheels be $t_r$.
See also [here.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/ego_entity.cpp#L278-L284)

$$L_m = max(t_r, t_f) + 2.0$$

![Lane pose calculation](../../image/lane_pose_calculation.png "Lane pose calculation.")

## Non-Ego Entity

For non-EgoEntity, the process diverges depending on whether planning is done in the lane coordinate system inside BehaviorPlugin.  
If planning is done in the lane coordinate system, there is no need to convert from the map coordinate system to the lane coordinate system after update frame function is executed.

### VehicleEntity (with Behavior-Tree)

Only during `follow_polyline_trajectory` execution is planning performed in map coordinate system, but [lane coordinate system calculations are not performed](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/behavior/follow_trajectory.cpp#L546)

### Vehicle Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.

### Pedestrian Entity (with Behavior-Tree)

Planning is done in map coordinates in the `walk_straight` and `follow_polyline_trajectory` actions. 

In the `walk_straight` Action, [the pose in the lane coordinate system is calculated.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L56)  
The procedure for calculating the pose in the lane coordinate system at this time is as follows.  

If lane matching was successful in the previous frame, do 1, otherwise do 2

1.[Set the length of the horizontal bar to 2.0 + the width of the bounding box and calculate the pose in the lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/a2c04ee2446f80aeacfe59fc87a6737ae18692cc/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L72-L77)  
2.[Calculate the pose in the lane coordinate system considering the size of the BoundingBox of the Entity](https://github.com/tier4/scenario_simulator_v2/blob/a2c04ee2446f80aeacfe59fc87a6737ae18692cc/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L79-L85)  

If calculation 1 or 2 fails,  
3.[Set the length of the horizontal bar to 4.0 and calculate the pose in the lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L86-L91)  
If 1 or 2 are successful, then 3 is skipped.

If the pose could not be calculated in the lane coordinate system by considering up to the result of 3, [the pose calculation in the lane coordinate system is a failure](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L125).

[Canonicalize pose in lane coordinate system](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L94) to determine the final pose in the lane coordinate system.

In the `follow_polyline_trajectory` Action, [lane coordinate system calculations are not performed](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/behavior/follow_trajectory.cpp#L546)

### Pedestrian Entity (with Do-Nothing)

While the do-nothing behavior plugin is running, Entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.

### MiscObjectEntity

Misc object entity does not move, so the lane coordinate system is calculated when spawning entity and is not recalculated thereafter.
