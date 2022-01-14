# NPC Behavior

Behavior logic of the NPC(non-player-character) is plugable.
If you want to use your own behavior, please see [this document](BehaviorPlugin.md).

In this document, we describe how default behavior tree NPC works in traffic environment.
Default behavior tree NPC logics are in [this package](https://github.com/tier4/scenario_simulator_v2/tree/master/simulation/behavior_tree_plugin).

## Vehicle NPC
Behavior tree of vehicle NPC is here.

```mermaid
graph TD
    A[Root]:::behavior --> B(Fallback):::flow
    B --> C[LaneChange]:::behavior
    B --> D(Fallback):::flow
    D --> E[FollowLane]:::behavior
    D --> F(Fallback):::flow
    F --> G[FollowFrontEntity]:::behavior
    F --> H[StopAtTrafficLight]:::behavior
    F --> I[StopAtStopLine]:::behavior
    F --> J[StopAtCrossingEntity]:::behavior
    F --> K[Yield]:::behavior
    F --> L[MoveBackward]:::behavior
    classDef behavior fill:#F5AD1A;
    classDef flow fill:#CFE7E8
    click C "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/lane_change_action.cpp"
    click E "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/follow_lane_action.cpp"
    click G "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/follow_front_entity_action.cpp"
    click H "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/stop_at_traffic_light_action.cpp"
    click I "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/stop_at_stop_line_action.cpp"
    click J "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.cpp"
    click K "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/yield_action.cpp"
    click L "https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/behavior_tree_plugin/src/vehicle/follow_lane_sequence/move_backward_action.cpp"
```

|        Action        |                      Behavior                       |                      Success                       |                                         Failure                                         |
| -------------------- | --------------------------------------------------- | -------------------------------------------------- | --------------------------------------------------------------------------------------- |
| LaneChange           | Changing to target lane.                            | Moved to target lane.                              | Failed to calculate lane change trajectory.                                             |
| FollowLane           | Following lane and moving to goal.                  |                                                    | Lane change was requested, traffic light, stop sing, conflicting entities are detected. |
| FollowFrontEntity    | Following target entity in front of the NPC.        | Target entity was disappeared in front of the NPC. |                                                                                         |
| StopAtTrafficLight   | Stopping at a traffic light until it becomes green. | The traffic light become green.                    |                                                                                         |
| StopAtStopLine       | Stopping at a stop line.                            | The NPC was stopped at stop line.                  | Overrun stop line.                                                                      |
| StopAtCrossingEntity | Stopping at crossing entity.                        | Target entity was crossed.                         |                                                                                         |
| Yield                | Yield to right-of-way entity.                       | Right of way entity is moved.                      |                                                                                         |
| MoveBackward         | Move backward on lane.                              | Another request and new goal point was suggested.  |                                                                                         |


### Behavior
#### LaneChange

By using `API::requestLaneChange` function, you send lane change request to target NPC.
You can send request with these parameters.

##### Target

|     Value      |                                                 Meaning                                                 |
| -------------- | ------------------------------------------------------------------------------------------------------- |
| AbsoluteTarget | Changing lanes to the exact target lanelet. You have to specify `lanelet_id`, `offset`                  |
| RelativeTarget | Changing lanes to the relative target lanelet. You have to specify `entity_name`, `shift`, `direction`. |

###### AbsoluteTarget

|   Value    |                     Meaning                      | Default |
| ---------- | ------------------------------------------------ | ------- |
| lanelet_id | Lanelet id, which you want to changing lanes to. |         |
| offset     | Target offset you want to changing lanes.        | 0       |

`offset` value is calculated under lane coordinate.

###### RelativeTarget

|    Value    |                       Meaning                       | Default |
| ----------- | --------------------------------------------------- | ------- |
| entity_name | Reference entity name.                              |         |
| shift       | Number of lanes counted from reference entity name. | 0       |
| offset      | Target offset you want to changing lanes.           | 0       |

##### TrajectoryShape

| Value  |                   Meaning                    |      Default       |
| ------ | -------------------------------------------- | ------------------ |
| CUBIC  | Changing lanes with cubic spline trajectory. | :heavy_check_mark: |
| LINEAR | Changing lanes with linear trajectory.       |                    |

##### Constraint

|         Value         |                      Meaning                      |      Default       |
| --------------------- | ------------------------------------------------- | ------------------ |
| NONE                  | Just changing lanes if possible.                  | :heavy_check_mark: |
| LATERAL_VELOCITY      | Changing lanes with target lateral velocity.      |                    |
| LONGITUDINAL_DISTANCE | Changing lanes with target longitudinal distance. |                    |
| TIME                  | Changing lanes with target duration.              |                    |

##### Policy

|    Value    |                                   Meaning                                   |      Default       |
| ----------- | --------------------------------------------------------------------------- | ------------------ |
| FORCE       | Changing lanes and fulfilling constraints ignoring dynamics.                | :heavy_check_mark: |
| BEST_EFFORT | Changing lanes and trying to fulfill constraints without ignoring dynamics. |                    |
