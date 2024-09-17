# Lane pose calculation when spawning

If you spawn entities by specifying in the lane coordinate system, [lane pose calculation is skipped](https://github.com/tier4/scenario_simulator_v2/blob/0ddeacd1eab22ffc334202ff9a5c458b5569dd32/simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp#L473-L485).  
Internally, spawning entities by specifying in the lane coordinate system means specifying LaneletPose as the argument pose of the [API::spawn function](https://tier4.github.io/scenario_simulator_v2-api-docs/classtraffic__simulator_1_1API.html).
In the OpenSCENARIO, it is to execute a TeleportAction by specifying the LanePosition. ([e.g](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/test_runner/scenario_test_runner/))

If you spawn entities by specifying in the map coordinate system, [lane pose calculation is performed](https://github.com/tier4/scenario_simulator_v2/blob/0ddeacd1eab22ffc334202ff9a5c458b5569dd32/simulation/traffic_simulator/include/traffic_simulator/entity/entity_manager.hpp#L473-L485).

There are several internal implementations of pose calculation in the lane coordinate system implemented in the [HDmapUtils class](https://tier4.github.io/scenario_simulator_v2-api-docs/classhdmap__utils_1_1HdMapUtils.html).
The one used in this case is [here](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L602-L630).

[This function](https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/simulation/traffic_simulator/src/utils/pose.cpp#L101-L122) and its overloads are commonly used to convert map pose to the lane coordinate system.

## Search for matching lanes

This procedure falls back to 2 if 1 fails.
If 2 also fails, fallback is performed to 3.
If 3 also fails, fallback is performed to 4.

1. [Obtain candidate lanes for matching considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L607)
1. [Obtain lanes in the neighborhood of the Entity as matching candidates without considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L608-L610)
1. [Obtain matching candidates before and after the lane matched by considering the bounding box.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L611-L628)
1. [Obtain matching candidates that exist in close proximity (0.1m) to the Entity.](https://github.com/tier4/scenario_simulator_v2/blob/9f03394f80e5de05cf087db3d00c7be73d27e963/simulation/traffic_simulator/src/hdmap_utils/hdmap_utils.cpp#L629C10-L629C23)

If all cases from 1~4 fail, the pose in the lane coordinate system will be a calculation failure.

## Calculate pose in lane coordinate system

After narrowing down the candidate lanes by the above procedure, the pose of the lane coordinate system is calculated from the coordinates of the intersection of the horizontal bar extended from the Entity and the Catmull-Rom Spline curve constructed from the center line of the lane for each candidate lane.

![Lane pose calculation](../../image/lane_pose_calculation.png "Lane pose calculation.")

### Vehicle / Ego Entity

Let $L_m$ be the length of the horizontal bar used in the lane coordinate system calculation and the tread of the front wheels be $t_f$ and the tread of the rear wheels be $t_r$.

$$L_m = max(t_r, t_f) + 1.0$$

### Pedestrian / MiscObject Entity

Let $L_m$ be the length of the horizontal bar used in the lane coordinate system calculation and width of the bounding box be $L_w$

$$L_m = L_w + 1.0$$
