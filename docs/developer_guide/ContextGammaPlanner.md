# Context Gamma Planner (Experimental)

!!! warning
    Context Gamma Planner is currently **experimental**. The interface and behavior may change without notice. For production use, we recommend using a mature, existing planner.

## Overview

In the current scenario simulator, NPCs cannot proactively avoid dynamic objects around them. For example, a pedestrian NPC cannot continue moving by avoiding a vehicle even if it detects one ahead.
**Context Gamma Planner** integrates the ORCA (Optimal Reciprocal Collision Avoidance) algorithm into NPC locomotion logic to enable dynamic obstacle avoidance.

## How to Run the Sample

The sample scenario `parked_at_crosswalk` covers a pedestrian crossing a crosswalk. It lets you quickly compare the behavior of the conventional planner with that of Context Gamma.

```bash
ros2 launch cpp_mock_scenarios mock_test.launch.py \
    scenario:=parked_at_crosswalk \
    launch_rviz:=true timeout:=120.0
```

## How to Customize a Scenario

To enable the Context Gamma Planner, simply pass `traffic_simulator::PedestrianBehavior::contextGamma()` as the **4th argument** to the NPC creation function `api_.spawn()`.

Example from the `parked_at_crosswalk` scenario:

```cpp
api_.spawn(
  "bob_gamma",
  traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
  getPedestrianParameters(),
  traffic_simulator::PedestrianBehavior::contextGamma()
);
```

There may be cases where you don’t want a specific NPC to be affected by “consideration” (reciprocal avoidance). In that case, **add** `__CONTEXT_GAMMA_IGNORE__` **to the NPC’s name**.

Example from the `parked_at_crosswalk` scenario:

```cpp
api_.spawn(
  "__CONTEXT_GAMMA_IGNORE__bob_normal",
  traffic_simulator::helper::constructCanonicalizedLaneletPose(34378, 0.0, 0.0),
  getPedestrianParameters()
);
```

## Known Limitations

* Currently supports **pedestrians only**.
