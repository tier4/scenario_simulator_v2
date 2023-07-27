# Release Notes

## Difference between the latest release and master

Major Changes :race_car: :red_car: :blue_car:

| Feature | Brief summary | Category | Pull request | Contributor |
|---------|---------------|----------|--------------|-------------|
|         |               |          |              |             |

Bug Fixes:bug:

| Feature | Brief summary | Category | Pull request | Contributor |
|---------|---------------|----------|--------------|-------------|
|         |               |          |              |             |

Minor Tweaks :oncoming_police_car:

| Feature | Brief summary | Category | Pull request | Contributor |
|---------|---------------|----------|--------------|-------------|
|         |               |          |              |             |

## Version 0.7.0

Major Changes :race_car: :red_car: :blue_car:

| Feature                                                  | Brief summary                                                                                                                                             | Category                                        | Pull request                                                      | Contributor                                   |
|----------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------|-------------------------------------------------------------------|-----------------------------------------------|
| OpenSCENARIO 1.2 `UserDefinedAction.CustomCommandAction` | Added `FaultInjectionAction@v2`, a CustomCommandAction for raising a fault injection event with specified error level.                                    | `openscenario_interpreter`                      | [#1002](https://github.com/tier4/scenario_simulator_v2/pull/1002) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO 1.2 `UserDefinedAction.CustomCommandAction` | Added `V2ITrafficSignalStateAction`, an implementation of `TrafficSignalState` for V2I traffic signals.                                                   | `openscenario_interpreter`, `traffic_simulator` | [#1004](https://github.com/tier4/scenario_simulator_v2/pull/1004) | [HansRobo](https://github.com/HansRobo)       |
| Add do noting behavior plugin                            | Add do nothing behavior for driving the simulator from Autoware rosbag data.                                                                              | `do_nothing_plugin`,`traffic_simulator`         | [#1001](https://github.com/tier4/scenario_simulator_v2/pull/1011) | [hakuturu583](https://github.com/hakuturu583) |
| OpenSCENARIO 1.2 `UserDefinedAction.CustomCommandAction` | Added `RequestToCooperateCommandAction@v1`, a CustomCommandAction to simulate an operation by humans or external applications for a request to cooperate. | `openscenario_interpreter`, `concealer`         | [#1013](https://github.com/tier4/scenario_simulator_v2/pull/1013) | [HansRobo](https://github.com/HansRobo)       |
+ OpenSCENARIO 1.2 `FollowTrajectoryAction`                | Support `FollowTrajectoryAction` only for vehicle entities directly controlled by the simulator.                                                          | `openscenario_interpreter`, `traffic_simulator` | [#906](https://github.com/tier4/scenario_simulator_v2/pull/906)   | [yamacir-kit](https://github.com/yamacir-kit) |

Bug Fixes:bug:

| Feature | Brief summary | Category | Pull request | Contributor |
|---------|---------------|----------|--------------|-------------|
|         |               |          |              |             |

Minor Tweaks :oncoming_police_car:

| Feature | Brief summary | Category | Pull request | Contributor |
|---------|---------------|----------|--------------|-------------|
|         |               |          |              |             |

## Version 0.6.8

Major Changes :race_car: :red_car: :blue_car:

| Feature                                                              | Brief summary                                                                                                                                                                                            | Category                   | Pull request                                                    | Contributor                                   |
|----------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Jerk simulation and planning                                         | Start supporting jerk simulation and enable limit jerk while effecting `API::requestSpeedChange` function.                                                                                               | `traffic_simulator`        | [#909](https://github.com/tier4/scenario_simulator_v2/pull/909) | [hakuturu583](https://github.com/hakuturu583) |
| OpenSCENARIO 1.2 `SpeedProfileAction`                                | OpenSCENARIO 1.2 `SpeedProfileAction` is now officially supported.                                                                                                                                       | `openscenario_interpreter` | [#928](https://github.com/tier4/scenario_simulator_v2/pull/928) | [yamacir-kit](https://github.com/yamacir-kit) |
| Randomization of the positions of detected objects                   | By setting `detectedObjectPositionStandardDeviation` to `Controller.Properties.Property`, it is now possible to add noise to the position of other entities that Autoware recognizes.                    | `openscenario_interpreter` | [#937](https://github.com/tier4/scenario_simulator_v2/pull/937) | [yamacir-kit](https://github.com/yamacir-kit) |
| `traffic_simulator`'s API `getLateralDistance`                       | Enable getting lateral distance via `API` class.                                                                                                                                                         | `traffic_simulator`        | [#945](https://github.com/tier4/scenario_simulator_v2/pull/945) | [hakuturu583](https://github.com/hakuturu583) |
| OpenSCENARIO 1.2 `DistanceCondition` and `RelativeDistanceCondition` | `DistanceCondition` and `RelativeDistanceCondition` now support distance measurement on the lateral axis of the lane coordinate system.                                                                  | `openscenario_interpreter` | [#962](https://github.com/tier4/scenario_simulator_v2/pull/962) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO 1.2 `Controller.Properties.Property`                    | By setting `detectedObjectMissingProbability` to `Controller.Properties.Property`, it is now possible to lost object recognition data with a specified probability.                                      | `openscenario_interpreter` | [#973](https://github.com/tier4/scenario_simulator_v2/pull/973) | [yamacir-kit](https://github.com/yamacir-kit) |
| UserDefinedValueCondition `RelativeHeadingCondition`                 | Added one-argument version to `RelativeHeadingCondition`. This version of `RelativeHeadingCondition` returns the lane coordinate system heading of the entity with the name given in the first argument. | `openscenario_interpreter` | [#978](https://github.com/tier4/scenario_simulator_v2/pull/978) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO 1.2 `Controller.Properties.Property`                    | Added support for delaying the publication of object detection data by setting the value `detectedObjectPublishingDelay` (in seconds) to `Controller.Properties.Property`.                               | `openscenario_interpreter` | [#986](https://github.com/tier4/scenario_simulator_v2/pull/986) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO 1.2 `EnvironmentAction`                                 | The parsing of `EnvironmentAction` is now supported                                                                                                                                                      | `openscenario_interpreter` | [#980](https://github.com/tier4/scenario_simulator_v2/pull/980) | [f0reachARR](https://github.com/f0reachARR)   |

Bug Fixes:bug:

| Feature               | Brief summary                                                                               | Category                                    | Pull request                                                    | Contributor                                   |
|-----------------------|---------------------------------------------------------------------------------------------|---------------------------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Behavior "stop"       | Update NPC behavior and avoid overrun.                                                      | `traffic_simulator`, `behavior_tree_plugin` | [#946](https://github.com/tier4/scenario_simulator_v2/pull/946) | [hakuturu583](https://github.com/hakuturu583) |
| `vehicle_model`       | Import bug fixes from `simple_planning_simulator` in Autoware.Universe for `vehicle_model`. | `traffic_simulator`                         | [#936](https://github.com/tier4/scenario_simulator_v2/pull/936) | [HansRobo](https://github.com/HansRobo)       |
| Next/previous lanelet | Enable get next/previous lanelet with `road_shoulder` subtype.                              | `traffic_simulator`                         | [#963](https://github.com/tier4/scenario_simulator_v2/pull/963) | [hakuturu583](https://github.com/hakuturu583) |

Minor Tweaks :oncoming_police_car:

| Feature                                  | Brief summary                                                                                                               | Category                                | Pull request                                                    | Contributor                                   |
|------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|-----------------------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| `topic_status_checker`                   | Delete `topic_status_checker` package.                                                                                      | common                                  | [#921](https://github.com/tier4/scenario_simulator_v2/pull/921) | [HansRobo](https://github.com/HansRobo)       |
| Health check                             | Supported a function to monitor whether the running simulator has become unresponsive.                                      | common                                  | [#932](https://github.com/tier4/scenario_simulator_v2/pull/932) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO `UserDefinedValueCondition` | Remove the message type package `openscenario_msgs` and moved its contents to an external repository `tier4_autoware_msgs`. | `openscenario_interpreter`              | [#874](https://github.com/tier4/scenario_simulator_v2/pull/874) | [yamacir-kit](https://github.com/yamacir-kit) |
| Legacy parameter distribution            | Fix `openscenario_utility.convert` to not to generate too long filename.                                                    | `openscenario_utility`                  | [#972](https://github.com/tier4/scenario_simulator_v2/pull/972) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO `UserDefinedValueCondition` | Support ADAPI interface for minimum-risk-maneuver state with backward compatibility for legacy emergency state              | `openscenario_interpreter`, `concealer` | [#975](https://github.com/tier4/scenario_simulator_v2/pull/975) | [HansRobo](https://github.com/HansRobo)       |
| Port management                          | Change communication between `traffic_simulator` and `simple_sensor_simulator` from multi-port to single-port.              | `simulation_interface`                  | [#981](https://github.com/tier4/scenario_simulator_v2/pull/981) | [dmoszynski](https://github.com/dmoszynski)   |
| OpenSCENARIO `model3d` in entity object  | Add `model3d` attribute parsing in entity objects.                                                                          | `openscenario_interpreter`              | [#977](https://github.com/tier4/scenario_simulator_v2/pull/977) | [f0reachARR](https://github.com/f0reachARR)   |

## Version 0.6.7

Major Changes :race_car: :red_car: :blue_car:

| Feature                                   | Brief summary                                                                                                                                                     | Category                                                     | Pull request                                                    | Contributor                                                                                  |
|-------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|-----------------------------------------------------------------|----------------------------------------------------------------------------------------------|
| Engagement                                | Changed time management to send engage as soon as autoware state transitions to `WAITING_FOR_ENGAGE` and start simulation as soon as it transitions to `DRIVING`. | `traffic_simulator`, `concealer`, `openscenario_interpreter` | [#823](https://github.com/tier4/scenario_simulator_v2/pull/823) | [hakuturu583](https://github.com/hakuturu583), [yamacir-kit](https://github.com/yamacir-kit) |
| Publication of Acceleration               | Updated concealer to publish acceleration information for Autoware.                                                                                               | `concealer`                                                  | [#875](https://github.com/tier4/scenario_simulator_v2/pull/875) | [yamacir-kit](https://github.com/yamacir-kit)                                                |
| OpenSCENARIO 1.2 `SpeedProfileAction`     | Added experimental support for some features of `SpeedProfileAction`.                                                                                             | `openscenario_interpreter`                                   | [#898](https://github.com/tier4/scenario_simulator_v2/pull/898) | [yamacir-kit](https://github.com/yamacir-kit)                                                |
| `requestSpeedChange` with time constraint | Start supporting `API::requestSpeedChange` function with time constraint. this feature only supports `continuous = false`.                                        | `traffic_simulator`                                          | [#901](https://github.com/tier4/scenario_simulator_v2/pull/901) | [hakuturu583](https://github.com/hakuturu583)                                                |
| OpenSCENARIO 1.2 `DynamicConstraints`     | Added message type `DynamicConstraints` corresponding to OpenSCENARIO 1.2 `DynamicConstraints` to `traffic_simulator_msg`.                                        | `traffic_simulator`                                          | [#900](https://github.com/tier4/scenario_simulator_v2/pull/900) | [yamacir-kit](https://github.com/yamacir-kit)                                                |

Bug Fixes:bug:

| Feature                      | Brief summary                                                                                                          | Category                   | Pull request                                                    | Contributor                                   |
|------------------------------|------------------------------------------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Interpreter state transition | Fixed a problem with error handling and state transition when an error occurred in the activation phase of simulation. | `openscenario_interpreter` | [#881](https://github.com/tier4/scenario_simulator_v2/pull/881) | [yamacir-kit](https://github.com/yamacir-kit) |
| Bounding box position        | Fixed bounding box shifting on Rviz                                                                                    | `simple_sensor_simulator`  | [#888](https://github.com/tier4/scenario_simulator_v2/pull/888) | [shouth](https://github.com/shouth)           |

Minor Tweaks :oncoming_police_car:

| Feature                                    | Brief summary                                                                                              | Category                  | Pull request                                                    | Contributor                                   |
|--------------------------------------------|------------------------------------------------------------------------------------------------------------|---------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| `lanelet2_extension_psim`                  | Remove `external/lanelet2_extension_psim` and add dependency to `lanelet2_extension` in `autoware_common`. | `external`                | [#863](https://github.com/tier4/scenario_simulator_v2/pull/863) | [HansRobo](https://github.com/HansRobo)       |
| `OccupancyGridSensor`                      | Improve performance of occupancy grid generation by changing internal data structure.                      | `simple_sensor_simulator` | [#866](https://github.com/tier4/scenario_simulator_v2/pull/866) | [shouth](https://github.com/shouth)           |
| `traffic_simulator`'s API `spawn`          | Changed the `spawn` API to require the initial coordinates of the entity.                                  | `traffic_simulator`       | [#896](https://github.com/tier4/scenario_simulator_v2/pull/896) | [yamacir-kit](https://github.com/yamacir-kit) |
| `traffic_simulator`'s distance calculation | Update the `traffic_simulator` distance measurement API to remove the default range limit of 100m maximum. | `traffic_simulator`       | [#908](https://github.com/tier4/scenario_simulator_v2/pull/908) | [yamacir-kit](https://github.com/yamacir-kit) |
| `topic_status_checker`                     | Add `topic_status_checker` package to check node state from outside via topic status                       | `common`                  | [#912](https://github.com/tier4/scenario_simulator_v2/pull/912) | [HansRobo](https://github.com/HansRobo)       |

## Version 0.6.6

Major Changes :race_car: :red_car: :blue_car:

| Feature                                                           | Brief summary                                                                                               | Category                   | Pull request                                                    | Contributor                                                                        |
|-------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|------------------------------------------------------------------------------------|
| Add `API::getRelativePose()` argument for function                | Add `API::getRelativePose()` argument for getting relative distance between lanelet pose and entity.        | `traffic_simulator`        | [#809](https://github.com/tier4/scenario_simulator_v2/pull/809) | [hakuturu583](https://github.com/hakuturu583)                                      |
| RTC (Request to Cooperate)                                        | Experimental support for automatic approval of requests to cooperate from Autoware.Universe.                | `concealer`                | [#818](https://github.com/tier4/scenario_simulator_v2/pull/818) | [yamacir-kit](https://github.com/yamacir-kit)                                      |
| Experimental UserDefinedValueCondition `RelativeHeadingCondition` | Update `UserDefinedValueCondition` to support new experimental condition `RelativeHeadingCondition`.        | `openscenario_interpreter` | [#830](https://github.com/tier4/scenario_simulator_v2/pull/830) | [yamacir-kit](https://github.com/yamacir-kit)                                      |
| OpenSCENARIO `ValueConstraint`, `ValueConstraintGroup`            | Add support for `ValueConstraint` and `ValueConstraintGroup`.                                               | `openscenario_interpreter` | [#847](https://github.com/tier4/scenario_simulator_v2/pull/847) | [HansRobo](https://github.com/HansRobo)                                            |
| Add `API::getTraveledDistance()`                                  | Add `API::getTraveledDistance()` to obtain how far an entity traveled, and remove `TraveledDistanceMetric`. | `traffic_simulator`        | [#858](https://github.com/tier4/scenario_simulator_v2/pull/858) | [hakuturu583](https://github.com/hakuturu583), [shouth](https://github.com/shouth) |

| Improve ego lane matching logic                                   | Retry matching to lanelet without using route information from Autoware.                             | `traffic_simulator`        | [#864](https://github.com/tier4/scenario_simulator_v2/pull/864) | [hakuturu583](https://github.com/hakuturu583) |

Bug Fixes:bug:

| Feature                                         | Brief summary                                                                                                                                                           | Category            | Pull request                                                    | Contributor                                   |
|-------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Change the stop position of the vehicle entity. | Change the stop position of the vehicle entity at the stop line and traffic light.                                                                                      | `traffic_simulator` | [#822](https://github.com/tier4/scenario_simulator_v2/pull/822) | [hakuturu583](https://github.com/hakuturu583) |
| Fix trajectory point.                           | Fix trajectory point at the end of the trajectory.                                                                                                                      | `traffic_simulator` | [#836](https://github.com/tier4/scenario_simulator_v2/pull/836) | [hakuturu583](https://github.com/hakuturu583) |
| Fix problems in getting right of way lane.      | Remove self lanelet id from right of way lanelet id.                                                                                                                    | `traffic_simulator` | [#834](https://github.com/tier4/scenario_simulator_v2/pull/834) | [hakuturu583](https://github.com/hakuturu583) |
| RTC (Request to Cooperate)                      | Changed to execute the service call in an independent thread to fix the problem that the main thread is stopped for about 1 second due to the query to the RTC service. | `concealer`         | [#841](https://github.com/tier4/scenario_simulator_v2/pull/841) | [yamacir-kit](https://github.com/yamacir-kit) |

Minor Tweaks :oncoming_police_car:

| Feature                        | Brief summary                                                                                                                                           | Category                   | Pull request                                                                                                                     | Contributor                                                                                    |  |
|--------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------|----------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------|--|
| OpenSCENARIO simulator core    | Changed to treat `traffic_simulator` as "Simulator Core" based on OpenSCENARIO standard Basic architecture components.                                  | `openscenario_interpreter` | [#783](https://github.com/tier4/scenario_simulator_v2/pull/783)                                                                  | [yamacir-kit](https://github.com/yamacir-kit)                                                  |  |
| Option `--record`              | Exclude a too large topic `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection` from bag recording. | `openscenario_interpreter` | [#829](https://github.com/tier4/scenario_simulator_v2/pull/829)                                                                  | [yamacir-kit](https://github.com/yamacir-kit)                                                  |  |
| Move to github docker registry | Move from dockerhub to github docker registry.                                                                                                          | `docker`                   | [#843](https://github.com/tier4/scenario_simulator_v2/pull/843), [#844](https://github.com/tier4/scenario_simulator_v2/pull/844) | [hakuturu583](https://github.com/hakuturu583), [kenji-miyake](https://github.com/kenji-miyake) |  |
| Update `lanelet2_extension`    | Copy `lanelet2_extension` of Autoware.Universe 0.3.7 as `lanelet2_extension_psim`.                                                                      | `lanelet2_extension`       | [#850](https://github.com/tier4/scenario_simulator_v2/pull/850)                                                                  | [yamacir-kit](https://github.com/yamacir-kit)                                                  |  |
| Remove trivial metrics         | Remove `StandstillMetric` and `CollisionMetric`.                                                                                                        | `traffic_simulator`        | [#854](https://github.com/tier4/scenario_simulator_v2/pull/854)                                                                  | [shouth](https://github.com/shouth)                                                            |  |

## Version 0.6.5

Major Changes :race_car: :red_car: :blue_car:

| Feature                                      | Brief summary                                                                                                | Category                   | Pull request                                                    | Contributor                                   |
|----------------------------------------------|--------------------------------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| OpenSCENARIO `UserDefinedValueCondition`     | Add condition to determine the state of the turn indicators. (unique to Autoware.Universe)                   | `openscenario_interpreter` | [#777](https://github.com/tier4/scenario_simulator_v2/pull/777) | [HansRobo](https://github.com/HansRobo)       |
| OpenSCENARIO `UserDefinedValueCondition`     | Add condition to determine the emergency state of the Autoware.Universe.                                     | `openscenario_interpreter` | [#760](https://github.com/tier4/scenario_simulator_v2/pull/760) | [HansRobo](https://github.com/HansRobo)       |
| OpenSCENARIO `Storyboard`                    | The state transition of StoryboardElement no longer consumes simulation time.                                | `openscenario_interpreter` | [#758](https://github.com/tier4/scenario_simulator_v2/pull/740) | [yamacir-kit](https://github.com/yamacir-kit) |
| `EgoEntity`'s Simulation model               | Fixed EgoEntity's simulation model to properly set gear information.                                         | `traffic_simulator`        | [#792](https://github.com/tier4/scenario_simulator_v2/pull/792) | [yamacir-kit](https://github.com/yamacir-kit) |
| OpenSCENARIO `Event`                         | Allows the omission of Event.StartTrigger. if it is omitted, interpreter uses one which always returns True. | `openscenario_interpreter` | [#774](https://github.com/tier4/scenario_simulator_v2/pull/774) | [HansRobo](https://github.com/HansRobo)       |
| Add `OccupancyGridSensor`                    | Add `OccupancyGridSensor` for publishing nav_msgs/msg/OccupancyGrid to the Autoware.                         | `simple_sensor_simulator`  | [#795](https://github.com/tier4/scenario_simulator_v2/pull/797) | [hakuturu583](https://github.com/hakuturu583) |
| Add `API::getDistanceToLaneBound()` function | Add `API::getDistanceToLaneBound()` function for getting distance from entity polygon to lane boundary.      | `traffic_simulator`        | [#795](https://github.com/tier4/scenario_simulator_v2/pull/807) | [hakuturu583](https://github.com/hakuturu583) |
| Support Humble distribution                  | Support new ROS 2 LTS distribution, Humble Hawksbill.                                                        |                            | [#792](https://github.com/tier4/scenario_simulator_v2/pull/793) | [wep21](https://github.com/wep21)             |

Bug Fixes:bug:

| Feature                                    | Brief summary                                                                                         | Category                   | Pull request                                                    | Contributor                                   |
|--------------------------------------------|-------------------------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Fix termination processing                 | Properly terminate the interpreter when the Autoware process exits abnormally                         | `openscenario_interpreter` | [#750](https://github.com/tier4/scenario_simulator_v2/pull/750) | [kyabe2718](https://github.com/kyabe2718)     |
| Fix collision2D function in `HermiteCurve` | Previous algorithm was failed to check collision when the line is almost parallel to the x and y axis | `traffic_simulator`        | [#795](https://github.com/tier4/scenario_simulator_v2/pull/795) | [hakuturu583](https://github.com/hakuturu583) |

Minor Tweaks :oncoming_police_car:

| Feature                          | Brief summary                                                                                                                                                                   | Category            | Pull request                                                    | Contributor                                                                             |
|----------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------|-----------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| Accessor to the class `Autoware` | A new member function `asAutoware` has been added to access the class `Autoware`, and instead the member functions specific to `EgoEntity` have been removed from `EntityBase`. | `traffic_simulator` | [#796](https://github.com/tier4/scenario_simulator_v2/pull/796) | [yamacir-kit](https://github.com/yamacir-kit)                                           |
| Autoware API                     | Changed the engagement service to Autoware from `/api/autoware/set/engage` to `/api/external/set/engage`.                                                                       | `concealer`         | [#804](https://github.com/tier4/scenario_simulator_v2/pull/804) | [yn-mrse](https://github.com/yn-mrse) and [yamacir-kit](https://github.com/yamacir-kit) |

## Version 0.6.4

Major Changes :race_car: :red_car: :blue_car:

| Feature                                       | Brief summary                                                                                                                                                            | Category                                                          | Pull request                                                    | Contributor                                   |
|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| Entity label publisher                        | Enable specify and publish NPC semantic information such as `MOTORCYCLE`, `TRUCK`, `BUS`.                                                                                | `traffic_simulator`                                               | [#726](https://github.com/tier4/scenario_simulator_v2/pull/726) | [hakuturu583](https://github.com/hakuturu583) |
| ROS 2 Launch XML-like substitution syntax     | Add new substitution syntax `$(ros2 <argument>...)`.                                                                                                                     | `openscenario_interpreter`                                        | [#727](https://github.com/tier4/scenario_simulator_v2/pull/727) | [yamacir-kit](https://github.com/yamacir-kit) |
| `Filter by range` option                      | Add `filter by range` option for detection sensor. If false, simulate detection result by lidar detection. If true, simulate detection result by range.                  | `traffic_simulator`                                               | [#729](https://github.com/tier4/scenario_simulator_v2/pull/729) | [hakuturu583](https://github.com/hakuturu583) |
| Optimization of the trajectory calculation    | Hermite curve optimization, entities' trajectories calculated only when route changes                                                                                    | `traffic_simulator`, `behavior_tree_plugin`                       | [#708](https://github.com/tier4/scenario_simulator_v2/pull/708) | [danielm1405](https://github.com/danielm1405) |
| OpenSCENARIO `Controller.Properties.Property` | Support new controller property `isClairvoyant`.                                                                                                                         | `openscenario_interpreter`                                        | [#735](https://github.com/tier4/scenario_simulator_v2/pull/735) | [yamacir-kit](https://github.com/yamacir-kit) |
| Lane matching improvement for EgoEntity       | Subscribe route information from Autoware (topic : `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id`) and try matching to on route lanelet. | `traffic_simulator`                                               | [#733](https://github.com/tier4/scenario_simulator_v2/pull/733) | [hakuturu583](https://github.com/hakuturu583) |
| ZMQ client can connect through the network    | Add `"simulator_host"` argument to define the address of the simulator host that ZMQ tries to connect to                                                                 | `simulation_interface`, `traffic_simulator`, `random_test_runner` | [#690](https://github.com/tier4/scenario_simulator_v2/pull/690) | [danielm1405](https://github.com/danielm1405) |
| Traffic signals                               | Support for multiple simultaneous lights at traffic signals and addition of color information to arrow signals.                                                          | `openscenario_interpreter`, `traffic_simulator`                   | [#740](https://github.com/tier4/scenario_simulator_v2/pull/740) | [yamacir-kit](https://github.com/yamacir-kit) |
| Reset acceleration after speed was changed    | reset acceleration and deceleration only while speed change was enabled.                                                                                                 | `traffic_simulator`                                               | [#752](https://github.com/tier4/scenario_simulator_v2/pull/752) | [hakuturu583](https://github.com/hakuturu583) |
| Traffic signals                               | Thereafter, a Lanelet ID that can be interpreted as either a relation ID or a way ID is treated as a relation ID.                                                        | `traffic_simulator`                                               | [#759](https://github.com/tier4/scenario_simulator_v2/pull/740) | [yamacir-kit](https://github.com/yamacir-kit) |
| Optimization of the trajectory calculation    | Calculate a segment of trajectory spline instead of recalculating it from scratch                                                                                        | `traffic_simulator`, `behavior_tree_plugin`                       | [#710](https://github.com/tier4/scenario_simulator_v2/pull/710) | [danielm1405](https://github.com/danielm1405) |

Bug Fixes:bug:

| Feature                                | Brief summary                                                                       | Category                   | Pull request                                                    | Contributor                                   |
|----------------------------------------|-------------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| OpenSCENARIO `Storyboard.Init.Actions` | Fix `Init.Actions.GlobalAction` and `Init.Actions.UserDefinedAction` to work.       | `openscenario_interpreter` | [#734](https://github.com/tier4/scenario_simulator_v2/pull/734) | [yamacir-kit](https://github.com/yamacir-kit) |
| Fix waypoint height                    | Height of the NPC waypoint was 0. Get waypoint height from center point of lanelet. | `traffic_simulator`        | [#718](https://github.com/tier4/scenario_simulator_v2/pull/718) | [hakuturu583](https://github.com/hakuturu583) |
| Fix calculateStopDistance function     | While calculating stop distance, deceleration was always 5.                         | `behavior_tree_plugin`     | [#747](https://github.com/tier4/scenario_simulator_v2/pull/747) | [hakuturu583](https://github.com/hakuturu583) |

Minor Tweaks :oncoming_police_car:

| Feature                   | Brief summary                                                                     | Category                   | Pull request                                                    | Contributor                                   |
|---------------------------|-----------------------------------------------------------------------------------|----------------------------|-----------------------------------------------------------------|-----------------------------------------------|
| OpenSCENARIO `Storyboard` | Update interpreter's main loop to not to stop even if `Storyboard` was completed. | `openscenario_interpreter` | [#720](https://github.com/tier4/scenario_simulator_v2/pull/720) | [yamacir-kit](https://github.com/yamacir-kit) |

## Version 0.6.3
- Speed up metrics manger class in order to reduce frame-rate dropping problem. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/680))
- Fix problem in warping NPCs spawned in world coordinate. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/686))
- End of support for ROS 2 Foxy and Autoware.Auto ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/696)).
- Fix problem in ideal steer acc geared dynamics model. Vehicle was pulled back very slowly even if the vehicle is stopped. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/698))
- Fix problem in getFrontEntityName function, consider yaw difference while stopping at crossing entity. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/703))
- Fix problem in delay steer acc geared dynamics model. Vehicle was pulled back very slowly even if the vehicle is stopped. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/707))
- Start considering offset in collision detection in crossing entity. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/709))
- Fix waypoint height, the height of each waypoint was zero. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/718))

## Version 0.6.2
- Start supporting linear trajectory shape while changing lane. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/661))
- Fix context panel to display simulation time. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Support new vehicle_model_type `DELAY_STEER_VEL` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/660)).
- Add lateral velocity and time constraint. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/664)).
- Update syntax `SpeedAction` to use `API::requestSpeedChange` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/665)).
- Enable control multiple traffic lights by relation ID from lanelet2 map. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/667)).
- Start supporting requestSpeedChange API in pedestrian. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/670)).
- Update syntax `LaneChangeAction` to use `API::requestLaneChange` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/668)).
- Fix logic in calculating lane change trajectory shape. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/671)).

## Version 0.6.1
- Add API::requestSpeedChange function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/618))
- Fix syntax `Controller` to not to overwrite `traffic_simulator`'s `DriverModel` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/649)).
- Update simulation models ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/652)).
- Enable request relative speed change. ([link](https://github.com/tier4/scenario_simulator_v2/pull/654))

## Version 0.6.0
- Start supporting Autoware.Universe. ([link](https://github.com/tier4/scenario_simulator_v2/pull/614))

## Version 0.5.8
- Remove newton methods in getSValue function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/612))
- Set withLaneChange parameter as false. ([link](https://github.com/tier4/scenario_simulator_v2/pull/618))
- Change traffic light topic name to "/perception/traffic_light_recognition/traffic_light_states" ([link](https://github.com/tier4/scenario_simulator_v2/pull/621))
- Remove hard coded parameters in behavior tree plugin and use acceleration and deceleration value in traffic_simulator_msgs/msg/DriverModel. ([link](https://github.com/tier4/scenario_simulator_v2/pull/624))
- Enable build with foxy latest version of behavior_tree_cpp_v3. ([link](https://github.com/tier4/scenario_simulator_v2/pull/625))
- Add API::getDriverModel function. ([link](https://github.com/tier4/scenario_simulator_v2/pull/626))
- Add Random test runner. [Link](https://github.com/tier4/scenario_simulator_v2/pull/619) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Support new Autoware architecture `Universe` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/614)).

## Version 0.5.7
- Fix problem in getNormalVector function. (Contribution by [Utaro-M](https://github.com/Utaro-M)).

## Version 0.5.6
- Fix context panel to display conditions' status. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Add NPC Behavior Plugin and Behavior-Tree Plugin for Vehicle and Pedestrian. ([link](https://github.com/tier4/scenario_simulator_v2/pull/566))
- Rename package `openscenario_msgs` to `traffic_simulator_msgs`
- Start supporting galactic environment with Docker. ([link](https://github.com/tier4/scenario_simulator_v2/pull/576))
- Update `UserDefinedValueCondition` to subscribe ROS 2 topic if path-like name given ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/567)).
- Add new package `openscenario_msgs` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/567)).
- Add getNearbyLaneletIds and filterLaneletIds function in HdMapUtils class. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/585))
- Fix calculating way of longitudinal distance. If forward distance and backward distance was calculated, choose smaller one. ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/586))

## Version 0.5.5
- Fix syntax `ReachPositionCondition` to not to use `API::reachPosition` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/561)).

## Version 0.5.4
- Revert [PR #544](https://github.com/tier4/scenario_simulator_v2/pull/544) ([link](https://github.com/tier4/scenario_simulator_v2/pull/557))
- Add context panel to display conditions' status. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Add TIER IV extension `conditionEdge="sticky"` to `OpenSCENARIO-1.1.xsd` ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/553)).
- Add new Controller's Property `maxSpeed` to set explicitly upper bound speed to Autoware ([pull request](https://github.com/tier4/scenario_simulator_v2/pull/554)).

## Version 0.5.3
- Fix setStatus function in EgoEntity class. ([link](https://github.com/tier4/scenario_simulator_v2/pull/549))

## Version 0.5.2
- Remove entity status which is empty. ([link](https://github.com/tier4/scenario_simulator_v2/pull/544)).
- Update syntaxes of `openscenario_interpreter` to be compiled separately ([link](https://github.com/tier4/scenario_simulator_v2/pull/532)).

## Version 0.5.1
- Support OpenSCENARIO 1.1 `AddEntityAction` ([link](https://github.com/tier4/scenario_simulator_v2/pull/506)).
- Add StandStillMetric. [Link](https://github.com/tier4/scenario_simulator_v2/pull/520)
- Add CollisionMetric. [Link](https://github.com/tier4/scenario_simulator_v2/pull/521)
- Support OpenSCENARIO 1.1 `RelativeDistanceCondition` ([link](https://github.com/tier4/scenario_simulator_v2/pull/519)).
- Fixed Action to not cause side effects during `startTransition` ([link](https://github.com/tier4/scenario_simulator_v2/pull/522)).
- Fixed log directory cleaning behavior, cleaning all files and directories under log directory without deleting itself. ([link](https://github.com/tier4/scenario_simulator_v2/pull/527)).
- Support new `CustomCommandAction` type `FaultInjectionAction` for ArchitectureProposal ([link](https://github.com/tier4/scenario_simulator_v2/pull/491))
- Enable run cpp_mock_test with colcon test (with -DWITH_INTEGRATION_TEST=ON) ([link](https://github.com/tier4/scenario_simulator_v2/pull/529))
- Support OpenSCENARIO 1.1 `DistanceCondition` ([link](https://github.com/tier4/scenario_simulator_v2/pull/533)).
- Fix problems in getLongitudinalDistance function when the target entity does not match to the lane. ([link](https://github.com/tier4/scenario_simulator_v2/pull/536)).

## Version 0.5.0
- Add arrow markers to visualize goal poses of entities. (Contribution by [Utaro-M](https://github.com/Utaro-M)).
- Fix problems in setting entity names in proto message. [Link](https://github.com/tier4/scenario_simulator_v2/pull/481) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in never hit line. [Link](https://github.com/tier4/scenario_simulator_v2/pull/480) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Start support getting longitudinal distance to the behind entity in API::getLongitudinalDistance function. [Link](https://github.com/tier4/scenario_simulator_v2/pull/486)
- Fix problems in update/getPhaseDuration function in traffic light class. [Link](https://github.com/tier4/scenario_simulator_v2/pull/492)
- Update dependency for message types of Autoware to reference `AutowareArchitectureProposal_msgs` instead of `AutowareArchitectureProposal.iv`.
- Add to the AutowareArchitectureProposal_api_msgs to the .repos file. [Link](https://github.com/tier4/scenario_simulator_v2/pull/496)
- Fix rotation calculation in toMapPose function. [Link](https://github.com/tier4/scenario_simulator_v2/pull/500)
- Simplify logics in bool API::spawn(const bool is_ego, const std::string & name, const openscenario_msgs::msg::VehicleParameters & params) [Link](https://github.com/tier4/scenario_simulator_v2/pull/486) (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix lane coordinate calculation logic for pedestrian in walk straight action. [Link](https://github.com/tier4/scenario_simulator_v2/pull/507)

## Version 0.4.5
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.5) on GitHub :fa-github:
- Add `move_backward` scenario. [Link](https://github.com/tier4/scenario_simulator_v2/pull/461)
- Fix problems in `CppScenarioNode::start()` function, `onInitialize()` function was called before starting simulation (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in initializing `current_time` value in entity manager class (Contribution by [Robotec.ai](https://robotec.ai/)).
- Supports the option (--architecture-type) to select between AutowareArchitectureProposal and Autoware.Auto (Contribution by [Robotec.ai](https://robotec.ai/)).
- Fix problems in HdMap::toMapPose function, offset distance was calculated from tangent vector. [Link](https://github.com/tier4/scenario_simulator_v2/pull/476)
- Support new options `initialize_duration:=<int>`, `launch_autoware:=<boolean>` and `record:=<boolean>`.

## Version 0.4.4
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.4) on GitHub :fa-github:
- Add a new metrics module which detects vel, acc and jerk out of range (Contribution by [kyabe2718](https://github.com/kyabe2718)).
- Fix phase control feature in traffic light manager class. [Link](https://github.com/tier4/scenario_simulator_v2/pull/450)
- Fix problems in crossing entity on crosswalk. [link](https://github.com/tier4/scenario_simulator_v2/pull/452)

## Version 0.4.3
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.3) on GitHub :fa-github:
- Checking collision between crosswalk and waypoints in lane coordinate.
- Remove division in checking collision function in order to avoid zero-division.
- Enables vehicle entity yield to merging entity. See also [this video](https://user-images.githubusercontent.com/10348912/128287863-8a2db025-d1af-4e54-b5a3-08e5d2e168e4.mp4).
- Simplify the contents of the scenario test result file `result.junit.xml`.

## Version 0.4.2
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.2) on GitHub :fa-github:
- Fix problems in coordinate conversion from world to lane in pedestrian entity.
- Adding `include_crosswalk` option to the HdMapUtils::getClosestLaneletId() and HdMapUtils::toLaneletPose()

## Version 0.4.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.1) on GitHub :fa-github:
- Fix problem in follow front entity action, velocity planner was ignored requested target speed.

## Version 0.4.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.4.0) on GitHub :fa-github:
- Support OpenSCENARIO 1.0 TrafficSignal features (RoadNetwork.TrafficSignalController, Action and Condition).
- Update AcquirePositionAction to support WorldPosition as destination.
- Update syntax 'RoadNetwork.LogicFile' to allow user to specify the directory that contains `lanelet2_map.osm`.
- Check boost::none in TargetSpeedPlanner class.
- Add ROS 2 galactic support.
- Update EgoEntity to publish self-position as PoseWithCovarianceStamped.

## Version 0.3.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.3.0) on GitHub :fa-github:
- Enable spawn MiscObjectEntity by using API class.
- Integrate with AutowareAuto (Autoware type is chosen at build time using `AUTOWARE_ARCHITECTURE_PROPOSAL` or `AUTOWARE_AUTO` flag). (Contribution by [Robotec.ai](https://robotec.ai/)).
- Update WorldPosition to be convertible with `openscenario_msgs::msg::LaneletPose`.
- Fix problems when the whole route is empty in route planner class.
- Support OpenSCENARIO 1.0 MiscObject.

## Version 0.2.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.2.0) on GitHub :fa-github:
- Enhance `/simulation/context` topic information. (adding TriggeringEntitiesRule, TriggeringEntities, CollisionCondition, TimeHeadwayCondition, AccelerationCondition, StandStillCondition, SpeedCondition, ReachPositionCondition, DistanceCondition, RelativeDistanceCondition, ParameterCondition, StoryboardElementStateCondition).
- NPC becomes unable to change lanes behind of them.

## Version 0.1.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.1.1) on GitHub :fa-github:
- Add support for RelativeTargetSpeed, the syntax of OpenSCENARIO.
- Add feature to publish context information during scenario execution to topic `/simulation/context` as a JSON string.
- Enable send warnings semantic error when you call setEntityStatus or setTargetSpeed function which targets to the ego vehicle after starting scenario.

## Version 0.1.0
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.1.0) on GitHub :fa-github:
- Synchronize ROS time between `traffic_simulator` and `sensor_simulator`.
- `openscenario_interpreter` now stores recorded rosbags for each simulation in `output_directory` (argument of `scenario_test_runner`).
- Add AutowareError, SemanticError, SimulationError, SpecificationViolation, SyntaxError and use these errors in `traffic_simulator` and `openscenario_interpreter` package.
- Remove old errors in `traffic_simulator` packages.
- Change Header ID of the lidar/detection sensor. (before : name of the entity -> after : `base_link`)
- Fix problems in publishing detection result. Before this change, the value of the pose was always (x, y, z, roll, pitch, yaw) = (0, 0, 0, 0, 0, 0).
- Remove unused packages (`joy_to_vehicle_cmd`, `scenario_runner_mock`).
- Fix problems when simulator running in 30 FPS and with 10 FPS sensors.
- Enable caching routing result, and center points and it' spline, lanelet length in hdmap_utils class.
- Update interpreter to access TrafficSignals from Action / Condition.
- Update EgoEntity to use precise simulation model parameters.
- Add getVehicleCommand function to the API class.

## Version 0.0.1
- [Release Page](https://github.com/tier4/scenario_simulator_v2/releases/0.0.1) on GitHub :fa-github:
- Partially support OpenSCENARIO 1.0.0 format.
- Support TIER IV Scenario Format v2.
