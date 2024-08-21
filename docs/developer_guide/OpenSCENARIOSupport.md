# OpenSCENARIO Support

The ROS 2 package `openscenario_interpreter` provides scenario-based simulation on [ASAM OpenSCENARIO 1.2](https://www.asam.net/standards/detail/openscenario-xml/).
This section describes the differences between our OpenSCENARIO Interpreter and the OpenSCENARIO standard set by ASAM, and the OpenSCENARIO implementation by other companies and organizations.
If you want to know about OpenSCENARIO, refer to the link below.

- [ASAM OpenSCENARIO: User Guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html)
- [OpenSCENARIO 1.0.0 XSD documentation](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html)

## Dialects and Extensions

### Substitution Syntax

The interpreter supports some substitution syntax of [the ROS 2 Launch system](https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration) (a.k.a. string-interpolation). The substitution syntax works with any attribute string in OpenSCENARIO XML.

Substitution syntaxes can be nested and the substitution is performed from the innermost to the outermost in order.

> [!IMPORTANT]
> This substitution is performed only once during the reading of the attribute, so changes in parameters that occur during the simulation, such as those from `ParameterModifyAction`, do not affect the substitution.

#### Available Syntax

##### `$(find-pkg-prefix <package-name>)`

Substituted with the install prefix path of the given package. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the specified name is not ROS 2 package.

##### `$(var <parameter-name>)`

Substituted with an external representation of the value of the specified OpenSCENARIO parameter. The parameters you specify must be declared by `ParameterDeclarations` before `$ (var ...)` is written.

##### `$(dirname)`

Substituted with the path to the directory where the running scenario script is located.

#### Example

```yml
ParameterDeclarations:
  ParameterDeclaration:
    - name: map_name
      parameterType: string
      value: kashiwanoha
RoadNetwork:
  LogicFile:
    filepath: $(find-pkg-share $(var map_name)_map)/map/lanelet2_map.osm
```

### Scoping

The OpenSCENARIO XML standard [does not define](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#:~:text=If%20a%20reference,lookup%20is%20undefined.) what to do if the name cannot be resolved. In the interpreter, the names of the Element and Parameter are lexically scoped.

- If you refer to an identifier that does not exist, the simulation will stop with an error.
- If multiple identifiers with the same name are defined, the identifier reference is chosen that is closest to the lexical position where the reference occurred.
- Defining a `StoryboardElement` with the same name at the same level is treated as a syntax error (In normal lexical scoping, this should be handled by shadowing, but in scenario languages it is likely a copy-and-paste mistake).

### Command

OpenSCENARIO XML standard states that `CustomCommandAction` can be used to either issue a command to the simulation environment or start an external script. 

For OpenSCENARIO interpreters implemented in scripting languages such as Python, this action is often implemented as a call to an external script file written in the same language as the host language. However, `scenario_simulator_v2` is implemented in C++ and we cannot simply implement such a feature. Therefore, `scenario_simulator_v2` treats the string given in `CustomCommandAction.type` as a command and executes it on a subprocess, as `sh` does.

> [!NOTE]
> To make scenarios portable, the usage of `CustomCommandAction` should be avoided.

#### Built-in Commands

##### `FaultInjectionAction(<EVENT-NAME>, ...)`

Same as `FaultInjectionAction@v1`.

##### `FaultInjectionAction@v1(<EVENT-NAME>, ...)`

Forward any number of event names to Autoware as ERROR level event. Events are forwarded by publishing to the `tier4_simulation_msgs::msg::SimulationEvents` type topic `/simulation/events`. In order to perform fault injection using this `CustomCommandAction`, Autoware must have a node that receives the above message types. Note that `scenario_simulator_v2` has no knowledge of the contents of the event name. In other words, what happens to Autoware by this `CustomCommandAction` depends on Autoware implementation.

##### `FaultInjectionAction@v2(<ERROR-LEVEL>, <EVENT-NAME>)`

Forwards a single event to Autoware with the specified error level. Same as `FaultInjectionAction@v1` except that instead of specifying an error level, only one event can be specified at a time. Available error levels are `OK`, `WARN`, `ERROR` and `STALE`.

##### `PseudoTrafficSignalDetectorConfidenceSetAction@v1(<LANELET-ID>, <CONFIDENCE>)`

Set a confidence value for traffic light topic. This action sets the confidence value to all traffic light bulbs of specified traffic light. If you specify the traffic light by a regulatory element ID, this action sets the confidence value to all traffic lights the regulatory element refers to.

##### `RequestToCooperateCommandAction@v1(<MODULE-NAME>, <COMMAND>)`

Send an `ACTIVATE` / `DEACTIVATE` command to the module publishing a valid request to cooperate. If the send fails, throw an exception to fail the scenario.

##### `V2ITrafficSignalStateAction(<LANELET-ID>, <STATE>, <PUBLISH-RATE(optional)>)`

`TrafficSignalStateAction` for V2I traffic signal. You can optionally specify the publish rate of the traffic signal topic, but otherwise the functionality is the same as `TrafficSignalStateAction`.

##### `WalkStraightAction(<ENTITY-REF>, ...)`

Make **pedestrian** entities walk straight without a target.

##### `exitFailure`

This command immediately terminates the simulation as a failure without transitioning the state of the `StoryboardElement`. See [Termination](#termination) for more details.

##### `exitSuccess`

This command immediately terminates the simulation as a success without transitioning the state of the `StoryboardElement`. See [Termination](#termination) for more details.

##### `:` (do nothing)

Actually `:` is not a built-in command but executed as a shell command, it is worth mentioning here as it achieves 'doing nothing'. `:` is known as the null command in shell scripts, and can be used as a command that does nothing in the simulator as well.

```yml
UserDefinedAction:
  CustomCommandAction:
    type: ':'
```

#### Example

In YAML format, `echo` command can be written as follows:

```yml
UserDefinedAction:
  CustomCommandAction:
    type: 'echo Hello, world!'
```

In XML format, the string given to attribute `type` of `CustomCommandAction` and the string given to its content are concatenated with whitespace and passed to the subprocess so following two examples have the same effect.

``` XML
<UserDefinedAction>
  <CustomCommandAction type="echo">Hello, world"</CustomCommandAction>
</UserDefinedAction>
```

``` XML
<UserDefinedAction>
  <CustomCommandAction type="echo Hello, world!" />
</UserDefinedAction>
```

The effect of calling a command with `CustomCommandAction` is outside the control of the interpreter.
Therefore, if you call a command that has a destructive effect on the system, there is no guarantee that the scenario execution can continue normally.

### Condition

`UserDefinedValueCondition` enables simulators to import external values and compare them with a specific value. The boolean value of the comparison result can be used as a condition to control the scenario.

#### Autoware-related Built-in Conditions

`scenario_simulator_v2` uses `UserDefinedValueCondition` to control the progress of the scenario by Autoware's state.

##### `<ENTITY-REF>.currentState`

Returns Autoware's [state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.

##### `<ENTITY-REF>.currentMinimumRiskManeuverState.behavior`

Returns Autoware's [MRM behavior](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/system/msg/MrmState.msg). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.

##### `<ENTITY-REF>.currentMinimumRiskManeuverState.state`

Returns Autoware's [MRM state](https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/system/msg/MrmState.msg). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.

##### `<ENTITY-REF>.currentEmergencyState`

Returns Autoware's [emergency state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/EmergencyState.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.

##### `<ENTITY-REF>.currentTurnIndicatorsState`

Returns Autoware's [turn indicators state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.

#### Other Built-in Conditions

##### `RelativeHeadingCondition(<ENTITY-REF>)`

Calculates the relative angle between the orientation of `<ENTITY-REF>` and the orientation of the lane on which `<ENTITY-REF>` is positioned.

##### `RelativeHeadingCondition(<ENTITY-REF>, <LANE-ID>, <S>)`

Calculates the relative angle between the orientation of `<ENTITY-REF>` and the direction at the position specified by `<S>` on `<LANE-ID>`.

#### ROS 2 Topic Condition

`scenario_simulator_v2` can read values from another ROS 2 node to a scenario through ROS 2 topics. `name` field should be filled with the name of the ROS 2 topic like below.

```yml
ByValueCondition:
  UserDefinedValueCondition:
    name: /count_up
    rule: equalTo
    value: '500'
```

The type of topic must be `tier4_simulation_msgs::msg::UserDefinedValue` type.
`scenario_simulator_v2` can handle the following through this function.

- Boolean
- DateTime
- Double
- Integer
- String
- UnsignedInt
- UnsignedShort

See [Message Definitions](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_simulation_msgs) for more information.

#### Example

```yml
ByValueCondition:
  UserDefinedValueCondition:
    name: ego.currentState
    rule: equalTo
    value: ARRIVED_GOAL
```

### Termination

`scenario_simulator_v2` is being developed to be integrated into the CI/CD pipeline of autonomous driving systems, and it concludes simulations with a status of either `success`, `failure`, or `error`. `failure` and `error` are each caused by different factors. Specifically, `failure` is used when issues arise from the autonomous driving system itself, such as a vehicle accident occurring during the simulation. On the other hand, `error` is specifically attributed to problems on the simulator's side, such as syntax errors in the scenario file or internal errors within the simulator itself.

Within the scenario, you can end the scenario simulation with a status of either `success` or `failure` by using `exitSuccess` and `exitFailure` from `CustomCommandAction`.

> [!IMPORTANT]
> `exitSuccess` and `exitFailure` terminate the simulation immediately without any state transitions in the lifecycle of a `StoryboardElement`. This means there is no way to detect simulations terminated by `exitSuccess` or `exitFailure` from within the scenario using conditions like `StoryboardElementStateCondition`.

Currently, the only way to know the result of the simulation is by viewing the status string printed to standard output. However, this method may change in the future.

## Standards Supported by `scenario_simulator_v2`

| Name                                         | Supported Version | Detail                                     |
|:---------------------------------------------|:-----------------:|:------------------------------------------:|
| AbsoluteSpeed                                | unimplemented     |                                            |
| AbsoluteTargetLane                           | 1.3               |                                            |
| AbsoluteTargetLaneOffset                     | unimplemented     |                                            |
| AbsoluteTargetSpeed                          | 1.3               |                                            |
| AccelerationCondition                        | 1.3               |                                            |
| AcquirePositionAction                        | 1.3 (partial)     | [detail](#AcquirePositionAction)           |
| Act                                          | 1.3               |                                            |
| Action                                       | 1.3               |                                            |
| ActivateControllerAction                     | unimplemented     |                                            |
| Actors                                       | 1.3 (partial)     |                                            |
| AddEntityAction                              | 1.3               |                                            |
| AngleCondition                               | unimplemented     |                                            |
| AngleType                                    | unimplemented     |                                            |
| AnimationAction                              | unimplemented     |                                            |
| AnimationFile                                | unimplemented     |                                            |
| AnimationState                               | unimplemented     |                                            |
| AnimationType                                | unimplemented     |                                            |
| AppearanceAction                             | unimplemented     |                                            |
| AssignControllerAction                       | 1.2 (partial)     | [detail](#AssignControllerAction)          |
| AssignRouteAction                            | 1.3               |                                            |
| AutomaticGear                                | unimplemented     |                                            |
| AutomaticGearType                            | unimplemented     |                                            |
| Axle                                         | 1.3               |                                            |
| Axles                                        | 1.3               | [detail](#Axles)                           |
| BoundingBox                                  | 1.3               |                                            |
| Brake                                        | unimplemented     |                                            |
| BrakeInput                                   | unimplemented     |                                            |
| ByEntityCondition                            | 1.3               |                                            |
| ByObjectType                                 | unimplemented     |                                            |
| ByType                                       | 1.3               |                                            |
| ByValueCondition                             | 1.3 (partial)     | [detail](#ByValueCondition)                |
| Catalog                                      | 1.3 (partial)     | [detail](#Catalog)                         |
| CatalogDefinition                            | 1.3               |                                            |
| CatalogLocations                             | 1.3               |                                            |
| CatalogReference                             | 1.3               |                                            |
| Center                                       | 1.3               |                                            |
| CentralSwarmObject                           | unimplemented     |                                            |
| Clothoid                                     | unimplemented     |                                            |
| ClothoidSpline                               | unimplemented     |                                            |
| ClothoidSplineSegment                        | unimplemented     |                                            |
| CloudState                                   | unimplemented     |                                            |
| CollisionCondition                           | 1.3 (partial)     | [detail](#CollisionCondition)              |
| Color                                        | unimplemented     |                                            |
| ColorCmyk                                    | unimplemented     |                                            |
| ColorRgb                                     | unimplemented     |                                            |
| ColorType                                    | unimplemented     |                                            |
| ComponentAnimation                           | unimplemented     |                                            |
| Condition                                    | 1.3               |                                            |
| ConditionEdge                                | 1.3 (modified)    | [detail](#ConditionEdge)                   |
| ConditionGroup                               | 1.3               |                                            |
| ConnectTrailerAction                         | unimplemented     |                                            |
| ControlPoint                                 | unimplemented     |                                            |
| Controller                                   | 1.1               | [detail](#Controller)                      |
| ControllerAction                             | 1.0               | [detail](#ControllerAction)                |
| ControllerCatalogLocation                    | 1.3               |                                            |
| ControllerDistribution                       | unimplemented     |                                            |
| ControllerDistributionEntry                  | unimplemented     |                                            |
| ControllerType                               | unimplemented     |                                            |
| CoordinateSystem                             | 1.2 (partial)     | [detail](#CoordinateSystem)                |
| CustomCommandAction                          | 1.3               |                                            |
| CustomContent                                | unimplemented     |                                            |
| DeleteEntityAction                           | 1.3               |                                            |
| Deterministic                                | 1.3               |                                            |
| DeterministicMultiParameterDistribution      | 1.3               |                                            |
| DeterministicMultiParameterDistributionType  | 1.3               |                                            |
| DeterministicParameterDistribution           | 1.3               |                                            |
| DeterministicSingleParameterDistribution     | 1.3               |                                            |
| DeterministicSingleParameterDistributionType | 1.3               |                                            |
| Dimensions                                   | 1.3               |                                            |
| DirectionOfTravelDistribution                | unimplemented     |                                            |
| DirectionalDimension                         | unimplemented     |                                            |
| Directory                                    | 1.3               |                                            |
| DisconnectTrailerAction                      | unimplemented     |                                            |
| DistanceCondition                            | 1.3 (partial)     | [detail](#DistanceCondition)               |
| DistributionDefinition                       | 1.3               |                                            |
| DistributionRange                            | 1.3 (partial)     | [detail](#DistributionRange)               |
| DistributionSet                              | 1.3               |                                            |
| DistributionSetElement                       | 1.3               |                                            |
| DomeImage                                    | 1.3               |                                            |
| DynamicConstraints                           | 1.3               |                                            |
| DynamicsDimension                            | 1.3               |                                            |
| DynamicsShape                                | 1.3 (partial)     | [detail](#DynamicsShape)                   |
| EndOfRoadCondition                           | unimplemented     |                                            |
| Entities                                     | 1.3 (partial)     | [detail](#Entities)                        |
| EntityAction                                 | 1.3               |                                            |
| EntityCondition                              | 1.3 (partial)     | [detail](#EntityCondition)                 |
| EntityDistribution                           | unimplemented     |                                            |
| EntityDistributionEntry                      | unimplemented     |                                            |
| EntityObject                                 | 1.3 (partial)     | [detail](#EntityObject)                    |
| EntityRef                                    | 1.3               |                                            |
| EntitySelection                              | 1.3               |                                            |
| Environment                                  | 1.3               |                                            |
| EnvironmentAction                            | 1.3               |                                            |
| EnvironmentCatalogLocation                   | 1.3               |                                            |
| Event                                        | 1.3               |                                            |
| ExternalObjectReference                      | unimplemented     |                                            |
| File                                         | 1.3               |                                            |
| FileHeader                                   | 1.3               |                                            |
| FinalSpeed                                   | unimplemented     |                                            |
| Fog                                          | 1.3               |                                            |
| FollowTrajectoryAction                       | 1.3               | [detail](#FollowTrajectoryAction)          |
| FollowingMode                                | 1.3               |                                            |
| FractionalCloudCover                         | 1.3               |                                            |
| Gear                                         | unimplemented     |                                            |
| GeoPosition                                  | unimplemented     |                                            |
| GlobalAction                                 | 1.1 (partial)     | [detail](#GlobalAction)                    |
| Histogram                                    | 1.3               |                                            |
| HistogramBin                                 | 1.3               |                                            |
| InRoutePosition                              | unimplemented     |                                            |
| InfrastructureAction                         | 1.3               |                                            |
| Init                                         | 1.3               |                                            |
| InitActions                                  | 1.3               |                                            |
| Knot                                         | unimplemented     |                                            |
| Lane                                         | unimplemented     |                                            |
| LaneChangeAction                             | 1.3 (partial)     | [detail](#LaneChangeAction)                |
| LaneChangeTarget                             | 1.3               |                                            |
| LaneOffsetAction                             | unimplemented     |                                            |
| LaneOffsetActionDynamics                     | unimplemented     |                                            |
| LaneOffsetTarget                             | unimplemented     |                                            |
| LanePosition                                 | 1.3               |                                            |
| LateralAction                                | 1.3 (partial)     | [detail](#LateralAction)                   |
| LateralDisplacement                          | unimplemented     |                                            |
| LateralDistanceAction                        | unimplemented     |                                            |
| License                                      | 1.3               |                                            |
| LightMode                                    | unimplemented     |                                            |
| LightState                                   | unimplemented     |                                            |
| LightStateAction                             | unimplemented     |                                            |
| LightType                                    | unimplemented     |                                            |
| LogNormalDistribution                        | unimplemented     |                                            |
| LongitudinalAction                           | 1.3               | [detail](#LongitudinalAction)              |
| LongitudinalDisplacement                     | unimplemented     |                                            |
| LongitudinalDistanceAction                   | unimplemented     |                                            |
| Maneuver                                     | 1.3               |                                            |
| ManeuverCatalogLocation                      | 1.3               |                                            |
| ManeuverGroup                                | 1.3               |                                            |
| ManualGear                                   | unimplemented     |                                            |
| MiscObject                                   | 1.3               |                                            |
| MiscObjectCatalogLocation                    | 1.3               |                                            |
| MiscObjectCategory                           | 1.3 (partial)     | [detail](#MiscObjectCategory)              |
| ModifyRule                                   | 1.1               | [detail](#ModifyRule)                      |
| MonitorDeclaration                           | unimplemented     |                                            |
| MonitorDeclarations                          | unimplemented     |                                            |
| None                                         | 1.3               |                                            |
| NormalDistribution                           | 1.3               |                                            |
| Nurbs                                        | unimplemented     |                                            |
| ObjectController                             | 1.3               | [detail](#ObjectController)                |
| ObjectType                                   | 1.3               |                                            |
| OffroadCondition                             | unimplemented     |                                            |
| OpenScenario                                 | 1.3               |                                            |
| OpenScenarioCategory                         | 1.3 (modified)    | [detail](#OpenScenarioCategory)            |
| Orientation                                  | 1.3               |                                            |
| OverrideBrakeAction                          | unimplemented     |                                            |
| OverrideClutchAction                         | unimplemented     |                                            |
| OverrideControllerValueAction                | unimplemented     |                                            |
| OverrideGearAction                           | unimplemented     |                                            |
| OverrideParkingBrakeAction                   | unimplemented     |                                            |
| OverrideSteeringWheelAction                  | unimplemented     |                                            |
| OverrideThrottleAction                       | unimplemented     |                                            |
| ParameterAction                              | 1.1               | [detail](#ParameterAction)                 |
| ParameterAddValueRule                        | 1.1               | [detail](#ParameterAddValueRule)           |
| ParameterAssignment                          | 1.3               |                                            |
| ParameterCondition                           | 1.3               |                                            |
| ParameterDeclaration                         | 1.3               |                                            |
| ParameterModifyAction                        | 1.1               | [detail](#ParameterModifyAction)           |
| ParameterMultiplyByValueRule                 | 1.1               | [detail](#ParameterMultiplyByValueRule)    |
| ParameterSetAction                           | 1.1               | [detail](#ParameterSetAction)              |
| ParameterType                                | 1.3               | [detail](#ParameterType)                   |
| ParameterValueDistribution                   | 1.3               |                                            |
| ParameterValueDistributionDefinition         | 1.3               |                                            |
| ParameterValueSet                            | 1.3               |                                            |
| Pedestrian                                   | 1.3 (partial)     | [detail](#Pedestrian)                      |
| PedestrianAnimation                          | unimplemented     |                                            |
| PedestrianCatalogLocation                    | 1.3               |                                            |
| PedestrianCategory                           | 1.3 (partial)     | [detail](#PedestrianCategory)              |
| PedestrianGesture                            | unimplemented     |                                            |
| PedestrianGestureType                        | unimplemented     |                                            |
| PedestrianMotionType                         | unimplemented     |                                            |
| Performance                                  | 1.3               |                                            |
| Phase                                        | 1.1               | [detail](#Phase)                           |
| PoissonDistribution                          | 1.3               |                                            |
| Polygon                                      | unimplemented     |                                            |
| Polyline                                     | 1.3               |                                            |
| Position                                     | 1.3 (partial)     | [detail](#Position)                        |
| PositionInLaneCoordinates                    | unimplemented     |                                            |
| PositionInRoadCoordinates                    | unimplemented     |                                            |
| PositionOfCurrentEntity                      | unimplemented     |                                            |
| Precipitation                                | 1.3               | [detail](#Precipitation)                   |
| PrecipitationType                            | 1.3               |                                            |
| Priority                                     | 1.1               | [detail](#Priority)                        |
| Private                                      | 1.3               |                                            |
| PrivateAction                                | 1.3               | [detail](#PrivateAction)                   |
| ProbabilityDistributionSet                   | 1.3               |                                            |
| ProbabilityDistributionSetElement            | 1.3               |                                            |
| Properties                                   | 1.3 (partial)     | [detail](#Properties)                      |
| Property                                     | 1.3               |                                            |
| RandomRouteAction                            | unimplemented     |                                            |
| Range                                        | 1.3               |                                            |
| ReachPositionCondition                       | 1.1               | [detail](#ReachPositionCondition)          |
| ReferenceContext                             | 1.3 (partial)     | [detail](#ReferenceContext)                |
| RelativeAngleCondition                       | unimplemented     |                                            |
| RelativeClearanceCondition                   | 1.3 (partial)     | [detail](#RelativeClearanceCondition)      |
| RelativeDistanceCondition                    | 1.3 (partial)     | [detail](#RelativeDistanceCondition)       |
| RelativeDistanceType                         | 1.3               | [detail](#RelativeDistanceType)            |
| RelativeLanePosition                         | unimplemented     |                                            |
| RelativeLaneRange                            | 1.3               |                                            |
| RelativeObjectPosition                       | 1.3               |                                            |
| RelativeRoadPosition                         | unimplemented     |                                            |
| RelativeSpeedCondition                       | unimplemented     |                                            |
| RelativeSpeedToMaster                        | unimplemented     |                                            |
| RelativeTargetLane                           | 1.3               |                                            |
| RelativeTargetLaneOffset                     | unimplemented     |                                            |
| RelativeTargetSpeed                          | 1.3               |                                            |
| RelativeWorldPosition                        | 1.3               |                                            |
| RoadCondition                                | 1.3               |                                            |
| RoadCursor                                   | unimplemented     |                                            |
| RoadNetwork                                  | 1.3 (partial)     | [detail](#RoadNetwork)                     |
| RoadPosition                                 | unimplemented     |                                            |
| RoadRange                                    | unimplemented     |                                            |
| Role                                         | unimplemented     |                                            |
| Route                                        | 1.3               |                                            |
| RouteCatalogLocation                         | 1.3               |                                            |
| RoutePosition                                | unimplemented     |                                            |
| RouteRef                                     | unimplemented     |                                            |
| RouteStrategy                                | 1.3 (partial)     | [detail](#RouteStrategy)                   |
| RoutingAction                                | 1.2               | [detail](#RoutingAction)                   |
| RoutingAlgorithm                             | 1.3               |                                            |
| Rule                                         | 1.3               |                                            |
| ScenarioDefinition                           | 1.1               | [detail](#ScenarioDefinition)              |
| ScenarioObject                               | 1.3               |                                            |
| ScenarioObjectTemplate                       | unimplemented     |                                            |
| SelectedEntities                             | 1.3               |                                            |
| SensorReference                              | unimplemented     |                                            |
| SensorReferenceSet                           | unimplemented     |                                            |
| SetMonitorAction                             | unimplemented     |                                            |
| Shape                                        | 1.2               | [detail](#Shape)                           |
| SimulationTimeCondition                      | 1.3               |                                            |
| SpeedAction                                  | 1.3 (partial)     | [detail](#SpeedAction)                     |
| SpeedActionTarget                            | 1.3               |                                            |
| SpeedCondition                               | 1.3 (partial)     | [detail](#SpeedCondition)                  |
| SpeedProfileAction                           | 1.3               |                                            |
| SpeedProfileEntry                            | 1.3               |                                            |
| SpeedTargetValueType                         | 1.3               |                                            |
| StandStillCondition                          | 1.3               |                                            |
| SteadyState                                  | unimplemented     |                                            |
| Stochastic                                   | 1.3               |                                            |
| StochasticDistribution                       | 1.3               |                                            |
| StochasticDistributionType                   | 1.2               | [detail](#StochasticDistributionType)      |
| Story                                        | 1.3               |                                            |
| Storyboard                                   | 1.3               |                                            |
| StoryboardElementState                       | 1.3               |                                            |
| StoryboardElementStateCondition              | 1.3 (modified)    | [detail](#StoryboardElementStateCondition) |
| StoryboardElementType                        | 1.3               |                                            |
| Sun                                          | 1.3               | [detail](#Sun)                             |
| SynchronizeAction                            | unimplemented     |                                            |
| TargetDistanceSteadyState                    | unimplemented     |                                            |
| TargetTimeSteadyState                        | unimplemented     |                                            |
| TeleportAction                               | 1.3               | [detail](#TeleportAction)                  |
| TimeHeadwayCondition                         | 1.0               | [detail](#TimeHeadwayCondition)            |
| TimeOfDay                                    | 1.3               |                                            |
| TimeOfDayCondition                           | unimplemented     |                                            |
| TimeReference                                | 1.3               |                                            |
| TimeToCollisionCondition                     | unimplemented     |                                            |
| TimeToCollisionConditionTarget               | unimplemented     |                                            |
| Timing                                       | 1.3               |                                            |
| TrafficAction                                | unimplemented     |                                            |
| TrafficArea                                  | unimplemented     |                                            |
| TrafficAreaAction                            | unimplemented     |                                            |
| TrafficDefinition                            | unimplemented     |                                            |
| TrafficDistribution                          | unimplemented     |                                            |
| TrafficDistributionEntry                     | unimplemented     |                                            |
| TrafficSignalAction                          | 1.3               |                                            |
| TrafficSignalCondition                       | 1.3               |                                            |
| TrafficSignalController                      | 1.3               |                                            |
| TrafficSignalControllerAction                | 1.3               |                                            |
| TrafficSignalControllerCondition             | 1.3               |                                            |
| TrafficSignalGroupState                      | unimplemented     |                                            |
| TrafficSignalState                           | 1.3               |                                            |
| TrafficSignalStateAction                     | 1.3               |                                            |
| TrafficSinkAction                            | unimplemented     |                                            |
| TrafficSourceAction                          | unimplemented     |                                            |
| TrafficStopAction                            | unimplemented     |                                            |
| TrafficSwarmAction                           | unimplemented     |                                            |
| Trailer                                      | unimplemented     |                                            |
| TrailerAction                                | unimplemented     |                                            |
| TrailerCoupler                               | unimplemented     |                                            |
| TrailerHitch                                 | unimplemented     |                                            |
| Trajectory                                   | 1.3               |                                            |
| TrajectoryCatalogLocation                    | 1.3               |                                            |
| TrajectoryFollowingMode                      | 1.3               |                                            |
| TrajectoryPosition                           | unimplemented     |                                            |
| TrajectoryRef                                | 1.3               |                                            |
| TransitionDynamics                           | 1.1               | [detail](#TransitionDynamics)              |
| TraveledDistanceCondition                    | unimplemented     |                                            |
| Trigger                                      | 1.3               |                                            |
| TriggeringEntities                           | 1.3               |                                            |
| TriggeringEntitiesRule                       | 1.3 (modified)    | [detail](#TriggeringEntitiesRule)          |
| UniformDistribution                          | 1.3               |                                            |
| UsedArea                                     | unimplemented     |                                            |
| UserDefinedAction                            | 1.3               |                                            |
| UserDefinedAnimation                         | unimplemented     |                                            |
| UserDefinedComponent                         | unimplemented     |                                            |
| UserDefinedDistribution                      | unimplemented     |                                            |
| UserDefinedLight                             | unimplemented     |                                            |
| UserDefinedValueCondition                    | 1.3               |                                            |
| ValueConstraint                              | 1.3               |                                            |
| ValueConstraintGroup                         | 1.3               |                                            |
| ValueSetDistribution                         | 1.3               |                                            |
| VariableAction                               | unimplemented     |                                            |
| VariableAddValueRule                         | unimplemented     |                                            |
| VariableCondition                            | unimplemented     |                                            |
| VariableDeclaration                          | unimplemented     |                                            |
| VariableDeclarations                         | unimplemented     |                                            |
| VariableModifyAction                         | unimplemented     |                                            |
| VariableModifyRule                           | unimplemented     |                                            |
| VariableMultiplyByValueRule                  | unimplemented     |                                            |
| VariableSetAction                            | unimplemented     |                                            |
| Vehicle                                      | 1.1 (partial)     | [detail](#Vehicle)                         |
| VehicleCatalogLocation                       | 1.3               |                                            |
| VehicleCategory                              | 1.3               |                                            |
| VehicleCategoryDistribution                  | unimplemented     |                                            |
| VehicleCategoryDistributionEntry             | unimplemented     |                                            |
| VehicleComponent                             | unimplemented     |                                            |
| VehicleComponentType                         | unimplemented     |                                            |
| VehicleLight                                 | unimplemented     |                                            |
| VehicleLightType                             | unimplemented     |                                            |
| VehicleRoleDistribution                      | unimplemented     |                                            |
| VehicleRoleDistributionEntry                 | unimplemented     |                                            |
| Vertex                                       | 1.3               |                                            |
| VisibilityAction                             | unimplemented     |                                            |
| Waypoint                                     | 1.3               |                                            |
| Weather                                      | 1.3               |                                            |
| Wetness                                      | 1.3               |                                            |
| Wind                                         | 1.3               |                                            |
| WorldPosition                                | 1.3               |                                            |

### Details

#### AcquirePositionAction

- Property `Position` of types `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.

#### AssignControllerAction

- Properties `activateAnimation`, `activateLateral`, `activateLighting`, and `activateLongitudinal` are ignored.
  - The simulator behaves as if these properties are `false`.
- Property `ObjectController` created in version 1.3 is **not** supported.
- Properties `Controller` and `CatalogReference` deprecated in version 1.3 are supported.

#### Axles

- Property `RearAxle` is made mandatory in version 1.3, but still can be omitted like specified in version 1.2.

#### ByValueCondition

- Properties `TimeOfDayCondition` and `VariableCondition` are **not** supported.

#### Catalog

- Property `Trajectory` is ignored.

#### CollisionCondition

- Property `ByType` is **not** supported.

#### ConditionEdge

- Enumeration literal `sticky` is added as TIER IV extension.

#### Controller

- Property `controllerType` created in version 1.2 is ignored.

#### ControllerAction

- Property `OverrideControllerValueAction` is ignored.
- Property `ActivateControllerAction` created in version 1.1 is ignored.

#### CoordinateSystem

- Enumeration literals `road` and `trajectory` are **not** supported.
- Enumeration literal `world` created in version 1.3 is **not** supported.

#### DistanceCondition

- Property `alongRoute` deprecated in version 1.1 is **not** supported.
- Property `Position` of types `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.
- Not all combinations of properties for distance calculation are supported. Supported combinations are listed below:
  | coordinateSystem | relativeDistanceType | routingAlgorithm | freespace |
  |:----------------:|:--------------------:|:----------------:|:---------:|
  | entity           | euclidianDistance    | undefined        | false     |
  | entity           | euclidianDistance    | undefined        | true      |
  | entity           | lateral              | undefined        | false     |
  | entity           | lateral              | undefined        | true      |
  | entity           | longitudinal         | undefined        | false     |
  | entity           | longitudinal         | undefined        | true      |
  | lane             | lateral              | undefined        | false     |
  | lane             | lateral              | undefined        | true      |
  | lane             | longitudinal         | undefined        | false     |
  | lane             | longitudinal         | undefined        | true      |
  | lane             | lateral              | shortest         | false     |
  | lane             | lateral              | shortest         | true      |
  | lane             | longitudinal         | shortest         | false     |
  | lane             | longitudinal         | shortest         | true      |

#### DistributionRange

- Property `stepWidth` is ignored.

#### DynamicsShape

- Enumeration literal `sinusoidal` is **not** supported.

#### Entities

- Property `EntitySelection` is **not** supported.

#### EntityCondition

- Properties `EndOfRoadCondition`, `OffroadCondition`, `TimeToCollisionCondition`, `RelativeDistanceCondition`, `TraveledDistanceCondition`, `AngleCondition`, and `RelativeAngleCondition` are **not** supported.
- Property `ReachPositionCondition` deprecated in version 1.2 is still supported.

#### EntityObject

- Property `ExternalObjectReference` is **not** supported.

#### FollowTrajectoryAction

- Properties `Trajectory` and `CatalogReference` deprecated in version 1.1 are ignored.
- Property `TrajectoryRef` of type `Clothoid` and `Nurbs` are **not** supported.

#### GlobalAction

- Properties `TrafficAction` and `VariableAction` are not supported.
- Property `ParameterAction` deprecated in version 1.2 is still supported.
- Property `SetMonitorAction` created in version 1.3 is **not** supported.

#### LaneChangeAction

- Specifying `step` for `LaneChangeActionDynamics.dynamicsDimension` is **not** supported.
  - Simulator may lead to an undefined behavior if `step` is specified.

#### LateralAction

- Properties `LaneOffsetAction` and `LateralDistanceAction` are **not** supported.

#### LongitudinalAction

- Property `LongitudinalDistanceAction` is **not** supported.

#### MiscObjectCategory

- Enumeration literals `barrier`, `building`, `crosswalk`, `gantry`, `none`, `parkingSpace`, `patch`, `pole`, `roadMark`, `soundBarrier`, `streetLamp`, `trafficIsland`, `tree`, and `vegetation` are **not** supported.
- Enumeration literal `wind` deprecated in version 1.1 is **not** supported.

#### ModifyRule

- Class `ModifyRule` deprecated in version 1.3 is still supported.

#### ObjectController

- Property `name` is ignored.

#### OpenScenarioCategory

- The simulator reads `Storyboard` in XML as property `ScenarioDefinition`
- The simulator reads `Catalog` in XML as property `CatalogDefinition`
- The simulator reads `ParameterValueDistribution` in XML as property `ParameterValueDistributionDefinition`

#### ParameterAction

- Class `ParameterAction` deprecated in version 1.2 is still supported.

#### ParameterAddValueRule

- Class `ParameterAddValueRule` deprecated in version 1.2 is still supported.

#### ParameterModifyAction

- Class `ParameterModifyAction` deprecated in version 1.2 is still supported.

#### ParameterMultiplyByValueRule

- Class `ParameterMultiplyByValueRule` deprecated in version 1.2 is still supported.

#### ParameterSetAction

- Class `ParameterSetAction` deprecated in version 1.2 is still supported.
- Class `ParameterSetAction` cannot handle parameters of type `DataTime`.

#### ParameterType

- Enumeration literal `integer` deprecated in version 1.2 is **not** supported.

#### Pedestrian

- Property `role` is ignored.
  - The simulator does not take into account `role`.
- Property `model` deprecated version 1.1 is ignored but mandatory.
  - Maybe this is simulator bug and need to be fixed.

#### PedestrianCategory

- Enumeration literals `wheelchair` and `animal` are **not** supported.

#### Phase

- Property `TrafficSignalGroupState` created in version 1.2 is ignored.

#### Position

- Properties `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.

#### Precipitation

- Property `intensity` deprecated in version 1.1 is still supported.

#### Priority

- Enumeration literal `override` created in version 1.2 is **not** supported.
- Enumeration literal `overwrite` deprecated in version 1.2 is supported.

#### PrivateAction

- Property `VisibilityAction`, `SynchronizeAction` and `ActivateControllerAction` are **not** supported.

#### Properties

- Property `CustomContent` is ignored.
  - The simulator does not take into account `CustomContent`.

#### ReachPositionCondition

- Class `ReachPositionCondition` deprecated in version 1.2 is still supported.
- Property `Position` of types `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.

#### ReferenceContext

- Enumeration literal `absolute` is **not** supported.

#### RelativeClearanceCondition

- Property `oppositeLanes` is ignored.
  - The simulator behaves as if `oppositeLanes` is `false`.

#### RelativeDistanceCondition

- Property `Position` of types `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.
- Not all combinations of properties for distance calculation are supported. Supported combinations are listed below:
  | coordinateSystem | relativeDistanceType | routingAlgorithm | freespace |
  |:----------------:|:--------------------:|:----------------:|:---------:|
  | entity           | euclidianDistance    | undefined        | false     |
  | entity           | euclidianDistance    | undefined        | true      |
  | entity           | lateral              | undefined        | false     |
  | entity           | lateral              | undefined        | true      |
  | entity           | longitudinal         | undefined        | false     |
  | entity           | longitudinal         | undefined        | true      |
  | lane             | lateral              | undefined        | false     |
  | lane             | lateral              | undefined        | true      |
  | lane             | longitudinal         | undefined        | false     |
  | lane             | longitudinal         | undefined        | true      |
  | lane             | lateral              | shortest         | false     |
  | lane             | lateral              | shortest         | true      |
  | lane             | longitudinal         | shortest         | false     |
  | lane             | longitudinal         | shortest         | true      |

#### RelativeDistanceType

- Enumeration literal `cartesianDistance` deprecated in version 1.1 is **not** supported.

#### RoadNetwork

- Property `UsedArea` is ignored.

#### RouteStrategy

- Enumeration literals `fastest`, `leastIntersections` and `random` are **not** supported.

#### RoutingAction

- Property `RandomRouteAction` created in version 1.3 is **not** supported.

#### ScenarioDefinition

- Property `VariableDeclarations` created in version 1.2 is ignored.
- Property `MonitorDeclarations` created in version 1.3 is ignored.

#### Shape

- Properties `Clothoid` and `Nurbs` are **not** supported.
- Property `ClothoidSpline` created in version 1.3 is **not** supported.

#### SpeedAction

- Specifying `time` or `distance` for `SpeedActionDynamics.dynamicsDimension` is **not** supported.
  - Simulator may lead to an undefined behavior if `time` or `distance` is specified.
- Specifying `cubic` for `SpeedActionDynamics.dynamicsShape` is **not** supported.
  - Simulator may lead to an undefined behavior if `cubic` is specified.

#### SpeedCondition

- Property `direction` is ignored.
  - The simulator behaves as if `direction` is not given.

#### StochasticDistributionType

- Property `LogNormalDistribution` created in version 1.3 is **not** supported.

#### StoryboardElementStateCondition

- [name prefix](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#:~:text=A%20name%20prefix%20,new%20name%20reference.) in OpenSCENARIO User Guide 3.1.2. is **not** supported.
  - The interpreter uses lexical scope instead. See [Scoping](#Scoping) for more details.

#### Sun

- Property `intensity` deprecated in version 1.2 is still supported.

#### TeleportAction

- Property `Position` of types `RoadPosition`, `RelativeRoadPosition`, `RelativeLanePosition`, `RoutePosition`, `GeoPosition`, and `TrajectoryPosition` are **not** supported.

#### TimeHeadwayCondition

- Properties `coordinateSystem` and `relativeDistanceType` created in version 1.1 is ignored.
- Property `alongRoute` deprecated in version 1.1 is ignored.
- Property `freespace` is ignored.
  - The simulator behaves as if `freespace` is `false`.

#### TransitionDynamics

- Property `followingMode` created in version 1.2 is ignored.

#### TriggeringEntitiesRule

- Enumeration literal `none` is added as TIER IV extension.

#### Vehicle

- Property `role` created in version 1.1 is ignored.
  - The simulator does not take into account `role`.
- Property `mass` is ignored.
  - The simulator behaves as if `mass` is not given.
