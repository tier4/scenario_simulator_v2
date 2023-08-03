# OpenSCENARIO Support

The ROS 2 package `openscenario_interpreter` provides scenario-based simulation on [ASAM OpenSCENARIO 1.0](https://www.asam.net/standards/detail/openscenario/).
This section describes the differences between our OpenSCENARIO Interpreter and the OpenSCENARIO standard set by ASAM, and the OpenSCENARIO implementation by other companies and organizations.
If you want to know about OpenSCENARIO, refer to the link below.

- [ASAM OpenSCENARIO: User Guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html)
- [OpenSCENARIO 1.0.0 XSD documentation](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html)

## Specific Features
---

### ROS 2 Launch-like substitution syntax

Our interpreter supports some substitution syntax of [the ROS 2 Launch system](https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration) (a.k.a. string-interpolation).
The substitution syntax works with any attribute string in OpenSCENARIO XML.

This substitution is done only once when reading the attribute.
Note that the substitution result is finalized before the simulation starts, so it is not affected by the `ParameterModifyAction`, etc. that takes effect during the simulation.

The supported functions and their behavior are shown below.

#### `$(find-pkg-prefix <package-name>)`

Equivalent to the following description given in ROS 2 Design (unless we made a mistake in our implementation).

> Substituted by the install prefix path of the given package. Forward and backwards slashes will be resolved to the local filesystem convention. Substitution will fail if the package cannot be found.

The package specified must be a ROS 2 package.

#### `$(var <parameter-name>)`

Replaces with an external representation of the value of the specified
OpenSCENARIO parameter.
The parameters you specify must be declared by ParameterDeclarations before
`$ (var ...)` is written.

#### `$(dirname)`

Substitute the path to the directory where the running scenario script is located.

#### Evaluation of nested substitution syntax

These substitution syntaxes can be nested in any rank.
The replacement is done from the inside to the outside of the nest.
An example is shown below.

``` XML
  <ParameterDeclarations>
    <ParameterDeclaration name="map-name" parameterType="string" value="kashiwanoha"/>
  </ParameterDeclarations>
  <CatalogLocations/>
  <RoadNetwork>
    <LogicFile filepath="$(find-pkg-share $(var map-name)_map)/map/lanelet2_map.osm"/>
  </RoadNetwork>
```

## Implementation Dependent Behaviors
---

OpenSCENARIO has the function that the detailed behavior is left to the decision of the implementation side, which is defined as "subject of a contract between simulation environment provider and scenario author".

### Scoping

The OpenSCENARIO standard does not define what to do if the name cannot be resolved, as quoted below.

> If a reference cannot be resolved uniquely, for example if too few name prefixes have been specified to disambiguate fully, the result of the lookup is undefined.

In our interpreter, the names of the Element and Parameter are lexically scoped.

- If you refer to an identifier that does not exist, the simulation will stop with an error.
- If multiple identifiers with the same name are defined, the identifier reference is chosen that is closest to the lexical position where the reference occurred.
- Defining a StoryboardElement with the same name at the same level is treated as a syntax error (In normal lexical scoping, this should be handled by shadowing, but in scenario languages it is likely a copy-and-paste mistake).

### CustomCommandAction

This Action is specified in the standard as follows.
> Used to either issue a command to the simulation environment or start an external script. Allows the user to activate custom actions in their simulation tool.

For OpenSCENARIO interpreters implemented in scripting languages such as Python, this Action is often implemented as a call to an external script file written in the same language as the host language.
However, our interpreter is implemented in C++ and we cannot simply implement such a feature.
Therefore, our interpreter treats the string given in CustomCommandAction.type as a command and executes it on a subprocess, as `sh` does.

For example, the `echo` command can be written as follows:
``` XML
  <UserDefinedAction>
    <CustomCommandAction type="echo">Hello, world!</CustomCommandAction>
  </UserDefinedAction>
```

The string given to attribute `type` of CustomCommandAction and the string given to its content are concatenated with whitespace and passed to the subprocess.
Therefore, the following two cases have the same effect.

``` XML
  <UserDefinedAction>
    <CustomCommandAction type="echo">Hello, world"</CustomCommandAction>
  </UserDefinedAction>
```

``` XML
  <UserDefinedAction>
    <CustomCommandAction type="echo Hello, world!"/>
  </UserDefinedAction>
```

The effect of calling a command with `CustomCommandAction` is outside the control of the interpreter.
Therefore, if you call a command that has a destructive effect on the system, there is no guarantee that the scenario execution can continue normally.

**For portable scenarios, the use of CustomCommandAction should be avoided as much as possible or limited to the scope of POSIX.**

#### NullAction

In particular, the following usages that achieve "do nothing action" are worth special mention.
Here, the colon (`:`) specified in the `CustomCommandAction.type` is the `sh` command is known by the name of the null-command.

``` XML
  <UserDefinedAction>
    <CustomCommandAction type=":"/>
  </UserDefinedAction>
```

#### Built-in commands

| Syntax                                                                                                                                                                                                | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: FaultInjectionAction(<EVENT-NAME\>, ...)</pre>                                                                                       | Same as FaultInjectionAction@v1.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: FaultInjectionAction@v1(<EVENT-NAME\>, ...)</pre>                                                                                    | Forward any number of event names to Autoware as `ERROR` level event. Events are forwarded by publishing to the `tier4_simulation_msgs::msg::SimulationEvents` type topic `/simulation/events`. In order to perform fault injection using this CustomCommandAction, Autoware must have a node that receives the above message types. Note that the simulator has no knowledge of the contents of the event name. In other words, what happens to Autoware by this CustomCommandAction depends on Autoware implementation. |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: FaultInjectionAction@v2(<ERROR-LEVEL\>, <EVENT-NAME\>)</pre>                                                                         | Forwards a single event to Autoware with the specified error level. Same as `FaultInjectionAction@v1` except that instead of specifying an error level, only one event can be specified at a time. Available error levels are `OK`, `WARN`, `ERROR` and `STALE`.                                                                                                                                                                                                                                                          |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: RequestToCooperateCommandAction@v1(<MODULE-NAME\>, <COMMAND\>)</pre>                                                                 | Send an `ACTIVATE` / `DEACTIVATE` command to the module publishing a valid request to cooperate. If the send fails, throw an exception to fail the scenario.                                                                                                                                                                                                                                                                                                                                                              |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: V2ITrafficSignalStateAction(<LANELET-ID\>, <STATE\>,    \<br/>                                      <PUBLISH-RATE(optional)\>)</pre> | TrafficSignalStateAction for V2I traffic signal. You can optionally specify the publish rate of the traffic signal topic, but otherwise the functionality is the same as `TrafficSignalStateAction`.                                                                                                                                                                                                                                                                                                                      |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: WalkStraightAction(<ENTITY-REF\>, ...)</pre>                                                                                         | Make **pedestrian** entities walk straight without a target.                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: exitFailure</pre>                                                                                                                    | Immediately terminates the simulation as a failure.                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| <pre>UserDefinedAction:<br/>  CustomCommandAction:<br/>    type: exitSuccess</pre>                                                                                                                    | Immediately terminates the simulation as successful.                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

The termination ignores the StoryboardElement's lifecycle transition (that is, it means that `StoryboardElementStateCondition` cannot be used to prevent or detect the execution of this command).

The terminated scenario determines the final success / failure / error.

**Currently, simulation results are notified by simply writing to standard output as text. This notification method is temporary and will change in the near future.**

The following built-in commands are debug commands for interpreter developers, not scenario creators.
It is unlikely that you will need these commands for normal scenario creation.

| Name    | Effect                                                                                            |
|:--------|:--------------------------------------------------------------------------------------------------|
| error   | Generate internal error: Used to ensure that the simulator does not crash with an internal error. |
| sigsegv | Access a null pointer: Used to make sure the simulator does not crash with an internal error.     |

### UserDefinedValueCondition

This condition enables us to import external values and compare them with a specific value.
The boolean value of  the comparison result can be used as a condition to control the scenario.
In scenario_simulator_v2, we use `UserDefinedValueCondition` to control the progress of the scenario by Autoware's state.

```XML
  <ByValueCondition>
     <UserDefinedValueCondition name="ego.currentState" rule="equalTo" value="ARRIVED_GOAL" />
  </ByValueCondition>
```
#### Built-in conditions

Like "currentState", the conditions start with "current" return Autoware-related conditions.
And like "ego.currentState", they can specify the entity reference by prepending the name of the entity

The following built-in conditions return a string that represents the state.
See Reference for specific strings.

| Name                       | Syntax                                                                                                   | Description                                                                                                                                                                                                                                                        |
|----------------------------|----------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| currentState               | `<ENTITY-REF>.currentState`                                                                              | Returns Autoware's [state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.                          |
| currentEmergencyState      | `<ENTITY-REF>.currentEmergencyState`                                                                     | Returns Autoware's [emergency state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/EmergencyState.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true.               |
| currentTurnIndicatorsState | `<ENTITY-REF>.currentTurnIndicatorsState`                                                                | Returns Autoware's [turn indicators state](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand.idl). `<ENTITY-REF>` must be the name of Vehicle with ObjectController's property `isEgo` set to true. |
| RelativeHeadingCondition   | `RelativeHeadingCondition(<ENTITY-REF>)` <br> `RelativeHeadingCondition(<ENTITY-REF>, <LANE-ID>, <S>)`   | Returns the relative angle to the lane heading.                                                                                                                                                                                                                    |

#### External ROS 2 topic condition

You can pass values from another ROS 2 node to a scenario through ROS 2 topics.
The `name` field should be filled with the name of the ROS 2 topic like below.
```XML
  <ByValueCondition>
     <UserDefinedValueCondition name="/count_up" rule="equalTo" value="500" />
  </ByValueCondition>
```

The type of topic must be `tier4_simulation_msgs::msg::UserDefinedValue` type.
You can handle the following through this function.

- Boolean
- DateTime
- Double
- Integer
- String
- UnsignedInt
- UnsignedShort

See [Message Definitions](https://github.com/tier4/tier4_autoware_msgs/tree/tier4/universe/tier4_simulation_msgs) for more information.

## Non-Standard Extensions
---

### Success/failure judgment

Our interpreters have been developed with the intention of incorporating them into the CI / CD pipelines in autonomous driving systems.
Therefore, executing a scenario can result in success, failure, or error.
Note that "error" means a flaw in the scenario, such as a syntax error, or an internal error, not an "error in an automated driving system" such as "a vehicle accident occurred in a simulation".
In such cases, you will be notified of a "failure".

## Supporting Status
---

Our OpenSCENARIO Interpreter does not currently support the full range of
OpenSCENARIO standards.

### Actions

| Name                                                                                    | Support Status | Limitations                        | OpenSCENARIO changes |
|:----------------------------------------------------------------------------------------|:--------------:|:-----------------------------------|----------------------|
| GlobalAction.**EnvironmentAction**                                                      |  Unsupported   |                                    |                      |
| GlobalAction.EntityAction.**AddEntityAction**                                           |       ✔        |                                    |                      |
| GlobalAction.EntityAction.**DeleteEntityAction**                                        |       ✔        |                                    |                      |
| GlobalAction.ParameterAction.**ParameterSetAction**                                     |       ✔        | See [here](#parametersetaction)    | deprecated from v1.2 |
| GlobalAction.ParameterAction.**ParameterModifyAction**                                  |       ✔        | No                                 | deprecated from v1.2 |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalControllerAction** |       ✔        |                                    |                      |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalStateAction**      |       ✔        |                                    |                      |
| GlobalAction.TrafficAction.**TrafficSourceAction**                                      |  Unsupported   |                                    |                      |
| GlobalAction.TrafficAction.**TrafficSinkAction**                                        |  Unsupported   |                                    |                      |
| GlobalAction.TrafficAction.**TrafficSwarmAction**                                       |  Unsupported   |                                    |                      |
| GlobalAction.TrafficAction.**TrafficStopAction**                                        |  Unsupported   |                                    |                      |
| GlobalAction.VariableAction.**VariableSetAction**                                       |  Unsupported   |                                    | created in v1.2      |
| GlobalAction.VariableAction.**VariableModifyAction**                                    |  Unsupported   |                                    | created in v1.2      |
| UserDefinedAction.**CustomCommandAction**                                               |       ✔        | See [here](#customcommandaction)   |                      |
| PrivateAction.AppearanceAction.**AnimationAction**                                      |  Unsupported   |                                    | created in v1.2      |
| PrivateAction.AppearanceAction.**LightStateAction**                                     |  Unsupported   |                                    | created in v1.2      |
| PrivateAction.LongitudinalAction.**SpeedAction**                                        |       ✔        | See [here](#speedaction)           |                      |
| PrivateAction.LongitudinalAction.**SpeedProfileAction**                                 |  Unsupported   |                                    | created in v1.2      |
| PrivateAction.LongitudinalAction.**LongitudinalDistanceAction**                         |  Unsupported   |                                    |                      |
| PrivateAction.LateralAction.**LaneChangeAction**                                        |       ✔        | See [here](#lanechangeaction)      |                      |
| PrivateAction.LateralAction.**LaneOffsetAction**                                        |  Unsupported   |                                    |                      |
| PrivateAction.LateralAction.**LateralDistanceAction**                                   |  Unsupported   |                                    |                      |
| PrivateAction.**VisibilityAction**                                                      |  Unsupported   |                                    |                      |
| PrivateAction.**SynchronizeAction**                                                     |  Unsupported   |                                    |                      |
| PrivateAction.**ActivateControllerAction**                                              |  Unsupported   |                                    | deprecated from v1.2 |
| PrivateAction.ControllerAction.**AssignControllerAction**                               |       ✔        |                                    |                      |
| PrivateAction.ControllerAction.**ActivateControllerAction**                             |  Unsupported   |                                    |                      |
| PrivateAction.ControllerAction.**OverrideControllerValueAction**                        |  Unsupported   |                                    |                      |
| PrivateAction.**TeleportAction**                                                        |       ✔        | See [here](#teleportaction)        |                      |
| PrivateAction.RoutingAction.**AssignRouteAction**                                       |       ✔        |                                    |                      |
| PrivateAction.RoutingAction.**FollowTrajectoryAction**                                  |  Unsupported   |                                    |                      |
| PrivateAction.RoutingAction.**AcquirePositionAction**                                   |       ✔        | See [here](#acquirepositionaction) |                      |

### Conditions

| Name                                                            | Support Status | Limitations                                  | OpenSCENARIO changes |
|:----------------------------------------------------------------|:--------------:|:---------------------------------------------|----------------------|
| ByEntityCondition.EntityCondition.**EndOfRoadCondition**        |  Unsupported   |                                              |                      |
| ByEntityCondition.EntityCondition.**CollisionCondition**        |       ✔        | See [here](#collisioncondition)              |                      |
| ByEntityCondition.EntityCondition.**OffroadCondition**          |  Unsupported   |                                              |                      |
| ByEntityCondition.EntityCondition.**TimeHeadwayCondition**      |       ✔        | See [here](#timeheadwaycondition)            |                      |
| ByEntityCondition.EntityCondition.**TimeToCollisionCondition**  |  Unsupported   |                                              |                      |
| ByEntityCondition.EntityCondition.**AccelerationCondition**     |       ✔        | No                                           |                      |
| ByEntityCondition.EntityCondition.**StandStillCondition**       |       ✔        | No                                           |                      |
| ByEntityCondition.EntityCondition.**SpeedCondition**            |       ✔        | No                                           |                      |
| ByEntityCondition.EntityCondition.**RelativeSpeedCondition**    |  Unsupported   |                                              |                      |
| ByEntityCondition.EntityCondition.**TraveledDistanceCondition** |  Unsupported   |                                              |                      |
| ByEntityCondition.EntityCondition.**ReachPositionCondition**    |       ✔        | See [here](#reachpositioncondition)          | deprecated from v1.2 |
| ByEntityCondition.EntityCondition.**DistanceCondition**         |       ✔        | See [here](#distancecondition)               |                      |
| ByEntityCondition.EntityCondition.**RelativeDistanceCondition** |       ✔        | See [here](#relativedistancecondition)       |                      |
| ByValueCondition.**ParameterCondition**                         |       ✔        |                                              |                      |
| ByValueCondition.**TimeOfDayCondition**                         |  Unsupported   |                                              |                      |
| ByValueCondition.**SimulationTimeCondition**                    |       ✔        | No                                           |                      |
| ByValueCondition.**StoryboardElementStateCondition**            |       ✔        | See [here](#storyboardelementstatecondition) |                      |
| ByValueCondition.**UserDefinedValueCondition**                  |       ✔        |                                              |                      |
| ByValueCondition.**TrafficSignalCondition**                     |       ✔        |                                              |                      |
| ByValueCondition.**TrafficSignalControllerCondition**           |       ✔        |                                              |                      |
| ByValueCondition.**VariableCondition**                          |  Unsupported   |                                              | created in v1.2      |

## Limitations

### ParameterSetAction

- Currently, ParameterSetAction cannot handle `dateTime` type parameters.

### CustomCommandAction
#### WalkStraightAction
- This action is a temporary feature until `FollowTrajectoryAction` is implemented.
- This action cannot be used in combination with `AcquirePositionAction` because `WalkStraightAction` just makes a pedestrian NPC go straight without a destination.


### SpeedAction

- The implementation of type [TransitionDynamics](#transitiondynamics) for element `SpeedActionDynamics` is incomplete and
  **SpeedActionDynamics.dynamicsDimension is ignored**.

### LaneChangeAction

- The implementation of type [TransitionDynamics](#transitiondynamics) for element `LaneChangeActionDynamics` and type [LaneChangeTarget](#lanechangetarget) for element `LaneChangeTarget` are incomplete.

### TeleportAction

- Currently, **only LanePosition** can be specified for element of
  TeleportAction.

### AcquirePositionAction

- Currently, **only LanePosition** can be specified for element of
  AcquirePositionAction.

### CollisionCondition

- Currently, **only EntityRef** can be specified for element of
  CollisionCondition.

### TimeHeadwayCondition

- Currently, the values of attribute "freespace" and "alongRoute" are ignored and always behave as if freespace="false" and alongRoute="true" were specified.

### ReachPositionCondition

- Currently, **only LanePosition and WorldPosition** can be specified for the element of ReachPositionCondition.

### DistanceCondition

- Currently, the values of attribute "freespace" and "alongRoute" are ignored and always behave as if freespace="false" and alongRoute="false" were specified.
- Currently, **only LanePosition and WorldPosition** can be specified for the element of Position of DistanceCondition.

### RelativeDistanceCondition

- Currently, the values of attribute "freespace" is ignored and always behave as if freespace="false" was specified.

### StoryboardElementStateCondition

- Currently, a feature called "name prefix" (in [OpenSCENARIO User Guide 3.1.2. Naming](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_general_concepts)) is unsupported.

Instead, our interpreter implements lexical scoping.
See also section [Scoping](#scoping).

### TransitionDynamics

- The implementation of type [DynamicsShape](#dynamicsshape) for attribute dynamicsShape is incomplete.

| Name              | Type                            |   Status   |
|:------------------|:--------------------------------|:----------:|
| dynamicsShape     | [DynamicsShape](#dynamicsshape) | Incomplete |
| value             | Double                          |     ✔      |
| dynamicsDimension | DynamicsDimension               |     ✔      |

### DynamicsShape

- Currently, only `linear` and `step` are implemented for values of this enumeration.
  If you specify `cubic` and `sinusoidal`, you will get an ImplementationFault.

| Value      |   Status    |
|:-----------|:-----------:|
| linear     |      ✔      |
| cubic      | Unsupported |
| sinusoidal | Unsupported |
| step       |      ✔      |

### LaneChangeTarget

- Currently, only AbsoluteTargetLane is implemented for element of this type.
  If you specify RelativeTargetLane, you will get a SyntaxError.

| Element            |   Status    |
|:-------------------|:-----------:|
| AbsoluteTargetLane |      ✔      |
| RelativeTargetLane | Unsupported |

### Position

- Currently, only WorldPosition and LanePosition are implemented for an element of this type.

| Element                |   Status    |
|:-----------------------|:-----------:|
| WorldPosition          |      ✔      |
| RelativeWorldPosition  | Unsupported |
| RelativeObjectPosition | Unsupported |
| RoadPosition           | Unsupported |
| RelativeRoadPosition   | Unsupported |
| LanePosition           |      ✔      |
| RelativeLanePosition   | Unsupported |
| RoutePosition          | Unsupported |
