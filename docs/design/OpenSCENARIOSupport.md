# OpenSCENARIO Support

The ROS2 package `openscenario_interpreter` provides scenario-based simulation on [ASAM OpenSCENARIO 1.0](https://www.asam.net/standards/detail/openscenario/).
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

| Name        | Effect                                               |
|:------------|:-----------------------------------------------------|
| exitSuccess | Immediately terminates the simulation as successful. |
| exitFailure | Immediately terminates the simulation as a failure.  |

These built-in commands force the simulation to terminate.
This termination ignores the StoryboardElement's lifecycle transition (that is, it means that `StoryboardElementStateCondition` cannot be used to prevent or detect the execution of this command).

The terminated scenario determines the final success / failure / error by collating the called command with the expected simulation result specified in `expect` of the [workflow file](../user_guide/scenario_test_runner/HowToWriteWorkflowFile.md).

**Currently, simulation results are notified by simply writing to standard output as text. This notification method is temporary and will change in the near future.**

The following built-in commands are debug commands for interpreter developers, not scenario creators.
It is unlikely that you will need these commands for normal scenario creation.

| Name    | Effect                                                                                            |
|:--------|:--------------------------------------------------------------------------------------------------|
| error   | Generate internal error: Used to ensure that the simulator does not crash with an internal error. |
| sigsegv | Access a null pointer: Used to make sure the simulator does not crash with an internal error.     |

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

| Name                                                                                    |   Status    | Limitations                        |
|:----------------------------------------------------------------------------------------|:-----------:|:-----------------------------------|
| GlobalAction.**EnvironmentAction**                                                      | Unsupported |                                    |
| GlobalAction.EntityAction.**AddEntityAction**                                           |      ✔      |                                    |
| GlobalAction.EntityAction.**DeleteEntityAction**                                        |      ✔      |                                    |
| GlobalAction.ParameterAction.**ParameterSetAction**                                     |      ✔      | See [here](#parametersetaction)    |
| GlobalAction.ParameterAction.**ParameterModifyAction**                                  |      ✔      | No                                 |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalControllerAction** |      ✔      |                                    |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalStateAction**      |      ✔      |                                    |
| GlobalAction.TrafficAction.**TrafficSourceAction**                                      | Unsupported |                                    |
| GlobalAction.TrafficAction.**TrafficSinkAction**                                        | Unsupported |                                    |
| GlobalAction.TrafficAction.**TrafficSwarmAction**                                       | Unsupported |                                    |
| GlobalAction.TrafficAction.**TrafficStopAction**                                        | Unsupported |                                    |
| UserDefinedAction.**CustomCommandAction**                                               |      ✔      | No                                 |
| PrivateAction.LongitudinalAction.**SpeedAction**                                        |      ✔      | See [here](#speedaction)           |
| PrivateAction.LongitudinalAction.**LongitudinalDistanceAction**                         | Unsupported |                                    |
| PrivateAction.LateralAction.**LaneChangeAction**                                        |      ✔      | See [here](#lanechangeaction)      |
| PrivateAction.LateralAction.**LaneOffsetAction**                                        | Unsupported |                                    |
| PrivateAction.LateralAction.**LateralDistanceAction**                                   | Unsupported |                                    |
| PrivateAction.**VisibilityAction**                                                      | Unsupported |                                    |
| PrivateAction.**SynchronizeAction**                                                     | Unsupported |                                    |
| PrivateAction.**ActivateControllerAction**                                              | Unsupported |                                    |
| PrivateAction.ControllerAction.**AssignControllerAction**                               |      ✔      |                                    |
| PrivateAction.ControllerAction.**ActivateControllerAction**                             | Unsupported |                                    |
| PrivateAction.ControllerAction.**OverrideControllerValueAction**                        | Unsupported |                                    |
| PrivateAction.**TeleportAction**                                                        |      ✔      | See [here](#teleportaction)        |
| PrivateAction.RoutingAction.**AssignRouteAction**                                       |      ✔      |                                    |
| PrivateAction.RoutingAction.**FollowTrajectoryAction**                                  | Unsupported |                                    |
| PrivateAction.RoutingAction.**AcquirePositionAction**                                   |      ✔      | See [here](#acquirepositionaction) |

### Conditions

| Name                                                            |   Status    | Limitations                                  |
|:----------------------------------------------------------------|:-----------:|:---------------------------------------------|
| ByEntityCondition.EntityCondition.**EndOfRoadCondition**        | Unsupported |                                              |
| ByEntityCondition.EntityCondition.**CollisionCondition**        |      ✔      | See [here](#collisioncondition)              |
| ByEntityCondition.EntityCondition.**OffroadCondition**          | Unsupported |                                              |
| ByEntityCondition.EntityCondition.**TimeHeadwayCondition**      |      ✔      | See [here](#timeheadwaycondition)            |
| ByEntityCondition.EntityCondition.**TimeToCollisionCondition**  | Unsupported |                                              |
| ByEntityCondition.EntityCondition.**AccelerationCondition**     |      ✔      | No                                           |
| ByEntityCondition.EntityCondition.**StandStillCondition**       |      ✔      | No                                           |
| ByEntityCondition.EntityCondition.**SpeedCondition**            |      ✔      | No                                           |
| ByEntityCondition.EntityCondition.**RelativeSpeedCondition**    | Unsupported |                                              |
| ByEntityCondition.EntityCondition.**TraveledDistanceCondition** | Unsupported |                                              |
| ByEntityCondition.EntityCondition.**ReachPositionCondition**    |      ✔      | See [here](#reachpositioncondition)          |
| ByEntityCondition.EntityCondition.**DistanceCondition**         |      ✔      | See [here](#distancecondition)               |
| ByEntityCondition.EntityCondition.**RelativeDistanceCondition** |      ✔      | See [here](#relativedistancecondition)       |
| ByEntityCondition.**ParameterCondition**                        |      ✔      |                                              |
| ByEntityCondition.**TimeOfDayCondition**                        | Unsupported |                                              |
| ByEntityCondition.**SimulationTimeCondition**                   |      ✔      | No                                           |
| ByEntityCondition.**StoryboardElementStateCondition**           |      ✔      | See [here](#storyboardelementstatecondition) |
| ByEntityCondition.**UserDefinedValueCondition**                 |      ✔      |                                              |
| ByEntityCondition.**TrafficSignalCondition**                    |      ✔      |                                              |
| ByEntityCondition.**TrafficSignalControllerCondition**          |      ✔      |                                              |

## Limitations

### ParameterSetAction

- Currently, ParameterSetAction cannot handle `dateTime` type parameters.

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
