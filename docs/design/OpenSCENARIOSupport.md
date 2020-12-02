# OpenSCENARIO Support

The ROS2 package `openscenario_interpreter` provides scenario-based simulation on
[ASAM OpenSCENARIO 1.0](https://www.asam.net/standards/detail/openscenario/).
This section describes the differences between our OpenSCENARIO Interpreter and
the OpenSCENARIO standard set by ASAM, and the OpenSCENARIO implementation by
other companies and organizations.

## Specific Features
---

### ROS 2 Launch-like substitution syntax

Our interpreter supports some of the substitution syntax of
[the ROS 2 Launch system](https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration)
(a.k.a string-interpolation).
The substitution syntax works with any attribute string in OpenSCENARIO XML.

This substitution is done only once when reading the attribute.
Note that the substituion result is finalized before the simulation starts, so
it is not affected by the `ParameterModifyAction`, etc. that takes effect during
the simulation.

The supported functions and their behavior are shown below.

#### `$(find-pkg-prefix <package-name>)`

Equivalent to the following description given in ROS 2 Design (unless we made a
mistake in our implementation).

> Substituted by the install prefix path of the given package. Forward and
> backwards slashes will be resolved to the local filesystem convention.
> Substitution will fail if the package cannot be found.

The package specified must be a ROS 2 package.

#### `$(var <parameter-name>)`

Replaces with an external representation of the value of the specified
OpenSCENARIO parameter.
The parameters you specify must be declared by ParameterDeclarations before
`$ (var ...)` is written.

#### `$(dirname)`

Substitute with the path to the directory where the running scenario script is
located.

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

OpenSCENARIO has the function that the detailed behavior is left to the decision
of the implementation side, which is defined as "subject of a contract between
simulation environment provider and scenario author".

### CustomCommandAction

This Action is specified in the standard as follows.
> Used to either issue a command to the simulation environment or start an
> external script. Allows the user to activate custom actions in their
> simulation tool.

For OpenSCENARIO interpreters implemented in scripting languages such as Python,
this Action is often implemented as a call to an external script file written in
the same language as the host language.
However, our interpreter is implemented in C ++ and we cannot simply implement
such a feature.
Therefore, our interpreter treats the string given in CustomCommandAction.type
as a command and executes it on a subprocess, as `sh` does.

For example, the `echo` command can be written as follows:
``` XML
  <UserDefinedAction>
    <CustomCommandAction type="echo">Hello, world!</CustomCommandAction>
  </UserDefinedAction>
```

The string given to attribute `type` of CustomCommandAction and the string given
to its content are concatenated with whitespace and passed to the subprocess.
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

The effect of calling a command with `CustomCommandAction` is outside the
control of the interpreter.
Therefore, if you call a command that has a destructive effect on the system,
there is no guarantee that scenario execution can continue normally.

**For portable scenarios, the use of CustomCommandAction should be avoided as
much as possible or limited to the scope of POSIX.**

#### NullAction

In particular, the following usages that achieve "do nothing action" are worth
special mention.
Here, the colon (`:`) specified in the `CustomCommandAction.type` is the `sh`
command known by the name of the null-command.
``` XML
  <UserDefinedAction>
    <CustomCommandAction type=":"/>
  </UserDefinedAction>
```

#### Built-in commands

| Name        | Effect |
|:------------|:-------|
| exitSuccess | Immediately terminates the simulation as successful. |
| exitFailure | Immediately terminates the simulation as a failure.  |

These built-in commands force the simulation to terminate.
This termination ignores the StoryboardElement's lifecycle transition (that is,
it means that `StoryboardElementStateCondition` cannot be used to prevent or
detect the execution of this command).

The terminated scenario determines the final success / failure / error by
collating the called command with the expected simulation result specified in
`expect` of the
[workflow file](../user_guide/scenario_test_runner/HowToWriteWorkflowFile.md)
.

**Currently, simulation results are notified by simply writing to standard output as text.**
**This notification method is temporary and will change in the near future.**

The following built-in commands are debug commands for interpreter developers, not scenario creators.
It is unlikely that you will need these commands for normal scenario creation.

| Name    | Effect |
|:--------|:-------|
| error   | Generate internal error: Used to ensure that the simulator does not crash with an internal error. |
| sigsegv | Access a null pointer: Used to make sure the simulator does not crash with an internal error. |

## Non-Standard Extensions
---

### Success/failure judgment

Our interpreters have been developed with the intention of incorporating them
into the CI / CD pipeline of autonomous driving systems.
Therefore, executing a scenario can result in success, failure, or error.
Note that "error" means a flaw in the scenario itself, such as a syntax error,
or an internal error, not an "error in an automated driving system" such as "a
vehicle accident occurred in a simulation".
In such cases, you will be notified of a "failure".

## Supporting Status
---

Our OpenSCENARIO Interpreter does not currently support the full range of
OpenSCENARIO standards.

### Actions

| Name                                                                                    | Status      | Limitations
|:----------------------------------------------------------------------------------------|:-----------:|:------------
| GlobalAction.**EnvironmentAction**                                                      | Unsupported |
| GlobalAction.EntityAction.**AddEntityAction**                                           | Unsupported |
| GlobalAction.EntityAction.**DeleteEntityAction**                                        | Unsupported |
| GlobalAction.ParameterAction.**ParameterSetAction**                                     | ✔           | See [here](#parametersetaction)
| GlobalAction.ParameterAction.**ParameterModifyAction**                                  | ✔           |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalControllerAction** | Unsupported |
| GlobalAction.InfrastructureAction.TrafficSignalAction.**TrafficSignalStateAction**      | Unsupported |
| GlobalAction.TrafficAction.**TrafficSourceAction**                                      | Unsupported |
| GlobalAction.TrafficAction.**TrafficSinkAction**                                        | Unsupported |
| GlobalAction.TrafficAction.**TrafficSwarmAction**                                       | Unsupported |
| UserDefinedAction.**CustomCommandAction**                                               | ✔           |
| PrivateAction.LongitudinalAction.**SpeedAction**                                        | ✔           | See [here](#speedaction)
| PrivateAction.LongitudinalAction.**LongitudinalDistanceAction**                         | Unsupported |
| PrivateAction.LateralAction.**LaneChangeAction**                                        | ✔           | See [here](#lanechangeaction)
| PrivateAction.LateralAction.**LaneOffsetAction**                                        | Unsupported |
| PrivateAction.LateralAction.**LateralDistanceAction**                                   | Unsupported |
| PrivateAction.**VisibilityAction**                                                      | Unsupported |
| PrivateAction.**SynchronizeAction**                                                     | Unsupported |
| PrivateAction.**ActivateControllerAction**                                              | Unsupported |
| PrivateAction.ControllerAction.**AssignControllerAction**                               | Unsupported |
| PrivateAction.ControllerAction.**OverrideControllerAction**                             | Unsupported |
| PrivateAction.**TeleportAction**                                                        | ✔           | See [here](#teleportaction)
| PrivateAction.RoutingAction.**AssignRouteAction**                                       | Unsupported |
| PrivateAction.RoutingAction.**FollowTrajectoryAction**                                  | Unsupported |
| PrivateAction.RoutingAction.**AcquirePositionAction**                                   | ✔           | See [here](#acquirepositionaction)

### Conditions

| Name                                                                    | Status |
|:------------------------------------------------------------------------|:------:|
| ByEntityCondition.EntityCondition.**EndOfRoadCondition**                |
| ByEntityCondition.EntityCondition.**CollisionCondition**                |
| ByEntityCondition.EntityCondition.**OffroadCondition**                  |
| ByEntityCondition.EntityCondition.**TimeHeadwayCondition**              |
| ByEntityCondition.EntityCondition.**TimeToCollisionCondition**          |
| ByEntityCondition.EntityCondition.**AccelerationCondition**             |
| ByEntityCondition.EntityCondition.**StandStillCondition**               |
| ByEntityCondition.EntityCondition.**SpeedCondition**                    |
| ByEntityCondition.EntityCondition.**RelativeDistanceCondition**         |
| ByEntityCondition.EntityCondition.**TraveledDistanceCondition**         |
| ByEntityCondition.EntityCondition.**ReachPositionCondition**            |
| ByEntityCondition.EntityCondition.**DistanceCondition**                 |
| ByEntityCondition.EntityCondition.**RelativeSpeedCondition**            |
| ByEntityCondition.ByValueCondition.**ParameterCondition**               |
| ByEntityCondition.ByValueCondition.**TimeOfDayCondition**               |
| ByEntityCondition.ByValueCondition.**SimulationTimeCondition**          |
| ByEntityCondition.ByValueCondition.**StoryboardElementStateCondition**  |
| ByEntityCondition.ByValueCondition.**UserDefinedValueCondition**        |
| ByEntityCondition.ByValueCondition.**TrafficSignalCondition**           |
| ByEntityCondition.ByValueCondition.**TrafficSignalControllerCondition** |

## Limitations

### ParameterSetAction

Currently, ParameterSetAction cannot handle `dateTime` type parameters.

### SpeedAction

The implementation of type [TransitionDynamics](#transitiondynamics) for element
`SpeedActionDynamics` is incomplete and **SpeedActionDynamics.dynamicsDimention
is ignored**.

### LaneChangeAction

The implementation of type [TransitionDynamics](#transitiondynamics) for element
`LaneChangeActionDynamics` and type [LaneChangeTarget](#lanechangetarget) for
element `LaneChangeTarget` are incomplete.

### TeleportAction

Currently, **only LanePosition** can be specified for TeleportAction.

### AcquirePositionAction

Currently, **only LanePosition** can be specified for AcquirePositionAction.






### TransitionDynamics

The implementation of type [DynamicsShape](#dynamicsshape) for attribute
`dynamicsShape` is incomplete.

| Name              | Type                            | Status
|:------------------|:--------------------------------|:------:
| dynamicsShape     | [DynamicsShape](#dynamicsshape) | Incomplete
| value             | Double                          | ✔
| dynamicsDimention | DynamicsDimension               | ✔

### DynamicsShape

Currently, only `linear` and `step` are implemented for values of this
enumeration.
If you specify `cubic` and `sinusoidal`, you will get an `ImplementationFault`.

| Value      | Status      |
|:-----------|:-----------:|
| linear     | ✔           |
| cubic      | Unsupported |
| sinusoidal | Unsupported |
| step       | ✔           |

### LaneChangeTarget

Currently, only `AbsoluteTargetLane` is implemented for element of this type.
If you specify `RelativeTargetLane`, you will get an `SyntaxError`.

| Element            | Status      |
|:-------------------|:-----------:|
| AbsoluteTargetLane | ✔           |
| RelativeTargetLane | Unsupported |

### Position

Currently, only `WorldPosition` and `LanePosition` are implemented for element
of this type.
