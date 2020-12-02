# OpenSCENARIO Support

The ROS2 package `openscenario_interpreter` provides scenario-based simulation on
[ASAM OpenSCENARIO 1.0](https://www.asam.net/standards/detail/openscenario/).
This section describes the differences between our OpenSCENARIO Interpreter and
the OpenSCENARIO standard set by ASAM, and the OpenSCENARIO implementation by
other companies and organizations.

Our OpenSCENARIO Interpreter does not currently support the full range of
OpenSCENARIO standards.

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

### Supports Lanelet2 instead of OpenDRIVE.

TODO

### Success/failure judgment

我々のインタプリタは自動運転システムの CI/CD パイプラインに組み込まれることを念頭に開発されています。
そのため、シナリオの実行には成功・失敗・エラーのいずれかの結果が伴います。

ここで、「エラー」は文法エラー等のシナリオそのものの不備を意味し、「シミュレーション内で車両が事故を起こした」といった「自動運転システムのエラー」を意味しないことに注意してください。
そのような場合には、「失敗」が通知されます。

## Supporting Status

TODO
