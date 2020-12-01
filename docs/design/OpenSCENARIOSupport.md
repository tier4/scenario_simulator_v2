# OpenSCENARIO Support

The ROS2 package `openscenario_interpreter` provides scenario-based simulation on
[ASAM OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/).
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
The supported functions and their behavior are shown below.

#### `$(find-pkg-prefix <package-name>)`

Equivalent to the following description given in ROS 2 Design (unless we made a
mistake in our implementation).

> Substituted by the install prefix path of the given package. Forward anda
> backwards slashes will be resolved to the local filesystem convention.
> Substitution will fail if the package cannot be found.

#### `$(var <parameter-name>)`

#### `$(dirname)`


## Implementation Dependent Behaviors
---

OpenSCENARIO has the function that the detailed behavior is left to the decision
of the implementation side, which is defined as "subject of a contract between
simulation environment provider and scenario author".
A typical example is "CustomCommandAction".

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

#### Builtin commands

TODO

## Non-Standard Extensions
---

### Supports Lanelet2 instead of OpenDRIVE.

TODO
