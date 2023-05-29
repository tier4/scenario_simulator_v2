# Configuring Perception Topics

This section describes properties for configuring perception topics that
`scenario_simulator_v2` publishes to Autoware.

## Overview

### Syntax

Perception topics are configurable with the following syntax:
```
        ObjectController:
          Controller:
            name: '...'
            Properties:
              Property:
                - name: "isEgo"
                  value: "true"
                - name: "<NAME>"
                  value: "<VALUE>"
```

where `<NAME>` and `<VALUE>` can be set to:

| Name                                      | Value                                         | Default | Description                                                                                                           |
|-------------------------------------------|-----------------------------------------------|:-------:|-----------------------------------------------------------------------------------------------------------------------|
| `detectedObjectMissingProbability`        | A `double` type value between `0.0` and `1.0` |  `0.0`  | Do not publish the perception topic with the given probability.                                                       |
| `detectedObjectPositionStandardDeviation` | A positive `double` type value                |  `0.0`  | Randomize the positions of other vehicles included in the perception topic according to the given standard deviation. |
| `detectedObjectPublishingDelay`           | A positive `double` type value                |  `0.0`  | Delays the publication of the perception topic by the specified number of seconds.                                    |
| `randomSeed`                              | A positive `integer` type value               |   `0`   | Specifies the seed value for the random number generator.                                                             |

These properties are not exclusive. In other words, multiple properties can be
specified at the same time. However, these properties only take effect for
entities that have the ObjectController's property `isEgo` set to `true` (that
is, entities controlled by Autoware). Otherwise the property is silently
ignored.

## Property `detectedObjectMissingProbability`

**Summary** - Do not publish the perception topic with the given probability.

**Purpose** - `scenario_simulator_v2` publishes the position of the vehicle in
the simulation space as it is to Autoware as a perception result. This is the
desired behavior for testing the planning module in isolation of performance
and accuracy concerns of the perception module. Because, as long as the
simulation is accurate, the perception result will not lose the position of
other vehicles. However, on the other hand, there is a problem that the fault
tolerance performance of the planning module cannot be tested. This property
addresses this problem by providing a means to avoid publishing perception
results with a certain probability.

**Specification** - The property value must be a real number between 0 and 1.
The value is the probability of not publishing the perception result. That is,
a value of 0 will publish all perception results and a value of 1 will not
publish any perception results. It is an error if the value is outside the
range of 0 to 1.

**Guarantee** - Since the random number generator is a pseudo-random number
generator, its behavior is deterministic. Therefore, as long as
`scenario_simulator_v2` can run at the specified frame rate, the loss of sight
of other vehicles is reproducible. However, since Autoware's behavior is
nondeterministic, if the scenario is such that vehicles interact with each
other, or if the scenario defines triggers for events that depend on Autoware's
behavior, the progress of the scenario itself will be nondeterministic. In
other words, `scenario_simulator_v2` makes every effort to reproducible
results, but the reproducibility of test results involving Autoware is not
guaranteed.

**Note** - This property shares the random number generator with the property
`detectedObjectPositionStandardDeviation`.

**Default behavior** - If the property is not specified, the default value is
`"0.0"`, meaning no missing other vehicles.

**[Example](https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/scenario_test_runner/scenario/Property.detectedObjectMissingProbability.yaml)** -
```
        ObjectController:
          Controller:
            name: '...'
            Properties:
              Property:
                - name: "isEgo"
                  value: "true"
                - name: "detectedObjectMissingProbability"
                  value: "0.7"
```

## Property `detectedObjectPositionStandardDeviation`

**Summary** - Randomize the positions of other vehicles included in the
perception topic according to the given standard deviation.

**Purpose** - `scenario_simulator_v2` publishes the position of the vehicle in
the simulation space as it is to Autoware as a perception result. This is the
desired behavior for testing the planning module in isolation of performance
and accuracy concerns of the perception module. Because, as long as the
simulation is accurate, the perception result will always point to the true
position. However, on the other hand, there is a problem that the fault
tolerance performance of the planning module cannot be tested. This property
addresses this problem by randomizing the vehicle positions included in the
perception results by the specified standard deviation.

**Specification** - The property's value must be a positive real number. The
value is the standard deviation. It is an error if the value is negative. The
vehicle position is randomized by adding normally distributed pseudorandom
numbers generated by a 32-bit Mersenne Twister to each of the x and y axes. The
random numbers added to the x-axis and y-axis are generated separately. See the
property `randomSeed` for how to set the pseudo-random number seed value.

**Guarantee** - Since the random number generator is a pseudo-random number
generator, its behavior is deterministic. Therefore, as long as
`scenario_simulator_v2` can run at the specified frame rate, the randomized
perception results by `scenario_simulator_v2` are reproducible. However, since
Autoware's behavior is nondeterministic, if the scenario is such that vehicles
interact with each other, or if the scenario defines triggers for events that
depend on Autoware's behavior, the progress of the scenario itself will be
nondeterministic. In other words, `scenario_simulator_v2` makes every effort to
reproducible results, but the reproducibility of test results involving
Autoware is not guaranteed.

**Note** - Distribution generators other than normal distribution are currently
not supported.

**Note** - This property shares the random number generator with the property
`detectedObjectMissingProbability`.

**Default behavior** - If the property is not specified, the default value is
`"0.0"`, meaning no randomization.

**[Example](https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/scenario_test_runner/scenario/Property.detectedObjectPositionStandardDeviation.yaml)** -
```
        ObjectController:
          Controller:
            name: '...'
            Properties:
              Property:
                - name: "isEgo"
                  value: "true"
                - name: "detectedObjectPositionStandardDeviation"
                  value: "3"
```

## Property `detectedObjectPublishingDelay`

**Summary** - Delays the publication of the perception topic by the specified
number of seconds.

**Purpose** - Normally, Autoware reflects the surrounding situation in the
steering operation by processing the data in the order of the sensing module,
perception module, planning module, and vehicle driver. However, when not
connected with AWSIM, `scenario_simulator_v2` skips the sensing module and
perception module and directly generates the data of the perception result, and
sends it to the planning module. This behavior is desirable as a test of the
planning module, but on the other hand, there is a problem that the time until
the perception result is generated is unrealistically fast in response to
changes in the environment surrounding the vehicle. This property works around
this problem by setting an interval of the specified number of seconds between
`scenario_simulator_v2` generating a perception result and publishing it.

**Specification** - The property's value must be a positive real number. The
unit is seconds. It is an error if the value is negative.　Since the delay is
set to the same value for each topic, it is not possible to delay only a
specific topic.

**Guarantee** - This delay setting ensures that `scenario_simulator_v2`
publishes the perception results in a consistent order. They are published
according to their original order. However, while `scenario_simulator_v2`
guarantees to publish in order, it does not guarantee that it reaches the
planning module in order. This is because the arrival order of topics in ROS 2
is not guaranteed.

**Note** - This feature only adjusts the interval between ssv2 generating a
perception result and publishing it. Note that there is another kind of delay
between when `scenario_simulator_v2` publishes the perception result and when
it reaches the planning module.

**Default behavior** - If the property is not specified, the default value is
`"0.0"`, meaning no delay.

**[Example](https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/scenario_test_runner/scenario/Property.detectedObjectPublishingDelay.yaml)** -
```
        ObjectController:
          Controller:
            name: '...'
            Properties:
              Property:
                - name: "isEgo"
                  value: "true"
                - name: "detectedObjectPublishingDelay"
                  value: "3"
```

## Property `randomSeed`

**Summary** - Specifies the seed value for the random number generator.

**Purpose** - By fixing the seed value of the random number generator, the
random number sequence is reproducible.

**Specification** - Gives the specified value as the seed value for the random
number generator. The random number generator is shared by the various
properties covered in this section.

**Guarantee** - Since the random number generator is a pseudo-random number
generator, the generated random number sequence is always the same as long as
the same seed value is given. Note, however, that obtaining the same sequence
of random numbers does not necessarily mean that the same scenario execution
results will be obtained since the progress of the scenario is not
deterministic.

**Default behavior** - If the property is not specified, the default value is
`"0"`.

**Example** -
```
        ObjectController:
          Controller:
            name: '...'
            Properties:
              Property:
                - name: "randomSeed"
                  value: "0"
```
