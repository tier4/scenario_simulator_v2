# Parameters

This section describes how to configure the topics that `scenario_simulator_v2`
publishes to Autoware.

## Overview

The topics that `scenario_simulator_v2` publishes to Autoware are configurable
from the ROS 2 parameter file given to the launch argument
`parameter_file_path` of scenario_test_runner. The default value of
`parameter_file_path` is the path to [a sample parameter
file](https://github.com/tier4/scenario_simulator_v2/blob/master/test_runner/scenario_test_runner/config/parameters.yaml).

All parameters that can be specified and their default values are shown in the
sample parameter file. In practice, it is not necessary to specify all
parameters except for some that are mandatory. In that case, the simulator will
behave as if similar default values had been specified.

There are currently two ways to configure some topics: an old way and a new way
described on this page. The new way is backward compatible and is the
recommended way. If you want to know how to use the old way, [see this
page](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/ConfiguringPerceptionTopics/).

## /perception/object_recognition/detection/objects

### `version`

An `int` type value in YYYYMMDD format, mandatory.
Suffix of `scenario_test_runner` launch argument `architecture_type`, used to
maintain backward compatibility of the simulator when changing the Autoware
interface.

### `seed`

A positive `int` type value, default `0`.
The seed value for the random number generator. If `0` is specified, a random
seed value will be generated for each run.

### `override_legacy_configuration`

A `boolean` type value, default `false`.
Some of the parameters described below can be configured in either the old or
new way. This parameter is used to determine which value to use. That is, as
long as this parameter is `false`, some of the following parameters will be
ignored and the values set by the old method will be used. If you want to
configure the new way, set it to `true`. For backward compatibility, the
default value of this parameter is `false`.

### `delay`

A positive `double` type value, default `0.0`. The unit is seconds. It is an
error if the value is negative.
Delays the publication of the topic by the specified number of seconds. This
parameter is used only if `override_legacy_configuration` is true. If it is
false, the value of `detectedObjectPublishingDelay` in
`ObjectController.Properties` in the scenario file is used.

### `range`

A positive `double` type value, default `300.0`. The unit is meters.
The sensor detection range. This parameter is used only if
`override_legacy_configuration` is true. If it is false, the value of
`detectionSensorRange` in `ObjectController.Properties` in the scenario file is
used.

### `occlusionless`

A `boolean` type value, default `false`.
The message is a simulated object recognition result based on a pointcloud.
Pointclouds are usually sensed by LiDAR, and scenario_simulator_v2 assumes this
and simulates it, including LiDAR occlusion. If this parameter is `true`,
object recognition is simulated as if there is no occlusion. In other words, it
produces recognition results as if objects behind the object are also visible
(even though they are in shadow and invisible in normal LiDAR).  This parameter
is used only if `override_legacy_configuration` is true. If it is false, the
value of `isClairvoyant` in `ObjectController.Properties` in the scenario file
is used.

### `noise.model.version`

A positive `int` type value, default `1`. If a non-existent version is
specified, it is an error.
This parameter specifies the version of the noise model to be used. Currently,
the following two noise models are implemented:
- version: 1 - Simple noise model with position randomization
- version: 2 - Elliptically approximated model of noise variation with distance
  from the ego entity

The parameters specific to the models are placed under `noise.v1.` and
`noise.v2`, respectively.

### `noise.v1.position.standard_deviation`

A positive `double` type value, default `0.0`.
Standard deviation used for randomization of the position of the vehicle in the
message. This parameter is used only if the value of `noise.model.version` is
`1`.

### `noise.v1.missing_probability`

A `double` type value between `0.0` and `1.0`, default `0.0`.
Based on the probability specified by the value of this parameter, random
vehicle data is removed from the message. This parameter is used only if the
value of `noise.model.version` is `1`.

### `noise.v2.ellipse_y_radii`

Array of positive double type values, default `[10.0, 20.0, 40.0, 60.0, 80.0,
120.0, 150.0, 180.0, 1000.0]`. Units are in meters. The size of the array is
arbitrary, but must be the same size as the array described later.
This parameter is used only if the value of `noise.model.version` is `2`.

### `noise.v2.distance.autocorrelation_coefficient.amplitude`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
distance noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as AR(1) model. The autocorrelation coefficient $\phi$ is used in
the model to calculate the position noise $X_\mathrm{distance}$: as follows:
$$X_\mathrm{distance}(t) = \mathtt{mean} + \phi * (X_\mathrm{distance}(t-1) -
\mathtt{mean}) + \mathcal{N}(0, 1 - \phi^2) * \mathtt{standard\_deviation}$$
This parameter is used only if the value of `noise.model.version` is `2`.

### `noise.v2.distance.autocorrelation_coefficient.decay`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
distance noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as AR(1) model. The autocorrelation coefficient $\phi$ is used in
the model to calculate the position noise $X_\mathrm{distance}$: as follows:
$$X_\mathrm{distance}(t) = \mathtt{mean} + \phi * (X_\mathrm{distance}(t -
\varDelta t) - \mathtt{mean}) + \mathcal{N}(0, 1 - \phi^2) *
\mathtt{standard\_deviation}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.distance.autocorrelation_coefficient.offset`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
distance noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as AR(1) model. The autocorrelation coefficient $\phi$ is used in
the model to calculate the position noise $X_\mathrm{distance}$: as follows:
$$X_\mathrm{distance}(t) = \mathtt{mean} + \phi * (X_\mathrm{distance}(t -
\varDelta t) - \mathtt{mean}) + \mathcal{N}(0, 1 - \phi^2) *
\mathtt{standard\_deviation}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.distance.mean.ellipse_normalized_x_radius`

A positive `double` type value, default `0.0`.
The noise models the space as an elliptical model. This parameter is the ratio
of the radius of the x-axis to the radius of the y-axis of that ellipse. The
coordinate system is a right-handed local coordinate system, where the x-axis
is the longitudinal direction of the ego entity and the y-axis is its lateral
direction. The value of this parameter is used to calculate the distance $d$
between the ego entity and the other vehicle using the following equation: $$d
= \sqrt[2]{(\varDelta x / \mathtt{ellipse\_normalized\_x\_radius})^2 +
\varDelta y^2}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.distance.mean.values`

Array of positive double type values, default `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0]`.
Each element of the array is a mean of normal distribution. The first element
with a value greater than $d$ is searched from `ellipse_y_radii` and the
elements with the same index are referenced from `values`. Therefore, the array
size of this parameter must be the same as `ellipse_y_radii`. Otherwise, it is
an error. This parameter is used only if the value of `noise.model.version` is
`2`.

### `noise.v2.distance.standard_deviation.ellipse_normalized_x_radius`

A positive `double` type value, default `0.0`.
The noise models the space as an elliptical model. This parameter is the ratio
of the radius of the x-axis to the radius of the y-axis of that ellipse. The
coordinate system is a right-handed local coordinate system, where the x-axis
is the longitudinal direction of the ego entity and the y-axis is its lateral
direction. The value of this parameter is used to calculate the distance $d$
between the ego entity and the other vehicle using the following equation: $$d
= \sqrt[2]{(\varDelta x / \mathtt{ellipse\_normalized\_x\_radius})^2 +
\varDelta y^2}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.distance.standard_deviation.values`

Array of positive double type values, default `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0]`.
Each element of the array is a standard deviation of normal distribution. The
first element with a value greater than $d$ is searched from `ellipse_y_radii`
and the elements with the same index are referenced from `values`. Therefore,
the array size of this parameter must be the same as `ellipse_y_radii`.
Otherwise, it is an error. This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw.autocorrelation_coefficient.amplitude`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of yaw
noise. The autocorrelation coefficient $\phi$ is calculated by the following
equation: $$ \phi(\varDelta t) = \mathtt{amplitude} * \exp(-\mathtt{decay} *
\varDelta t) + \mathtt{offset}$$ The noise models the time series as AR(1)
model. The autocorrelation coefficient $\phi$ is used in the model to calculate
the yaw noise $X_\mathrm{yaw}$: as follows: $$ X_\mathrm{yaw}(t) =
\mathtt{mean} + \phi * (X_\mathrm{yaw}(t - \varDelta t) - \mathtt{mean}) +
\mathcal{N}(0, 1 - \phi^2) * \mathtt{standard\_deviation}$$ This parameter is
used only if the value of `noise.model.version` is `2`.

### `noise.v2.yaw.autocorrelation_coefficient.decay`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of yaw
noise. The autocorrelation coefficient $\phi$ is calculated by the following
equation: $$ \phi(\varDelta t) = \mathtt{amplitude} * \exp(-\mathtt{decay} *
\varDelta t) + \mathtt{offset}$$ The noise models the time series as AR(1)
model. The autocorrelation coefficient $\phi$ is used in the model to calculate
the yaw noise $X_\mathrm{yaw}$: as follows: $$ X_\mathrm{yaw}(t) =
\mathtt{mean} + \phi * (X_\mathrm{yaw}(t - \varDelta t) - \mathtt{mean}) +
\mathcal{N}(0, 1 - \phi^2) * \mathtt{standard\_deviation}$$ This parameter is
used only if the value of `noise.model.version` is `2`.

### `noise.v2.yaw.autocorrelation_coefficient.offset`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of yaw
noise. The autocorrelation coefficient $\phi$ is calculated by the following
equation: $$ \phi(\varDelta t) = \mathtt{amplitude} * \exp(-\mathtt{decay} *
\varDelta t) + \mathtt{offset}$$ The noise models the time series as AR(1)
model. The autocorrelation coefficient $\phi$ is used in the model to calculate
the yaw noise $X_\mathrm{yaw}$: as follows: $$ X_\mathrm{yaw}(t) =
\mathtt{mean} + \phi * (X_\mathrm{yaw}(t - \varDelta t) - \mathtt{mean}) +
\mathcal{N}(0, 1 - \phi^2) * \mathtt{standard\_deviation}$$ This parameter is
used only if the value of `noise.model.version` is `2`.

### `noise.v2.yaw.mean.ellipse_normalized_x_radius`

A positive `double` type value, default `0.0`.
The noise models the space as an elliptical model. This parameter is the ratio
of the radius of the x-axis to the radius of the y-axis of that ellipse. The
coordinate system is a right-handed local coordinate system, where the x-axis
is the longitudinal direction of the ego entity and the y-axis is its lateral
direction. The value of this parameter is used to calculate the distance $d$
between the ego entity and the other vehicle using the following equation: $$d
= \sqrt[2]{(\varDelta x / \mathtt{ellipse\_normalized\_x\_radius})^2 +
\varDelta y^2}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw.mean.values`

Array of positive double type values, default `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0]`.
Each element of the array is a mean of normal distribution. The first element
with a value greater than $d$ is searched from `ellipse_y_radii` and the
elements with the same index are referenced from `values`. Therefore, the array
size of this parameter must be the same as `ellipse_y_radii`. Otherwise, it is
an error. This parameter is used only if the value of `noise.model.version` is
`2`.

### `noise.v2.yaw.standard_deviation.ellipse_normalized_x_radius`

A positive `double` type value, default `0.0`.
The noise models the space as an elliptical model. This parameter is the ratio
of the radius of the x-axis to the radius of the y-axis of that ellipse. The
coordinate system is a right-handed local coordinate system, where the x-axis
is the longitudinal direction of the ego entity and the y-axis is its lateral
direction. The value of this parameter is used to calculate the distance $d$
between the ego entity and the other vehicle using the following equation: $$d
= \sqrt[2]{(\varDelta x / \mathtt{ellipse\_normalized\_x\_radius})^2 +
\varDelta y^2}$$. This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw.standard_deviation.values`

Array of positive double type values, default `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0]`.
Each element of the array is a standard deviation of normal distribution. The
first element with a value greater than $d$ is searched from `ellipse_y_radii`
and the elements with the same index are referenced from `values`. Therefore,
the array size of this parameter must be the same as `ellipse_y_radii`.
Otherwise, it is an error. This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw_flip.autocorrelation_coefficient.amplitude`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
yaw-flip noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the yaw-flip noise with following transition matrix:
$$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix} =
\begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw_flip.autocorrelation_coefficient.decay`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
yaw-flip noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the yaw-flip noise with following transition matrix:
$$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix} =
\begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$  This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw_flip.autocorrelation_coefficient.offset`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
yaw-flip noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the yaw-flip noise with following transition matrix:
$$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix} =
\begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw_flip.speed_threshold`

A positive `double` type value, default `0.1`.
When the absolute speed of one other vehicle is less than the value of this
parameter, it is determined whether yaw-flip occurs or not based on the `rate`
described below. This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.yaw_flip.rate`

A positive `double` type value, default `0.0`.
Vehicles whose absolute speed is below the aforementioned `speed_threshold`
will have yaw-flip noise applied with the probability of the value of this
parameter. This parameter is used only if the value of `noise.model.version` is
`2`.

### `noise.v2.true_positive.autocorrelation_coefficient.amplitude`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
random-mask noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the random-mask noise with following transition
matrix: $$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix}
= \begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.true_positive.autocorrelation_coefficient.decay`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
random-mask noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the random-mask noise with following transition
matrix: $$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix}
= \begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.true_positive.autocorrelation_coefficient.offset`

A positive `double` type value, default `0.0`.
The parameter of the autocorrelation coefficient used in the generation of
random-mask noise. The autocorrelation coefficient $\phi$ is calculated by the
following equation: $$\phi(\varDelta t) = \mathtt{amplitude} *
\exp(-\mathtt{decay} * \varDelta t) + \mathtt{offset}$$ The noise models the
time series as Markov process. The autocorrelation coefficient $\phi$ is used
in the model to calculate the random-mask noise with following transition
matrix: $$ \begin{bmatrix} p_{0,0} & p_{0,1} \\ p_{1,0} & p_{1,1} \end{bmatrix}
= \begin{bmatrix} \pi_0 + \phi \pi_1 && \pi_1 (1 - \phi) \\ \pi_0 (1 - \phi) &&
\pi_1 - \phi \pi_0 \end{bmatrix}$$ This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.true_positive.rate.ellipse_normalized_x_radius`

A positive `double` type value, default `0.0`.
The noise models the space as an elliptical model. This parameter is the ratio
of the radius of the x-axis to the radius of the y-axis of that ellipse. The
coordinate system is a right-handed local coordinate system, where the x-axis
is the longitudinal direction of the ego entity and the y-axis is its lateral
direction. The value of this parameter is used to calculate the distance $d$
between the ego entity and the other vehicle using the following equation: $$d
= \sqrt[2]{(\varDelta x / \mathtt{ellipse\_normalized\_x\_radius})^2 +
\varDelta y^2}$$. This parameter is used only if the value of
`noise.model.version` is `2`.

### `noise.v2.true_positive.rate.values`

Array of positive double type values, default `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
1.0, 1.0, 1.0]`.
Each element of this array is the probability that the value will be output
correctly (true positive rate). The first element with a value greater than $d$
is searched from `ellipse_y_radii` and the elements with the same index are
referenced from `values`. Therefore, the array size of this parameter must be
the same as `ellipse_y_radii`. Otherwise, it is an error. This parameter is
used only if the value of `noise.model.version` is `2`.

## /perception/object_recognition/ground_truth/objects

### `version`

An `int` type value in YYYYMMDD format, mandatory.
Suffix of `scenario_test_runner` launch argument `architecture_type`, used to
maintain backward compatibility of the simulator when changing the Autoware
interface.

### `override_legacy_configuration`

A `boolean` type value, default `false`.
Some of the parameters described below can be configured in either the old or
new way. This parameter is used to determine which value to use. That is, as
long as this parameter is `false`, some of the following parameters will be
ignored and the values set by the old method will be used. If you want to
configure the new way, set it to `true`. For backward compatibility, the
default value of this parameter is `false`.

### `delay`

A positive `double` type value, default `0.0`. The unit is seconds. It is an
error if the value is negative.
Delays the publication of the topic by the specified number of seconds. This
parameter is used only if `override_legacy_configuration` is true. If it is
false, the value of `detectedObjectGroundTruthPublishingDelay` in
`ObjectController.Properties` in the scenario file is used.
