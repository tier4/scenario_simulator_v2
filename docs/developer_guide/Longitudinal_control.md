# Longitudinal Control

traffic_simulator has various ways to control the longitudinal behavior of the npc.

## List of Longitudinal Controllers

| Name                                                      | Description                                  |
| --------------------------------------------------------- | -------------------------------------------- |
| [**requestSpeedChange**](#requestSpeedChange)             | Changes the speed of the npc.                |
| [**requestSynchronize**](#requestSynchronize)             | Synchronizes the npc with the target entity. |
| [**setLinearVelocity**](#setLinearVelocity)               | Sets the linear velocity of the npc.         |
| [**setTwist**](#setTwist)                                 | Sets the twist of the npc.                   |
| [**setAcceleration**](#setAcceleration)                   | Sets the acceleration of the npc.            |
| [**setAccelerationLimit**](#setAccelerationLimit)         | Sets the acceleration limit of the npc.      |
| [**setAccelerationRateLimit**](#setAccelerationRateLimit) | Sets the acceleration rate limit of the npc. |
| [**setDecelerationLimit**](#setDecelerationLimit)         | Sets the deceleration limit of the npc.      |
| [**setDecelerationRateLimit**](#setDecelerationRateLimit) | Sets the deceleration rate limit of the npc. |
| [**setVelocityLimit**](#setVelocityLimit)                 | Sets the velocity limit of the npc.          |

## Details
### requestSpeedChange
By using `API::requestSpeedChange`, you can change the speed of the npc.
MiscObjectEntity can not be controlled by this API.

| Value        | Type   | Description                                                 |
| ------------ | ------ | ----------------------------------------------------------- |
| name         | string | Name of the npc.                                            |
| target_speed | double | Target speed of the npc.                                    |
| continuous   | bool   | If true the npc will keep the speed until the next command. |
#### EgoEntity
`target_speed` will be set as initial target speed of the EgoEntity only before the scenario starts.

#### Other entity
The function will change the target speed of entity to `target_speed` immediately.
If `continuous` is set to `false`, job to accelerate to target speed will be deleted after the velocity has reached the target speed. If set to `true`, the npc will keep the speed until the next longitudinal control command ordered. It will accelerate on maximum acceleration rate set previously.

| Value        | Type                     | Description                                                 |
| ------------ | ------------------------ | ----------------------------------------------------------- |
| name         | string                   | Name of the npc.                                            |
| target_speed | double                   | Target speed of the npc.                                    |
| transition   | speed_change::Transition | Transition type.                                            |
| constraint   | speed_change::Constraint | Constraint type.                                            |
| continuous   | bool                     | If true the npc will keep the speed until the next command. |
#### EgoEntity
`target_speed` will be set as initial target speed of the EgoEntity only before the scenario starts.

#### Other entity
If `continuous` is set to `false`, job to accelerate to target speed will be deleted after the velocity has reached the target speed. If set to `true`, the npc will keep the speed until the next longitudinal control command ordered.

When `constraint` is set to `LONGITUDINAL_ACCELERATION`, the entity will accelerate to the target speed with acceleration rate you set.
- If `transition` is set to `LINEAR`, the entity will accelerate to the target speed linearly.
- If `transition` is set to `AUTO`, it will change the maximum acceleration speed of the entity and append the job to change the target speed of the npc to the job queue. After the npc reaches the target speed, it will change the maximum acceleration speed back to the original value.
- If `transition` is set to `STEP`, it will


When `constraint` is set to `TIME`, the entity will accelerate to the target speed by the time you set.
When `constraint` is set to `NONE`, is will append the job to change the target_speed of the npc to the job queue.

`change target speed` block in the below figure is the highest priority compared to other blocks.
If target speed is set somewhere else, the entity will try to change the speed to the target speed under acceleration/deceleration constraints set by the user or if not by the default value.
![requestSpeedChange](../images/Longitudinal_control/requestSpeedChange.png)


| Value        | Type                              | Description                                                 |
| ------------ | --------------------------------- | ----------------------------------------------------------- |
| name         | string                            | Name of the npc.                                            |
| target_speed | speed_change::RelativeTargetSpeed | Relative target speed.                                      |
| continuous   | bool                              | If true the npc will keep the speed until the next command. |

| Value        | Type                              | Description                                                 |
| ------------ | --------------------------------- | ----------------------------------------------------------- |
| name         | string                            | Name of the npc.                                            |
| target_speed | speed_change::RelativeTargetSpeed | Relative target speed.                                      |
| transition   | speed_change::Transition          | Transition type.                                            |
| constraint   | speed_change::Constraint          | Constraint type.                                            |
| continuous   | bool                              | If true the npc will keep the speed until the next command. |

### requestSynchronize
By using `API::requestSynchronize`, you can request the entity to adjust speed to stop at the designated lanelet by the time target entity crosses the another designated lanelet.

| Value            | Description                                                            |
| ---------------- | ---------------------------------------------------------------------- |
| name             | Name of the npc.                                                       |
| target_name      | Name of the target entity.                                             |
| target_sync_pose | Target lanelet pose for target entity.                                 |
| entity_target    | Target lanelet pose for controlling entity.                            |
| target_speed     | Target speed for controlling entity (meter per second).                |
| tolerance        | Tolerance for how much margin to accept to stop at the target (meter). |

### setLinearVelocity
By using `API::setLinearVelocity`, you can set the linear velocity of the npc.

| Value           | Type   | Description                 |
| --------------- | ------ | --------------------------- |
| name            | string | Name of the npc.            |
| linear_velocity | double | Linear velocity of the npc. |

### setTwist
By using `API::setTwist`, you can set the twist of the npc.

| Value | Type                      | Description       |
| ----- | ------------------------- | ----------------- |
| name  | string                    | Name of the npc.  |
| twist | geometry_msgs::msg::Twist | Twist of the npc. |

### setAcceleration

By using `API::setAcceleration`, you can set the acceleration of the npc.

| Value        | Type                      | Description              |
| ------------ | ------------------------- | ------------------------ |
| name         | string                    | Name of the npc.         |
| acceleration | geometry_msgs::msg::Accel | Acceleration of the npc. |

### setAccelerationLimit

By using `API::setAccelerationLimit`, you can set the acceleration limit of the npc.

| Value        | Type   | Description                    |
| ------------ | ------ | ------------------------------ |
| name         | string | Name of the npc.               |
| acceleration | double | Acceleration limit of the npc. |

### setAccelerationRateLimit

By using `API::setAccelerationRateLimit`, you can set the acceleration rate limit of the npc.

| Value             | Type   | Description                         |
| ----------------- | ------ | ----------------------------------- |
| name              | string | Name of the npc.                    |
| acceleration_rate | double | Acceleration rate limit of the npc. |

### setDecelerationLimit

By using `API::setDecelerationLimit`, you can set the deceleration limit of the npc.

| Value        | Type   | Description                    |
| ------------ | ------ | ------------------------------ |
| name         | string | Name of the npc.               |
| deceleration | double | Deceleration limit of the npc. |

### setDecelerationRateLimit

By using `API::setDecelerationRateLimit`, you can set the deceleration rate limit of the npc.

| Value             | Type   | Description                         |
| ----------------- | ------ | ----------------------------------- |
| name              | string | Name of the npc.                    |
| deceleration_rate | double | Deceleration rate limit of the npc. |

### setVelocityLimit

By using `API::setVelocityLimit`, you can set the velocity limit of the npc.

| Value           | Type   | Description                |
| --------------- | ------ | -------------------------- |
| name            | string | Name of the npc.           |
| linear_velocity | double | Velocity limit of the npc. |
