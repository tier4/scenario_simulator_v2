# ros_ping

the node receiving any types of topic and return whether the node received it.

## How to Use

1. Execute
```bash
ros2 run ros_ping ping --ros-args -p topic_name:=<topic name>
```

2. Show Result
```bash
echo $?
```

The meaning of the return codes is shown in the table below.

| return code | description                            |
|-------------|----------------------------------------|
| 0           | Succeeded in receiving topic           |
| 1           | Timed out the attempt to receive topic |
| 2           | Found no publishers                    |
| 3           | Interrupted                            |
| 4           | Unknown error                          |

## Parameters

| parameter name          | description                                                    | default value |
|-------------------------|----------------------------------------------------------------|---------------|
| topic_name              | name of the topic                                              | /ping         |
| timeout_ms              | timeout for waiting to receive the topic (unit : milliseconds) | 1000          |
| topic_discovery_time_ms | discovery time for the topic (unit : milliseconds)             | 500           |
| verbose                 | whether to give verbose output                                 | false         |
