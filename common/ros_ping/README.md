# ros_ping

This package can be used anywhere as long as the queried node implements a ping server with `std_srvs::srv::Empty`

## How to Use

1. Execute
```bash
ros2 run ros_ping ping
```

2. Show Result
```bash
echo $?
```

The meaning of the return codes is shown in the table below.

| return code | description                                               |
|-------------|-----------------------------------------------------------|
| 0           | node is alive                                             |
| 1           | service server is not found in the time limit             |
| 2           | there is no response to the request within the time limit |
| 3           | the request is interrupted                                |
| 4           | unknown error                                             |

## Parameters

| parameter name        | description                                                            | default value    |
|-----------------------|------------------------------------------------------------------------|------------------|
| service_name          | the name of the service to call                                        | /ping |
| connection_timeout_ms | timeout for establishing connection with service (unit : milliseconds) | 1000             |
| request_timeout_ms    | timeout for waiting for service response (unit : milliseconds)         | 1000             | 

You can set parameters like below
```bash
ros2 run ros_ping ping --ros-args -p connection_timeout_ms:=500

```

## How to Integrate `ros_ping` server into your node

### Add `ros_ping` to your node dependencies

Add to `package.xml` and `CMakeLists.txt`

### Add `ros_ping::Server` to your node class as a member

```your_node.hpp
#include <ros_ping/server.hpp>

class YourNode : public rclcpp::Node {
    private:
        ros_ping::Server ros_ping_server;
    public:
        YourNode(rclcpp::NodeOptions options) : 
            rclcpp::Node(options),
            ros_ping_server(*this) {}
        
        // your main loop
        void mainLoop(){
            // received ping request won't be returned 
            // until notifyAlive function is executed
            ros_ping_server.notifyAlive();
        }
};
```

### Specify `MultiThreadExecutor` for your node
If you don't specify a `MultiThreadExecutor` for the executor, 
it can cause a deadlock depending on where you call `notifyAlive`
```node.cpp
rclcpp::executors::MultiThreadedExecutor executor;
auto node = std::make_shared<YourNode>(rclcpp::NodeOptions());
executor.add_node((*node).get_node_base_interface());
```
