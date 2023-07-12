# Simple sensor simulator

![simple sensor simulator](../image/simple_sensor_simulator.png "simple sensor simulator")

The simple sensor simulator is a reference implementation of the simulator which follows our scenario testing framework.  
This package includes very simple sensor or detection result simulation.

Our simple sensor simulators do not include noise simulation, because scenario_simulator_v2 is a testing framework for planners or controllers instead of any perception modules. 


[//]: # (This package includes very, very simple lidar simulation and send simulated detection result to the Autoware.)

<font color="#065479E">_Note! Simple Sensor Simulator is just a reference implementation, so we can adapt any kinds of autonomous driving simulators if we can develop ZeroMQ interface to your simulator._</font>


Architecture of the sensor simulation is below.

```mermaid
sequenceDiagram

openscenario_interpretor ->> simple_sensor_simulator: Request attaching sensors.
simple_sensor_simulator ->> lidar_sensor: Attaching LiDAR sensors and configure.
simple_sensor_simulator ->> detection_sensor: Attaching detection sensors and configure.
simple_sensor_simulator ->> occupancy_grid_sensor: Attaching occupancy grid sensors and configure.

loop Update sensor frame
    openscenario_interpretor ->> simple_sensor_simulator: Update each entity pose.
    simple_sensor_simulator ->> openscenario_interpretor: Notify finished update.
    openscenario_interpretor ->> simple_sensor_simulator: Update requesting sensor frame.
    simple_sensor_simulator ->> lidar_sensor: Update LiDAR sensor
    lidar_sensor ->> lidar_sensor: Execute ray-tracing, memorize hit entities.
    lidar_sensor ->> Autoware : Publish simulated point cloud.
    lidar_sensor ->> simple_sensor_simulator: Notify finished LiDAR simulation.

    simple_sensor_simulator ->> detection_sensor: Update detection sensors.
    alt Clairvoyant mode (filter_by_range = true)
        lidar_sensor ->> detection_sensor: Hit entities while tracing ray.
    else
        simple_sensor_simulator ->> detection_sensor: Entities existing in the surrounding N(m)
    end
    detection_sensor ->> detection_sensor: Generate detection results from entities.
    detection_sensor ->> Autoware : Public detection results.
    detection_sensor ->> simple_sensor_simulator: Notify finished detection results simulation.
    alt Clairvoyant mode (filter_by_range = true)
        lidar_sensor ->> occupancy_grid_sensor: Hit entities while tracing ray.
    else
        simple_sensor_simulator ->> occupancy_grid_sensor: Entities existing in the surrounding N(m)
    end
    occupancy_grid_sensor ->> occupancy_grid_sensor: Generate occupancy grid map from bounding boxes of the entities.
    occupancy_grid_sensor ->> Autoware : Publish occupancy grid map.
    occupancy_grid_sensor ->> simple_sensor_simulator: Notify finished occupancy grid simulation.
end
```

## LiDAR Simulation
With this simulation, we can get lidar point-cloud data based on simple ray-casting algorithm.

### Interfaces

| interface                                   | type                                   | note                                                  |
|---------------------------------------------|----------------------------------------|-------------------------------------------------------|
| `traffic_simulator::API::attachLidarSensor` | C++ traffic simulator API interface    |                                                       |
| `attach_lidar_sensor`                       | ZeroMQ traffic simulator API interface | See [ZeroMQ Interfaces documentation](ZeroMQ.md)<br/> |

### Configuration

See [the Lidar Configuration documentation](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#lidarconfiguration)

### Acknowledgments
In lidar simulation, we use intel's ray-casting library embree.

<iframe
class="hatenablogcard"
style="width:100%;height:155px;max-width:450px;"
title="embree"
src="https://hatenablog-parts.com/embed?url=https://github.com/embree/embree"
width="300" height="150" frameborder="0" scrolling="no">
</iframe>

## Occupancy Grid Sensor Simulation
With this simulation, we can get a cost map without processing lidar point cloud data.  
This enables us to reduce computational resources when we want to test Autoware's planners or controllers.  

### Interfaces

| interface                                           | type                                   | note                                                  |
|-----------------------------------------------------|----------------------------------------|-------------------------------------------------------|
| `traffic_simulator::API::attachOccupancyGridSensor` | C++ traffic simulator API interface    |                                                       |
| `attach_occupancy_grid_sensor`                      | ZeroMQ traffic simulator API interface | See [ZeroMQ Interfaces documentation](ZeroMQ.md)<br/> |

### Occupancy Grid Values

| grid type      | value | description                                                         |
|----------------|-------|---------------------------------------------------------------------|
| occupied grid  | 100   | a grid determined by a simulated lidar sensor that an object exists |
| invisible grid | 50    | a grid that is out of range or occlusion of simulated lidar sensor  |
| empty grid     | 0     | an empty grid proved by a simulated ray-cast passing through        |



### Configuration

See [the OccupancyGridSensorConfiguration documentation](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#occupancygridsensorconfiguration)

## Object Detection Results Simulation
With this simulation, you can get object detection results without processing images by heavy object detection algorithms.  
This also enables you to reduce computational resources when you want to test Autoware's planners or controllers.

### Interfaces

| interface                                       | type                                   | note                                                  |
|-------------------------------------------------|----------------------------------------|-------------------------------------------------------|
| `traffic_simulator::API::attachDetectionSensor` | C++ traffic simulator API interface    |                                                       |
| `attach_detection_sensor`                       | ZeroMQ traffic simulator API interface | See [ZeroMQ Interfaces documentation](ZeroMQ.md)<br/> |


### Configuration

See [the DetectionSensorConfiguration documentation](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#detectionsensorconfiguration)
