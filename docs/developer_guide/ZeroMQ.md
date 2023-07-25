# Communication with simulator and interpreter

![ZeroMQ communication](../image/inter_process_communication.png "ZeroMQ inter-process communication")

We use ZeroMQ Inter-Process communication to communicate with your simulator and traffic simulator.

<font color="#065479E">_Note! Simple Sensor Simulator is just a reference implementation. We can adapt any kinds of autonomous driving simulators if we can develop ZeroMQ interface to your simulator._</font>

## What is ZeroMQ

[ZeroMQ](https://zeromq.org/) is an open-source messaging library. It supports TCP/UDP/Inter-Process messaging communication.  
We use [ZeroMQ](https://zeromq.org/) in order to communicate with the simulator and interpreter.
We use Request/Reply sockets in order to run the simulators synchronously.

<iframe 
  class="hatenablogcard" 
  style="width:100%;height:155px;max-width:450px;" 
  title="embree" 
  src="https://hatenablog-parts.com/embed?url=https://zeromq.org/" 
  width="300" height="150" frameborder="0" scrolling="no">
</iframe>

## Sequence diagram

The traffic simulator has a ZeroMQ client and the simple sensor simulator has a ZeroMQ server.
We use [Protocol Buffers](https://developers.google.com/protocol-buffers) (protobuf) in order to serialize data.

```mermaid
sequenceDiagram
    participant Traffic Simulator
    participant Simple Sensor Simulator
    participant Autoware
    Traffic Simulator ->+ Simple Sensor Simulator : InitializeRequest
    Simple Sensor Simulator ->-Traffic Simulator : InitializeResponse
    Traffic Simulator ->+ Simple Sensor Simulator : SpawnVehicleEntityRequest
    Simple Sensor Simulator ->-Traffic Simulator : SpawnVehicleEntityResponse
    Traffic Simulator ->+ Simple Sensor Simulator : AttachLidarSensorRequest
    Simple Sensor Simulator ->-Traffic Simulator : AttachLidarSensorResponse
    Traffic Simulator ->+ Simple Sensor Simulator : AttachDetectionSensorRequest
    Simple Sensor Simulator ->-Traffic Simulator : AttachDetectionSensorResponse
    loop every frame
      Traffic Simulator ->+ Simple Sensor Simulator : UpdateFrameRequest
      Simple Sensor Simulator ->-Traffic Simulator : UpdateFrameResponse
      Traffic Simulator ->+ Simple Sensor Simulator : UpdateEntityStatusRequest
      Simple Sensor Simulator ->> Autoware : Send Pointcloud (ROS 2 topic)
      Simple Sensor Simulator ->> Autoware : Send Detection Result (ROS 2 topic)
      Simple Sensor Simulator ->-Traffic Simulator : UpdateEntityStatusResponse
    end
```

## Schema of the message

`traffic_simulator::API` sends the request to the simulator. The requests are serialized by using protobuf and use various ports in order to communicate with the simulator.

### Protobuf definition

The schema of protobuf is [here](https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/simulation_interface/proto/simulation_api_schema.proto).  
Protobuf documentation is [here](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf). All data are serialized as string and sent via TCP by using ZeroMQ.

## Required APIs for co-simulation

The traffic simulator and the simple sensor simulator communicate with APIs. If you want to integrate the simulators with your simulator, only you have to do is preparing the following APIs:

| API                          | Request                                                                                                                                                           | Response                                                                                                                                                            |
|------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| initialize                   | [InitializeRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.InitializeRequest)                               | [InitializeResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.InitializeResponse)                               |
| update_frame                 | [UpdateFrameRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateFrameRequest)                             | [UpdateFrameResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateFrameResponse)                             |
| spawn_vehicle_entity         | [SpawnVehicleEntityRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnVehicleEntityRequest)               | [SpawnVehicleEntityResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnVehicleEntityResponse)               |
| spawn_pedestrian_entity      | [SpawnPedestrianEntityRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnPedestrianEntityRequest)         | [SpawnPedestrianEntityResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnPedestrianEntityResponse)         |
| spawn_misc_object_entity     | [SpawnMiscObjectEntityRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnMiscObjectEntityRequest)         | [SpawnPedestrianEntityResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.SpawnMiscObjectEntityResponse)         |
| despawn_entity               | [DespawnEntityRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.DespawnEntityRequest)                         | [DespawnEntityResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.DespawnEntityResponse)                         |
| update_entity_status         | [UpdateEntityStatusRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateEntityStatusRequest)               | [UpdateEntityStatusResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateEntityStatusResponse)               |
| attach_lidar_sensor          | [AttachLidarSensorRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachLidarSensorRequest)                 | [AttachLidarSensorResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachLidarSensorResponse)                 |
| attach_detection_sensor      | [AttachDetectionSensorRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachDetectionSensorRequest)         | [AttachDetectionSensorResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachDetectionSensorResponse)         |
| attach_occupancy_grid_sensor | [AttachOccupancyGridSensorRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachOccupancyGridSensorRequest) | [AttachOccupancyGridSensorResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.AttachOccupancyGridSensorResponse) |
| update_traffic_lights        | [UpdateTrafficLightsRequest](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateTrafficLightsRequest)             | [UpdateTrafficLightsResponse](https://tier4.github.io/scenario_simulator_v2-docs/proto_doc/protobuf/#simulation_api_schema.UpdateTrafficLightsResponse)             |
