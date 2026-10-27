# Communication with simulator and interpreter

![ZeroMQ communication](../image/inter_process_communication.png "ZeroMQ inter-process communication")

We use ZeroMQ Inter-Process communication to communicate with your simulator and traffic simulator.

!!!note
    Simple Sensor Simulator is just a reference implementation. We can adapt any kinds of autonomous driving simulators if we can develop ZeroMQ interface to your simulator.

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
    Traffic Simulator ->+ Simple Sensor Simulator : AttachOccupancyGridSensorRequest
    Simple Sensor Simulator ->-Traffic Simulator : AttachOccupancyGridSensorResponse
    Traffic Simulator ->+ Simple Sensor Simulator : AttachPseudoTrafficLightDetectorRequest
    Simple Sensor Simulator ->-Traffic Simulator : AttachPseudoTrafficLightDetectorResponse
    loop every frame
      Traffic Simulator ->+ Simple Sensor Simulator : UpdateEntityStatusRequest
      Simple Sensor Simulator ->-Traffic Simulator : UpdateEntityStatusResponse
      Traffic Simulator ->+ Simple Sensor Simulator : UpdateTrafficLightsRequest
      Simple Sensor Simulator ->-Traffic Simulator : UpdateTrafficLightsResponse
      Traffic Simulator ->+ Simple Sensor Simulator : UpdateFrameRequest
      Simple Sensor Simulator ->> Autoware : Send Pointcloud (ROS 2 topic)
      Simple Sensor Simulator ->> Autoware : Send Detection Result (ROS 2 topic)
      Simple Sensor Simulator ->> Autoware : Send Occupance Grid Map (ROS 2 topic)
      Simple Sensor Simulator ->> Autoware : Send Traffic Light Info (ROS 2 topic)
      Simple Sensor Simulator ->-Traffic Simulator : UpdateFrameResponse
    end
```

## Schema of the message

The `traffic_simulator::API` sends a request to the simulator. The request is serialized using protobuf and uses the port specified by the ROS Parameter `port` (default is 5555) to communicate with the simulator.

### Protobuf definition

The schema of protobuf is [here](https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/simulation_interface/proto/simulation_api_schema.proto).  
Protobuf documentation is [here](../proto_doc/protobuf.md). All data are serialized as string and sent via TCP by using ZeroMQ.

## Required APIs for co-simulation

The traffic simulator and the simple sensor simulator communicate with APIs. If you want to integrate the simulators with your simulator, only you have to do is preparing the following APIs:

| API                                 | Request                                                                                                                                                   | Response                                                                                                                                                    |
|-------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| initialize                          | [InitializeRequest](../proto_doc/protobuf.md#initializerequest)                                             | [InitializeResponse](../proto_doc/protobuf.md#initializeresponse)                                             |
| update_frame                        | [UpdateFrameRequest](../proto_doc/protobuf.md#updateframerequest)                                           | [UpdateFrameResponse](../proto_doc/protobuf.md#updateframeresponse)                                           |
| spawn_vehicle_entity                | [SpawnVehicleEntityRequest](../proto_doc/protobuf.md#spawnvehicleentityrequest)                             | [SpawnVehicleEntityResponse](../proto_doc/protobuf.md#spawnvehicleentityresponse)                             |
| spawn_pedestrian_entity             | [SpawnPedestrianEntityRequest](../proto_doc/protobuf.md#spawnpedestrianentityrequest)                       | [SpawnPedestrianEntityResponse](../proto_doc/protobuf.md#spawnpedestrianentityresponse)                       |
| spawn_misc_object_entity            | [SpawnMiscObjectEntityRequest](../proto_doc/protobuf.md#spawnmiscobjectentityrequest)                       | [SpawnMiscObjectEntityResponse](../proto_doc/protobuf.md#spawnmiscobjectentityresponse)                       |
| despawn_entity                      | [DespawnEntityRequest](../proto_doc/protobuf.md#despawnentityrequest)                                       | [DespawnEntityResponse](../proto_doc/protobuf.md#despawnentityresponse)                                       |
| update_entity_status                | [UpdateEntityStatusRequest](../proto_doc/protobuf.md#updateentitystatusrequest)                             | [UpdateEntityStatusResponse](../proto_doc/protobuf.md#updateentitystatusresponse)                             |
| attach_lidar_sensor                 | [AttachLidarSensorRequest](../proto_doc/protobuf.md#attachlidarsensorrequest)                               | [AttachLidarSensorResponse](../proto_doc/protobuf.md#attachlidarsensorresponse)                               |
| attach_detection_sensor             | [AttachDetectionSensorRequest](../proto_doc/protobuf.md#attachdetectionsensorrequest)                       | [AttachDetectionSensorResponse](../proto_doc/protobuf.md#attachdetectionsensorresponse)                       |
| attach_occupancy_grid_sensor        | [AttachOccupancyGridSensorRequest](../proto_doc/protobuf.md#attachoccupancygridsensorrequest)               | [AttachOccupancyGridSensorResponse](../proto_doc/protobuf.md#attachoccupancygridsensorresponse)               |
| attach_pseudo_traffic_light_detector | [AttachPseudoTrafficLightDetectorRequest](../proto_doc/protobuf.md#attachpseudotrafficlightdetectorrequest) | [AttachPseudoTrafficLightDetectorResponse](../proto_doc/protobuf.md#attachpseudotrafficlightdetectorresponse) |
| update_traffic_lights               | [UpdateTrafficLightsRequest](../proto_doc/protobuf.md#updatetrafficlightsrequest)                           | [UpdateTrafficLightsResponse](../proto_doc/protobuf.md#updatetrafficlightsresponse)                           |
