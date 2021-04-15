# Simple sensor simulator

![simple sensor simulator](../image/simple_sensor_simulator.png "simple sensor simulator")

The simple sensor simulator is a reference implementation of the simulator which follows our scenario testing framework.
This package includes very very simple lidar simulation and send simulated detection result to the Autoware.

## Sequence Diagram

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
      Simple Sensor Simulator ->> Autoware : Send Pointcloud (ROS2 topic)
      Simple Sensor Simulator ->> Autoware : Send Detection Result (ROS2 topic)
      Simple Sensor Simulator ->-Traffic Simulator : UpdateEntityStatusResponse
    end
```

## Required APIs to communicate with traffic simulator and sensor simulator
### Initialize
[InitializeRequest](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.InitializeRequest)  
[InitializeResponse](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.InitializeResponse)  

[UpdateFrameRequest](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.UpdateFrameRequest)  
[UpdateFrameResponse](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.UpdateFrameResponse)  
[UpdateSensorFrameRequest](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.UpdateSensorFrameRequest)  
[UpdateSensorFrameResponse](https://tier4.github.io/scenario_simulator_v2/proto_doc/protobuf/#simulation_api_schema.UpdateSensorFrameResponse)  

In lidar simulation, we use intel's ray-casting library embree.

<iframe 
  class="hatenablogcard" 
  style="width:100%;height:155px;max-width:450px;" 
  title="embree" 
  src="https://hatenablog-parts.com/embed?url=https://github.com/embree/embree" 
  width="300" height="150" frameborder="0" scrolling="no">
</iframe>

