# Communication with simulator and interpreter

![ZeroMQ communication](../image/inter_process_communication.png "ZeroMQ inter process communication")

We use ZeroMQ Inter-Process communication with your simulator and traffic simulator.

<font color="#065479E">*Note! Simple Sensor Simulator is just a reference implementation, so we can adapt any kinds of autonomous driving simulators if we can develop ZeroMQ interface to your simulator.*</font>

## What is ZeroMQ
[ZeroMQ](https://zeromq.org/) is an open-source messaging library, it supports TCP/UDP/Intra-Process messaging communication.  
We use [ZeroMQ](https://zeromq.org/) in order to communicate with the simulator and interpreter.
We use Request/Reply socket in order to run the simulator synchronously.  

## Schema of the message
traffic_simulatorr::API send the request to the simulator, Requests are serialized by using [protobuf](https://developers.google.com/protocol-buffers) and use different port in order to communicate with the simulator.  

### Protobuf Definition
[Schema of Protobuf](https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/simulation_interface/proto/simulation_api_schema.proto) is here.
[Protobuf Documentation](../proto_doc/protobuf.md) is here. all datas are serialized as string and send via TCP by using ZeroMQ.