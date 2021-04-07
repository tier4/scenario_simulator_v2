# Communication with simulator and interpreter
We use ZeroMQ Inter-Process communication with simulator and interpreter.
All communications are synchronized.

## What is ZeroMQ
[ZeroMQ](https://zeromq.org/) is an open-source messaging library, it supports TCP/UDP/Intra-Process messaging communication.  
We use [ZeroMQ](https://zeromq.org/) in order to communicate with the simulator and interpreter.
We use Request/Reply socket in order to run the simulator synchronously.  

## Schema of the message
traffic_simulatorr::API send the request to the simulator, Requests are serialized by using [protobuf](https://developers.google.com/protocol-buffers) and use different port in order to communicate with the simulator.  

### Protobuf Definition
[Protobuf Definition](../proto_doc/protobuf.md) is here. all datas are serialized as string and send via TCP by using ZeroMQ.