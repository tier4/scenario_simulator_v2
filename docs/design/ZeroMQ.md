# ZeroMQ Inter-Process Comunication
## What is ZeroMQ
[ZeroMQ](https://zeromq.org/) is a open-source messaging library, it supports TCP/UDP/Intra-Process messaging communication.  
We use zeromq in order to communicate wiht simulator and interpretor.  
We use Request/Reply socket in order to run simulation synchronously.  

## Schema of the message
scenario_simulator::API send the Request to the Simulator, Requests are serialized by using [protobuf](https://developers.google.com/protocol-buffers) and use defferent port in order to communicate with Simulator.  

ports and protobuf schemas are below.  
