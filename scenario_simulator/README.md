# Scenario Simulator
## reference implimentation of the scenario simulator

how to use
```
roslaunch scenario_simulator scenario_simulator.launch
```

## XMLRPC API Reference
scenario_simulator communicates scenario_runner by xmlrpc protocol.  
port of the XMLRPC server is defeined by rosparam "~/port".  
all return values of the methods contains "[result/description]" and "[result/return_code]".  
[result/description] is the description of the execution status of the function.  
[result/return_code] is the return code of the function, 0 is success and 1 is fail.  

### Methods
#### initialize
##### parameters
1. "[sim/realtime_factor]" : realtime factor of the simulator
1. "[sim/step_time]" : step time of the simulation

##### return
1. "[sim/initialized]" : if True, the simulator is initialized

##### description
initialize function calls once before you start simulation.  

#### update_frame
##### parameters
1. "[runner/current_time]" : current elapsed simulation time in scenario_runner. if the timestamp does not match, the frame does not update.

##### return
1. "[sim/current_time]" : current simulation time in simulator