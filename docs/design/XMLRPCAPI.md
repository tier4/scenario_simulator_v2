# XMLRPC API
## XMLRPC API Reference  
scenario_simulator communicates scenario_runner by xmlrpc protocol.  
Simulator has a XMLRPC server and Intepretor has a XMLRPC client.
Port of the XMLRPC server is defeined by rosparam "~/port".  
All return values of the methods contains "[result/description]" and "[result/return_code]".  
[result/description] is the description of the execution status of the function.  
[result/return_code] is the return code of the function, 0 is success and 1 is fail.  

### methods
#### initialize
##### parameters
```
"[sim/realtime_factor]" : Realtime factor of the simulator
"[sim/step_time]" : Step time of the simulation (seconds)
```

If you define these values linke below,
$$
r = realtime\_factor
$$

$$
\delta t = step\_time
$$

One loop of the simulation should be finished in this duration.
$$
r * \delta t
$$

##### return values
```
"[sim/initialized]" : If True, the simulator is initialized
```

##### description
initialize function calls once before you start simulation.  

#### update_frame
##### parameters
```
"[runner/current_time]" : current elapsed simulation time in scenario_runner.
If the timestamp does not match, the frame does not update.
```

##### return values
```
"[sim/current_time]" : current simulation time in simulator
```