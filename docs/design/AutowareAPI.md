# Autoware API

Autoware API provides features to control autoware easily via ROS2 API.  
AWAPI accessor is a C++ wrapper of the Autoware API, and it enables us to integrate with Autoware and other tools very easily!  
We can control autoware while initialize_duration of the simulation. (current_time < 0)

<font color="#065479E">*Note! Autoware API is now under development and we are preparing documentation about this. We provide source code, but it is not a final version.*</font>

```mermaid
sequenceDiagram
    participant Autoware API
    participant Autoware

Autoware API -->+ Autoware : launch
Autoware -->- Autoware API : launch result
alt failed or timeout
    Autoware API ->> Autoware : terminate
end

Autoware API -->+ Autoware : initialize
Autoware -->- Autoware API : initialize result
alt failed
    Autoware API ->> Autoware : terminate
end

Autoware API -->+ Autoware : send goal
Autoware -->- Autoware API : planing result
alt failed
    Autoware API ->> Autoware : terminate
end

loop every simulation frame
    Autoware API ->> Autoware : vehicle status
end

Autoware API ->> Autoware : terminate
```
