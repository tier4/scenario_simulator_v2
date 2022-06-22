# Autoware API

Autoware APIs provide features to control autoware easily via ROS 2 APIs.  
`concealer::Autoware` is a C++ wrapper of the Autoware API, and it enables us to integrate Autoware with other tools very easily!  
We can control autoware while initialize_duration of the simulation. (current_time < 0)

<font color="#065479E">*Note! Autoware APIs are now under development, and we are preparing documentation about this. The current code is not a final version, might be changed in the future.*</font>

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
Autoware -->- Autoware API : planning result
alt failed
    Autoware API ->> Autoware : terminate
end

loop every simulation frame
    Autoware API ->> Autoware : vehicle status
end

Autoware API ->> Autoware : terminate
```
