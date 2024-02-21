# Lane pose calculation when getting longitudinal distance

Attempts to calculate the pose for adjacent lane coordinate systems when measuring longitudinal distance.  
The length of the horizontal bar must intersect with the adjacent lanelet, [so it is always 10 m regardless of the entity type.](https://github.com/tier4/scenario_simulator_v2/blob/5f19d39ef29243396f26225976975f0c27914c12/simulation/traffic_simulator/src/entity/entity_manager.cpp#L375C21-L442)

![Get longitudinal distance](../../image/longitudinal_distance.png "Getting longitudinal distance.")

The shorter of the two blue arrows shown in the figure above is the longitudinal distance between entities.
