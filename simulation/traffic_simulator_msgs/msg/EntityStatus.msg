traffic_simulator_msgs/EntityType type
traffic_simulator_msgs/EntitySubtype subtype

float64 time 0

string name

traffic_simulator_msgs/BoundingBox bounding_box
traffic_simulator_msgs/ActionStatus action_status

# fields for world frame
geometry_msgs/Pose pose

# fields for lane frame
traffic_simulator_msgs/LaneletPose[] lanelet_poses
