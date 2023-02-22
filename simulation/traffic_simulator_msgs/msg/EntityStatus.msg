traffic_simulator_msgs/EntityType type
traffic_simulator_msgs/EntitySubtype subtype

float64 time 0

string name

traffic_simulator_msgs/BoundingBox bounding_box
traffic_simulator_msgs/ActionStatus action_status

# fields for world frame
geometry_msgs/Pose pose

# fields for lane frame
traffic_simulator_msgs/LaneletPose lanelet_pose
bool lanelet_pose_valid true

# fields for lane frame
# [<=1] means array with maximu length is 1
# If the length of the array is 0, it means calculating lanelet pose is failed.
# If the length of the array is 1, the value of lanelet_pose[0] is the pose in lane coordinate.
# This idiom was suggested in this thread.(https://discourse.ros.org/t/optional-fields-in-message/991/16)
# traffic_simulator_msgs/LaneletPose[<=1] lanelet_pose
