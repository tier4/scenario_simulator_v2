uint8 EGO = 0
uint8 VEHICLE = 1
uint8 PEDESTRIAN = 2
uint8 type

uint8 WORLD = 0
uint8 LANE = 1
uint8 coordinate

float64 time

string name

openscenario_msgs/BoundingBox bounding_box

string current_action

# fields for world frame
geometry_msgs/Pose pose

# fields for lane frame
openscenario_msgs/LaneletPose lanelet_pose

# fields for each frame
geometry_msgs/Twist twist
geometry_msgs/Accel accel