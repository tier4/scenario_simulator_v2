uint8 EGO = 0
uint8 VEHICLE = 1
uint8 PEDESTRIAN = 2
uint8 type

uint8 WORLD = 0
uint8 LANE = 1
uint8 coordinate

float64 time

string name

# fields for world frame
geometry_msgs/Pose pose

# fields for lane frame
int32 lanelet_id
float64 s
float64 offset
geometry_msgs/Vector3 rpy

# fields for each frame
geometry_msgs/Twist twist
geometry_msgs/Accel accel