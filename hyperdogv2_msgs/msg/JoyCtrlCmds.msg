# `header`
std_msgs/Header header

# `bool states of the robot`
bool start
bool walk
bool side_move_mode
uint8 gait_type

# `robot's pose`
# pose.position: displacement of center of mass of the robot from home position: displacement_x, displacement_y, height
# pose.orientation: roll, pitch, yaw
geometry_msgs/Pose pose

# `robot's gait parameters`
# cycle time of the gait
float32 gait_cycle_time
# gait_step.x = steplen_x
# gait_step.y = steplen_y
# gait_step.z = swing_height
geometry_msgs/Vector3 gait_step
