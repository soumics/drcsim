#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
# testing everything a robot needs
std_msgs/Header header

# Default joint indices used when publishing the
# JointCommands joint_states topic below
# For exmaple, if you subscribe to this message, then
# msg.joint_states.position[atlas_msgs::AtlasStates::back_lbz] gives back
# the position of the back_lbz.
int32 BACK_LBZ  = 0 # v1
int32 BACK_BKZ  = 0 # v3, v4
int32 BACK_MBY  = 1 # v1
int32 BACK_BKY  = 1 # v3, v4
int32 BACK_UBX  = 2 # v1
int32 BACK_BKX  = 2 # v3, v4
int32 NECK_AY   = 3 # v1
int32 NECK_RY   = 3 # v3, v4
int32 L_LEG_UHZ = 4 # v1
int32 L_LEG_HPZ = 4 # v3, v4
int32 L_LEG_MHX = 5 # v1
int32 L_LEG_HPX = 5 # v3, v4
int32 L_LEG_LHY = 6 # v1
int32 L_LEG_HPY = 6 # v3, v4
int32 L_LEG_KNY = 7 # v3, v4
int32 L_LEG_UAY = 8 # v1
int32 L_LEG_AKY = 8 # v3, v4
int32 L_LEG_LAX = 9 # v1
int32 L_LEG_AKX = 9 # v3, v4
int32 R_LEG_UHZ = 10 # v1
int32 R_LEG_HPZ = 10 # v3, v4
int32 R_LEG_MHX = 11 # v1
int32 R_LEG_HPX = 11 # v3, v4
int32 R_LEG_LHY = 12 # v1
int32 R_LEG_HPY = 12 # v3, v4
int32 R_LEG_KNY = 13 # v3, v4
int32 R_LEG_UAY = 14 # v1
int32 R_LEG_AKY = 14 # v3, v4
int32 R_LEG_LAX = 15 # v1
int32 R_LEG_AKX = 15 # v3, v4
int32 L_ARM_USY = 16 # v1
int32 L_ARM_SHY = 16 # v3
int32 L_ARM_SHZ = 16 # v4
int32 L_ARM_SHX = 17 # v3, v4
int32 L_ARM_ELY = 18 # v3, v4
int32 L_ARM_ELX = 19 # v3, v4
int32 L_ARM_UWY = 20 # v1
int32 L_ARM_WRY = 20 # v3, v4
int32 L_ARM_MWX = 21 # v1
int32 L_ARM_WRX = 21 # v3, v4
int32 R_ARM_USY = 22 # v1
int32 R_ARM_SHY = 22 # v3
int32 R_ARM_SHZ = 22 # v4
int32 R_ARM_SHX = 23 # v3, v4
int32 R_ARM_ELY = 24 # v3, v4
int32 R_ARM_ELX = 25 # v3, v4
int32 R_ARM_UWY = 26 # v1
int32 R_ARM_WRY = 26 # v3, v4
int32 R_ARM_MWX = 27 # v1
int32 R_ARM_WRX = 27 # v3, v4

# repeating data from osrf_msgs/JointCommands as joint_states
float32[] position
float32[] velocity
float32[] effort
float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity
float32[] i_effort_min
float32[] i_effort_max

uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.


#sensor_msgs/Imu imu 
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration

#atlas_msgs/ForceTorqueSensors force_torque_sensors
geometry_msgs/Wrench l_foot
geometry_msgs/Wrench r_foot
geometry_msgs/Wrench l_hand
geometry_msgs/Wrench r_hand
