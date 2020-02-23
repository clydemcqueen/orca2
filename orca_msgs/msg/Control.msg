# Interface between orca_base and orca_driver

# Header
std_msgs/Header header

#------------------------
# Status
#------------------------

# Plan in world frame
geometry_msgs/Pose plan_pose
geometry_msgs/Twist plan_twist

# Estimate in world frame
geometry_msgs/Pose estimate_pose

# Efforts in world frame, range [-1.0, 1.0]
orca_msgs/Efforts efforts

# Stability, range [1.0, 0.0]
float64 stability

# Odom lag, seconds
float64 odom_lag

# Control mode
uint8 DISARMED=0            # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 ROV=1                 # ROV: manual thruster control
uint8 ROV_HOLD_PRESSURE=2   # ROV: hold pressure
uint8 AUV=3                 # AUV, see Mission.action for details
uint8 mode

# Global planner status
int32 targets_total
int32 target_idx
int32 target_marker_id

# Local planner
uint8 PLAN_NONE=0           # No local planner
uint8 PLAN_LOCAL=1          # Running a local plan to the target
uint8 PLAN_RECOVERY_MTM=2   # Recovery: move to the nearest marker to get a good pose
uint8 PLAN_RECOVERY_360=3   # Recovery: spin 360 to find a marker (future)
uint8 planner

# Local planner status
int32 segments_total
int32 segment_idx
string segment_info

#------------------------
# Control
#------------------------

# Camera tilt pwm, range [1100, 1900]
uint16 TILT_45_UP=1100
uint16 TILT_0=1500
uint16 TILT_45_DOWN=1900
uint16 camera_tilt_pwm

# Light brightness pwm, range [1100, 1900]
uint16 LIGHTS_OFF=1100
uint16 LIGHTS_FULL=1900
uint16 brightness_pwm

# Thruster efforts, range [1100, 1900]
uint16 THRUST_FULL_REV=1100
uint16 THRUST_STOP=1500
uint16 THRUST_FULL_FWD=1900
uint16[] thruster_pwm