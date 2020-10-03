# Interface between orca_base and orca_driver

# Header
std_msgs/Header header

#------------------------
# Diagnostics
#------------------------

# Control mode

uint8 DISARMED=0            # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 ROV=1                 # ROV: manual thruster control
uint8 ROV_HOLD_PRESSURE=2   # ROV: hold pressure
uint8 AUV=3                 # AUV, see Mission.action for details
uint8 mode

# Target pressure (ROV only)

float64 target_pressure

# Mission state (AUV only)
# The tuple {global_plan_idx, mission.target_idx, mission.plan_idx, mission.segment_idx}
# uniquely names a segment while auv_node is running

int32 global_plan_idx
orca_msgs/MissionState mission

# Estimated pose (AUV only)

orca_msgs/FiducialPose estimate     # Pose, observer, observations
int32 covariance_dof                # Information about the filter that produced the pose
bool good_pose                      # True if pose is deemed "good" (x, y, z, yaw are good)
bool has_good_observation           # True if there's a "good" marker observation

# Efforts, range [-1.0, 1.0]

float64 odom_lag                    # Lag from sensor timestamp to now()
orca_msgs/Efforts efforts
bool thruster_saturated

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
orca_msgs/Thrusters thruster_pwm
