# Interface between orca_base and orca_driver

# Header
std_msgs/Header header

# Error for diagnostics
geometry_msgs/Pose error

# 4 DoF efforts for diagnostics
orca_msgs/Efforts efforts

# Stability, range [1.0, 0.0]
float64 stability

# Odom lag, seconds
float64 odom_lag

# Mode
uint8 DISARMED=0            # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 ROV=1                 # ROV: manual thruster control
uint8 ROV_HOLD_PRESSURE=2   # ROV: hold pressure
uint8 AUV=3                 # AUV, see Mission.action for details
uint8 mode

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