# Header
std_msgs/Header header

# Stability, range [1.0, 0.0]
float64 stability

# Odom lag, seconds
float64 odom_lag

# Mode
uint8 DISARMED=0      # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 MANUAL=1        # ROV: manual thruster control
uint8 HOLD_H=2        # ROV: hold heading
uint8 HOLD_D=3        # ROV: hold depth
uint8 HOLD_HD=4       # ROV: hold heading and depth
uint8 KEEP_STATION=5  # AUV: hold position
uint8 MISSION_6=6     # AUV: mission #6
uint8 MISSION_7=7     # AUV: mission #7
uint8 MISSION_8=8     # AUV: mission #8
uint8 MISSION_9=9     # AUV: mission #9
uint8 MISSION_10=10   # AUV: mission #10
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