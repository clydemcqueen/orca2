# Header
std_msgs/Header header

# Stability, range [1.0, 0.0]
float64 stability

# Odom lag, seconds
float64 odom_lag

# Mode
uint8 DISARMED=0            # Thrusters are off, all joystick buttons except "arm" are ignored
uint8 ROV=1                 # ROV: manual thruster control
uint8 ROV_HOLD_Z=2          # ROV: hold z
uint8 AUV_KEEP_STATION=3    # AUV: hold pose
uint8 AUV_4=4               # AUV: mission #4
uint8 AUV_5=5               # AUV: mission #5
uint8 AUV_6=6               # AUV: mission #6
uint8 AUV_7=7               # AUV: mission #7
uint8 AUV_8=8               # AUV: mission #8
uint8 AUV_9=9               # AUV: mission #9
uint8 AUV_10=10             # AUV: mission #10
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