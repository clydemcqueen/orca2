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
uint8 RANDOM_PATH=6   # AUV: random path
uint8 mode

# Camera tilt pwm, range [1100, 1900]
# 1100 = tilt 45 degrees up
# 1500 = straight ahead
# 1900 = tilt 45 degrees down
uint16 camera_tilt_pwm

# Light brightness pwm, range [1100, 1900]
# 1100 = lights off
# 1900 = lights on full
uint16 brightness_pwm

# Thruster efforts, range [1100, 1900]
# 1100 = full reverse
# 1500 = stop
# 1900 = full forward
uint16[] thruster_pwm