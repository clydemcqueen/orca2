ros2 topic pub -p 100 /control orca_msgs/msg/Control "{thruster_pwm: {fr_1: $1, fl_2: $1, rr_3: $1, rl_4: $1, vr_5: $1, vl_6: $1}}"
