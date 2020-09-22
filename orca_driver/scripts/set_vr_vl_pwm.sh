ros2 topic pub -p 100 /control orca_msgs/msg/Control "{thruster_pwm: {fr_1: 1500, fl_2: 1500, rr_3: 1500, rl_4: 1500, vr_5: $1, vl_6: $2}}"
