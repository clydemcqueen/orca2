ROV and AUV functionality for [Orca2](https://github.com/clydemcqueen/orca2).

***TODO update this***

Modes:
* disarmed: thrusters are off, all joystick buttons except "arm" are ignored
* manual: thrusters are manually controlled
* hold_h: the autopilot controls yaw
* hold_d: the autopilot controls z
* hold_hd: the autopilot controls yaw and z
* mission (AUV mode): the autopilot is running a mission, and will switch to manual when complete

Orca supports tethered and untethered operation.
If you run without a tether, make sure the vehicle is positively buoyant!

The world coordinate frame is ENU (East, North, Up).
Rotations follow the right hand rule.
These are the default used by ROS and Gazebo.
To avoid confusion the code uses names like _z_ and _yaw_ instead of _depth_ and _heading_.

Coordinate frame conventions:
* t_destination_source is a transformation from source frame to destination frame
* xxx_f_destination means xxx is expressed in destination frame

Therefore:
* t_destination_source == source_f_destination
* t_a_c = t_a_b * t_b_c


Forces that we model:
* gravity
* buoyancy
* thruster translation forces
* thruster moments are not modeled (vehicle is designed to cancel thruster moments)
* vehicle drag
* tether drag is not modeled

Control inputs (world frame):
* u_x
* u_y
* u_z
* u_yaw

Control outputs (world frame):
* x_dot_dot
* y_dot_dot
* z_dot_dot
* yaw_dot_dot

Constants:
* m: mass of vehicle
* g: 9.81 m/s^2
* b: buoyancy (weight of displaced water)
* c_drag_xy: drag coefficient for x and y motion
* c_drag_z: drag coefficient for z motion
* c_drag_yaw: drag coefficient for yaw motion

Motion equations:
* x_dot_dot = u_x / m * cos(yaw) - c_drag_xy * x_dot ^ 2
* y_dot_dot = u_y / m * sin(yaw) - c_drag_xy * y_dot ^ 2
* z_dot_dot = u_z / m - c_drag_z * z_dot ^ 2 - g + b
* yaw_dot_dot = u_yaw / inertia - c_drag_yaw * yaw_dot ^ 2
* roll_dot_dot = 0
* pitch_dot_dot = 0