# Orca messages and actions

## Messages

| Message | Description | Publisher(s) | Subscriber(s) |
|-----|--------|------|-----|
| Barometer | Barometer reading | driver_node | depth_node, rov_node |
| Control | Controls the device hardware, includes mission state | rov_node, auv_node  | driver_node, plot_control.py, plot_auv_segments.py |
| Depth | Depth reading | depth_node | filter_node, auv_node |
| Driver | Driver status, includes battery voltage and leak status | driver_node | rov_node, auv_node |
| Efforts | Thrust efforts in the range [-1, 1] |
| FiducialPose | Marker observations and the resulting pose |
| FiducialPoseStamped | Stamped FiducialPose | fp_node, filter_node | filter_node, auv_node |
| MissionState | Mission state |
| Observations | Marker observations, includes Observer |
| Observer | Information about the camera making the observations |
| PolarObservation | Marker location in polar coordinates from base_link |
| PoseBody | 4 DoF pose in the body frame |
| Thrusters | Thruster pwm values |

## Action

| Action | Description | Server | Client(s) |
|-----|--------|------|-----|
| Mission | Run automomous mission | auv_node | rov_node experiments.py |
