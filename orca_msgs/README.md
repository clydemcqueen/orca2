Custom messages for [Orca2](https://github.com/clydemcqueen/orca2).

| Message | Description | Publisher(s) | Subscriber(s) |
|-----|--------|------|-----|
| `Control` | Controls the device hardware (thrusters, lights, camera tilt), includes diagnostic information | `rov_node` `auv_node`  | `driver_node` `plot_control.py` `plot_auv_segments.py` |
| `Driver` | Driver status, including battery voltage and leak status | `driver_node` | `rov_node` `auv_node` |
| `Barometer` | Barometer reading | `driver_node` | `depth_node` `rov_node` |
| `Depth` | Depth measurement | `depth_node` | `filter_node` `auv_node` |
| `FiducialPoseStamped` | ArUco marker observations and the resulting pose | `fp_node` `filter_node` | `filter_node` `auv_node` |


| Action | Description | Server | Client(s) |
|-----|--------|------|-----|
| `Mission` | Run automomous mission | `auv_node` | `rov_node` `experiments.py` |
