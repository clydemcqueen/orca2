# `mw`

The `mw` (message wrapper) library provides core classes for Orca.

### Design

* Largely 1:1 object:msg, though msg is optional
* Message round trip: Class{msg}.msg() -> msg
* Default constructable
* May be immutable
* Message composition leads to object composition
* Stream-printable

### Core classes

| Class | Message | Description |
|-----|-----|-----|
| Acceleration | n/a | Acceleration in the world frame |
| AccelerationBody | n/a | Acceleration in the body frame |
| Efforts | orca_msgs::msg::Efforts | Thrust efforts in the range [-1, 1] |
| FiducialPose | orca_msgs::msg::FiducialPose | Observations + PoseWithCovariance |
| FiducialPoseStamped | orca_msgs::msg::FiducialPoseStamped | Header + FiducialPose |
| Header | std_msgs::msg::Header | Time stamp + frame id |
| Map | fiducial_vlam_msgs::msg::Map | Marker[] |
| Marker | n/a | Marker location in the world frame |
| Observation | fiducial_vlam_msgs::msg::Observation | Marker location in the camera frame |
| ObservationStamped | n/a | Header + Observation |
| Observations | fiducial_vlam_msgs::msg::Observations | Observer + Observation[] + PolarObservation[] |
| Observer | orca_msgs::msg::Observer | Converts Observation <=> PolarObservation |
| Point | geometry_msgs::msg::Point | Point |
| PolarObservation | orca_msgs::msg::PolarObservation | Marker location in polar coordinates from base_link |
| PolarObservationStamped | n/a | Header + PolarObservation |
| Pose | geometry_msgs::msg::Pose | Point + Quaternion |
| PoseStamped | geometry_msgs::msg::PoseStamped | Header + Pose |
| PoseWithCovariance | geometry_msgs::msg::PoseWithCovariance | Pose + covariance matrix |
| PoseWithCovarianceStamped | geometry_msgs::msg::PoseWithCovarianceStamped | Header + PoseWithCovariance |
| Target | n/a | Target pose for AUV, often near a marker |
| Twist | geometry_msgs::msg::Twist | Velocity in the world frame |
| TwistBody | n/a | Velocity in the body frame |
| Quaternion | geometry_msgs::msg::Quaternion | Quaternion |

### Unwrapped messages

| Message | Description |
|-----|-----|
| orca_msgs::msg::Barometer | Barometer reading |
| orca_msgs::msg::Control | Driver control |
| orca_msgs::msg::Depth | Depth reading |
| orca_msgs::msg::Driver | Driver status |
