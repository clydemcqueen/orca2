# Mission status

#--------------
# Global plan status
#--------------

int32 targets_total             # Number of targets
int32 target_idx                # Current target
int32 target_marker_id          # Marker id for this target, -1 if there is no marker

#--------------
# Local plan status
#--------------

uint8 PLAN_NONE=0               # No local planner (temporary state)
uint8 PLAN_LOCAL=1              # Running a local plan to the target
uint8 PLAN_RECOVERY_MTM=2       # Recovery: move to the nearest marker to get a good pose
uint8 PLAN_RECOVERY_360=3       # Recovery: spin 360 to find a marker (future)
uint8 planner

int32 local_plan_idx            # Current attempt at a local plan, starts at 0 for this target

int32 segments_total            # Number of segments in this local plan
int32 segment_idx               # Current segment
string segment_info             # Human-readable segment information

uint8 SEGMENT_NONE=0
uint8 PAUSE=1
uint8 POSE_VERTICAL=2
uint8 POSE_ROTATE=3
uint8 POSE_LINE=4
uint8 POSE_COMBO=5
uint8 OBS_RTM=6
uint8 OBS_MTM=7
uint8 segment_type

#--------------
# Planned pose and twist
#--------------

orca_msgs/FiducialPoseStamped pose
geometry_msgs/Twist twist
