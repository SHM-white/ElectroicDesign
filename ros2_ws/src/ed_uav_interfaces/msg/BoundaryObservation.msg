# Header stamp is acquisition time. Pose components outside observable_dof_mask
# are unspecified and consumers must not use them.
std_msgs/Header header
uint32 source_sequence

uint8 DOF_X=1
uint8 DOF_Y=2
uint8 DOF_Z=4
uint8 DOF_ROLL=8
uint8 DOF_PITCH=16
uint8 DOF_YAW=32
uint8 observable_dof_mask
uint8 constraint_count
float32 confidence
geometry_msgs/PoseWithCovariance pose
string<=64 association_id
