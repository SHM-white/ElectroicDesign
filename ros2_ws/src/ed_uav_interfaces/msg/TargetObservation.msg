# Acquisition-time observation of the approved D-task platform target.
uint16 CONTRACT_VERSION=1
uint16 contract_version
builtin_interfaces/Time acquisition_stamp
uint32 source_sequence
string<=32 observation_id
string<=32 target_revision
string<=64 frame_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
float32 outer_diameter_m
float32 inner_diameter_m
float32 line_width_m
