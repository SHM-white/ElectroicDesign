# All SI values are converted from Lingxiao V7 only by ed_uav_fcu_bridge.
builtin_interfaces/Time acquisition_stamp
uint32 source_sequence

uint8 SOURCE_V7=1
uint8 SOURCE_SIMULATOR=2
uint8 source

uint8 MODE_STABILIZE=0
uint8 MODE_ALTITUDE_HOLD=1
uint8 MODE_POSITION_HOLD=2
uint8 MODE_PROGRAM=3
uint8 mode
bool motors_armed
bool communication_ok

# ENU displacement from the FCU's takeoff origin, in meters.
geometry_msgs/Vector3 optical_flow_position_m
float32 altitude_m
float32 battery_voltage_v
uint8 optical_flow_quality
string<=64 frame_id
