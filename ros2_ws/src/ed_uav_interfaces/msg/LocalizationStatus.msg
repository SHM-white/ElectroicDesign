builtin_interfaces/Time acquisition_stamp
uint32 source_sequence

uint8 SOURCE_NONE=0
uint8 SOURCE_LIO=1
uint8 SOURCE_VISUAL_BOUNDARY=2
uint8 SOURCE_FUSED=3
uint8 source

uint8 STATE_UNINITIALIZED=0
uint8 STATE_ACTIVE=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3
uint8 state
float32 age_sec
bool map_to_odom_valid
string<=96 reason
