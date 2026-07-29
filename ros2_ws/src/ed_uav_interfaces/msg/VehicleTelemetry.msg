# D-task wheel-vehicle telemetry. Distances and speeds are SI in vehicle_start.
uint16 CONTRACT_VERSION=1
uint16 contract_version
builtin_interfaces/Time start_stamp
builtin_interfaces/Time acquisition_stamp
uint32 source_sequence
uint16 checksum_crc16
string<=32 vehicle_id
bool start_event
bool heartbeat_alive

uint8 MOTION_DISPLACEMENT=1
uint8 MOTION_WHEEL_SPEED=2
uint8 motion_kind
float32 displacement_m
float32 wheel_speed_m_s

uint8 TURN_STRAIGHT=0
uint8 TURN_SMALL=1
uint8 TURN_LARGE=2
uint8 turn_class

uint8 ROUTE_START=0
uint8 ROUTE_B=1
uint8 ROUTE_D=2
uint8 ROUTE_A=3
uint8 ROUTE_COMPLETE=4
uint8 route_stage
bool lap_complete
string<=32 frame_id
