# Operator-visible D-task state owned by ed_uav_mission.
uint16 CONTRACT_VERSION=1
uint16 contract_version
builtin_interfaces/Time acquisition_stamp
uint32 source_sequence
string<=64 mission_id

uint8 STATE_PRE_ARM=0
uint8 STATE_TAKEOFF=1
uint8 STATE_SEARCHING=2
uint8 STATE_ACCOMPANYING=3
uint8 STATE_PAYLOAD_DROP=4
uint8 STATE_LANDING_ON_VEHICLE=5
uint8 STATE_VEHICLE_DWELL=6
uint8 STATE_RETURNING_HOME=7
uint8 STATE_LANDING_HOME=8
uint8 STATE_SUCCEEDED=9
uint8 STATE_ABORTED=10
uint8 state
uint8 route_stage
bool complete
string<=96 reason
