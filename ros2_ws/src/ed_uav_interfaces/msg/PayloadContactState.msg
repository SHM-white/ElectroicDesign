# Payload retention and landing-contact state in base_link.
uint16 CONTRACT_VERSION=1
uint16 contract_version
builtin_interfaces/Time acquisition_stamp
uint32 source_sequence

uint8 PAYLOAD_UNKNOWN=0
uint8 PAYLOAD_SECURED=1
uint8 PAYLOAD_RELEASED=2
uint8 payload_state

uint8 CONTACT_AIRBORNE=0
uint8 CONTACT_HOME=1
uint8 CONTACT_VEHICLE=2
uint8 contact_state
bool contact_stable
float32 contact_duration_s
string<=64 owner
string<=64 frame_id
