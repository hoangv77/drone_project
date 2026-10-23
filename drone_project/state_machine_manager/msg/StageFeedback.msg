builtin_interfaces/Time timestamp

# Echo the current MissionState identity. No feedback is accepted by default.
uint64 mission_id
uint64 transition_sequence
# Strictly increasing within this state entry; zero is invalid.
uint64 observation_sequence
bool verified

uint8 THROW_DETECTED=1
uint8 LAUNCH_COMPLETE=2
uint8 STABILIZED=3
uint8 TAG_ACQUIRED=4
uint8 TAG_TRACKED=5
uint8 TAG_LOST=6
uint8 LANDING_READY=7
uint8 STAGE_FAILED=8
uint8 event
string detail
