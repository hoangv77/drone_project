builtin_interfaces/Time timestamp

uint8 STANDBY=0
uint8 PREFLIGHT=1
uint8 ARMED_WAITING=2
uint8 THROW_LAUNCH=3
uint8 STABILIZE=4
uint8 SEARCH_TAG=5
uint8 TRACK_TAG=6
uint8 PRECISION_LAND=7
uint8 RECOVERY=8
uint8 ABORTED=9
uint8 FAULT=10
uint8 current_state
uint8 previous_state
string state_name
string transition_reason

uint8 RESULT_NONE=0
uint8 RESULT_RUNNING=1
uint8 RESULT_SUCCEEDED=2
uint8 RESULT_ABORTED=3
uint8 RESULT_FAILED=4
uint8 mission_result
uint64 mission_id
uint64 transition_sequence
bool mission_active
bool abort_requested
bool telemetry_fresh
bool safety_fresh
bool stage_feedback_enabled
uint64 state_elapsed_ms

# Informational entry action; never a flight-control command.
string entry_action
string last_rejection
uint64 rejected_requests
