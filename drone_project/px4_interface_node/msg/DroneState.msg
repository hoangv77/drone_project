builtin_interfaces/Time last_update_time

bool armed

# Last-known PX4 status; only authoritative while status_fresh is true.
uint8 arming_state
uint8 nav_state
uint8 nav_state_user_intention
bool failsafe
bool pre_flight_checks_pass
bool gcs_connection_lost
bool safety_off
bool calibration_enabled
uint8 system_id
uint8 component_id

# Land detection has its own source and freshness gate.
bool landed
bool land_detected_fresh

# Conservative local command readiness, including fresh landed telemetry.
bool ready_to_arm
string arming_block_reason
bool command_pending

string flight_mode

bool position_valid
float32 x
float32 y
float32 z
float32 vx
float32 vy
float32 vz

bool gps_valid
float32 battery_remaining

bool status_fresh
bool local_position_fresh
bool odometry_fresh
bool gps_fresh
bool battery_fresh
