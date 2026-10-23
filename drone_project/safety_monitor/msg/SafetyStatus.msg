builtin_interfaces/Time timestamp

uint8 UNKNOWN=0
uint8 READY=1
uint8 WARNING=2
uint8 BLOCKED=3
uint8 EMERGENCY=4
uint8 level

uint8 PHASE_UNKNOWN=0
uint8 GROUNDED=1
uint8 FLIGHT=2
uint8 LANDING=3
uint8 VISUAL_LANDING=4
uint8 phase

bool telemetry_healthy
bool preflight_passed
bool battery_healthy
bool position_healthy
bool px4_failsafe_active
bool operator_link_healthy
bool gnss_healthy
bool landed
bool launch_permitted
bool autonomous_flight_permitted
bool visual_landing_permitted

# Reason describes the current phase; each permission has its own explanation.
string reason
string launch_block_reason
string autonomous_flight_block_reason
string visual_landing_block_reason
