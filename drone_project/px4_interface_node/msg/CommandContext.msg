# Internal authorization facts from the safety monitor and mission coordinator.
builtin_interfaces/Time stamp

bool telemetry_healthy
bool launch_permitted
bool bootstrap_permitted
bool recovery_eligible

uint8 IDLE=0
uint8 PREFLIGHT=1
uint8 RECOVERY=2
uint8 OTHER=3
uint8 phase
