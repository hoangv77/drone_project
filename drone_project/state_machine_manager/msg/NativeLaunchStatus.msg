# SITL-only, read-only observation from PX4's actuator_armed uORB topic.
# Freshness is checked by steady receipt time; timestamp also prevents replay.
uint64 px4_timestamp
bool armed
bool lockdown
bool kill
bool termination
