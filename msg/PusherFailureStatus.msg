# Manual standard-VTOL pusher failure recovery, owned by Navigator.
uint64 timestamp # [us] Time since system start
uint8 state
uint8 STATE_IDLE = 0
uint8 STATE_GLIDING = 1
uint8 STATE_QUADCHUTE = 2
uint8 STATE_LANDING = 3
uint8 STATE_ABORTED = 4
uint8 STATE_LANDED = 5
bool pusher_failed # Latched until disarm, including after recovery is interrupted
float32 recovery_height # [m] VT_FW_QC_HMAX captured at trigger time
float32 height # [m] Current distance to ground, otherwise height above home; NaN if unavailable
bool terrain_height # True when height comes from valid distance-to-ground
