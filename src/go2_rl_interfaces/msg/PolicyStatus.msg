uint8 WAITING_FOR_SENSORS=0
uint8 STAND_TRANSITION=1
uint8 POLICY_WARMUP=2
uint8 ACTIVE=3
uint8 FAULT=4

builtin_interfaces/Time stamp
uint8 state
bool policy_loaded
uint32 observation_size
uint32 action_size
string message
