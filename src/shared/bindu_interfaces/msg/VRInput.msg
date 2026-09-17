# Pose is row-major OpenXR SE(3), metres. Stamp is receiver event time.
# source_id changes on browser reconnect; seq increases only on input events.
builtin_interfaces/Time stamp
string source_id
uint64 seq
string side
float64[16] pose
float64 grip
float64 trigger
bool init
bool stop
bool valid
uint64 clutch_seq
