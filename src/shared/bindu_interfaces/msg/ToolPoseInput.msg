# Absolute tool pose after VR mapping. Row-major SE(3), metres, robot base.
# Only pose and tight are action inputs. The remaining fields fence stale data.
builtin_interfaces/Time stamp
string source_id
string runtime_mode
uint64 seq
string frame_id
float64[16] pose
# false: right axial support; left stays locked. Not a full gripper release.
bool tight
bool valid
bool stop
